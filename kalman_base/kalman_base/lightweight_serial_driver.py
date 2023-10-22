#!/usr/bin/env python3
import struct
from serial import Serial, SerialException
from enum import Enum
from typing import List, Deque
from dataclasses import dataclass

from rclpy.node import Node
from std_msgs.msg import String, Int16, UInt8MultiArray

UInt8 = int


@dataclass
class UartMsg:
    cmd: UInt8
    argc: UInt8
    argv: List[UInt8]


class BinaryParserState(Enum):
    IDLE = 0
    CMD = 1
    LEN = 2
    HRC = 3
    DATA = 4
    CRC = 5
    URC = 6


class LightweightSerialDriver(Node):
    def __init__(self, _port_name='/dev/ttyTHS2', _START_BYTE='<', _STOP_BYTE='>', _BAUD=115200, tick_sleep_time_s=0.01, ascii_mode=False) -> None:
        """
        Initializes a new instance of the LightweightSerialDriver class.

        Args:
            _port_name (str, optional): The name of the serial port to use. Defaults to '/dev/ttyTHS2'.
            _START_BYTE (str, optional): The start byte used to delimit messages. Defaults to '<'.
            _STOP_BYTE (str, optional): The stop byte used to delimit messages. Defaults to '>'.
            _BAUD (int, optional): The baud rate to use. Defaults to 115200.
            tick_sleep_time_s (float, optional): The amount of time to sleep between ticks. Defaults to 0.01.
            ascii_mode (bool, optional): Whether to use ASCII mode. Defaults to False.
        """
        super().__init__('lightweight_serial_driver')

        self.serial_read_buffer: bytes = b''
        self.serial_write_buffer: Deque[bytes] = Deque()
        self.ros_msg_read_buffer: List[UartMsg] = []
        self._START_BYTE: str = _START_BYTE
        self._STOP_BYTE: str = _STOP_BYTE
        self._START_BYTE_ENCODED = self._START_BYTE.encode()
        self._STOP_BYTE_ENCODED = self._STOP_BYTE.encode()

        self.serial = Serial(port=_port_name, baudrate=_BAUD, timeout=0.01)

        self.tick_sleep_rate = self.create_rate(1/tick_sleep_time_s)

        self.malformed_packets_timer_time = 30
        self.timer_log_data = self.create_timer(1, self.log_data_speed)
        self.timer_log_malformed_packets = self.create_timer(self.malformed_packets_timer_time, self.log_malformed_packets)


        self.baudrate_debug_pub = self.create_publisher(String, "/kalman_rover/baudrate_debug", 10)
        self.malformed_packets_pub = self.create_publisher(Int16, f"/kalman_rover/malformed_packets_last_{self.malformed_packets_timer_time}_secs", 10)
        self.bitrate_tx = 0
        self.bitrate_rx = 0
        self.correct_bitrate_rx = 0
        self.malformed_packets = 0

        self.ascii_mode = ascii_mode

        self.binary_parser_state: BinaryParserState = BinaryParserState.IDLE
        self.binary_parser_to_read = 0
        self.binary_parser_msg_buffer: List[UInt8] = []
        self.binary_parser_crc_correct = False

    def log_malformed_packets(self, event):
        """
        Logs the number of malformed packets received in the last malformed_packets_timer_time seconds.
        Used as callback for a timer.

        Args:
            event (TimerEvent): The timer event.

        Returns:
            None
        """
        msg = f"malformed packets in last {self.malformed_packets_timer_time} seconds: {self.malformed_packets}"
        self.baudrate_debug_pub.publish(msg)
        self.malformed_packets_pub.publish(self.malformed_packets)
        self.malformed_packets = 0

    def log_data_speed(self, event):
        """
        Logs the number of bytes read and written per second.
        Used as callback for a timer.

        Args:
            event (TimerEvent): The timer event.

        Returns:
            None
        """
        self.baudrate_debug_pub.publish(f"data TX {self.bitrate_tx}")
        self.baudrate_debug_pub.publish(f"data RX {self.bitrate_rx}")
        self.baudrate_debug_pub.publish(
            f"correct data RX {self.correct_bitrate_rx}")
        self.bitrate_tx = 0
        self.bitrate_rx = 0
        self.correct_bitrate_rx = 0

    def write_msg(self, msg: UInt8MultiArray):
        """
        Write a message to the serial_write_buffer.
        
        Args:
            msg (UInt8MultiArray): The message to write.

        Returns:
            None
        """
        if (self.ascii_mode):
            encoded_msg = self._encode_msg(msg)
        else:
            encoded_msg = self._encode_msg_binary(msg)
        self.serial_write_buffer.append(encoded_msg)

    def tick(self):
        """
        Read and write bytes from/to the serial device.
        """
        self._write_to_serial()
        self.tick_sleep_rate.sleep()
        if (self.ascii_mode):
            self._read_from_serial()
        else:
            self._read_from_serial_binary()

    def read_all_msgs(self) -> List[UartMsg]:
        """
        Read all messages from the ros_msg_read_buffer.

        Returns:
            List[UartMsg]: The messages read from the ros_msg_read_buffer.
        """
        msgs = self.ros_msg_read_buffer[:]
        self.ros_msg_read_buffer = []
        return msgs

    def _init_port(self):
        """
        Initialize the serial port.
        """
        on_error_sleep_rate = self.create_rate(1)
        while True:
            try:
                self.serial.open()
                break
            except SerialException as e:
                self.get_logger().error(e)
                on_error_sleep_rate.sleep()

    def _read_from_serial(self):
        """
        Read bytes from serial using _validate_decode_and_store_packet to parse them.

        Gets all bytes from the serial device and stores them in the serial_read_buffer.
        For each start byte found, it tries to parse a packet.
        """
        if not self.serial.in_waiting:
            return

        new_bytes = self.serial.read_all()
        self.bitrate_rx += 8 * len(new_bytes)
        self.serial_read_buffer += new_bytes
        start_byte_idx = self.serial_read_buffer.find(self._START_BYTE_ENCODED)
        iterations = 0
        # iterations -- sanity check in case of unexpected blocking
        while not start_byte_idx == -1 and iterations < 20:
            iterations += 1
            self.serial_read_buffer = self.serial_read_buffer[start_byte_idx:]
            stop_byte_idx = self.serial_read_buffer.find(
                self._STOP_BYTE_ENCODED)
            if stop_byte_idx != -1:
                packet = self.serial_read_buffer[:stop_byte_idx+1]
                start_byte_idx = packet[1:].find(self._START_BYTE_ENCODED)

                # found start byte in the middle of the packet -> malformed packet
                if start_byte_idx != -1 and start_byte_idx < stop_byte_idx:
                    self.serial_read_buffer = self.serial_read_buffer[start_byte_idx:]
                    iterations += 1
                    self.malformed_packets += 1
                    self.get_logger().error("malformed packet, double start byte without end byte")
                    continue
                self._validate_decode_and_store_packet(packet)
                self.serial_read_buffer = self.serial_read_buffer[stop_byte_idx+1:]
                start_byte_idx = self.serial_read_buffer.find(
                    self._START_BYTE_ENCODED)
            else:
                break
        if iterations == 20:
            self.serial_read_buffer = b''
            self.get_logger().error("iterations == 20, that is not a good sign")

    def _validate_decode_and_store_packet(self, packet: bytes):
        """
        Validate, decode and store a packet given in ASCII format. 
        
        Decodes from ASCII hex to integers.
        Saves the decoded packet in the ros_msg_read_buffer.

        Args:
            packet (bytes): The packet to validate, decode and store.

        Returns:
            None
        """
        if len(packet) < 2:
            self.get_logger().error("received packet smaller than 2 ascii signs")
            self.malformed_packets += 1
            return
        
        packet = packet[1:-1]
        result: List[UInt8] = []
        for i in range(0, len(packet), 2):
            try:
                byte = int(packet[i:i+2], 16)
            except Exception as e:
                self.get_logger().error(e)
                byte = 0
            result.append(byte)

        if len(result) < 2:
            self.get_logger().error("received packet smaller than 2 bytes")
            self.malformed_packets += 1
            return
        
        if self._crc_valid(result):
            # *8*2 bo zamieniamy bity na bajty, a kazdy bajt jest kodowany jako 2 znaki ascii
            # dodajemy 2*8 bo to '<' i '>'
            self.correct_bitrate_rx += len(result)*8*2 + 2 * 8
            uart_msg = UartMsg(
                cmd=result[0], argc=result[1], argv=result[2:-1]
            )
            self.ros_msg_read_buffer.append(uart_msg)
        else:
            self.malformed_packets += 1

    def _read_from_serial_binary(self):
        """
        Read bytes from the serial device using _parse_packet_binary to parse them.

        Returns:
            None
        """
        if not self.serial.in_waiting:
            return

        new_bytes = self.serial.read_all()
        self.bitrate_rx += 8 * len(new_bytes)

        # Process packet byte by byte
        for byte in new_bytes:
            self._parse_packet_binary(byte)

    def _parse_packet_binary(self, byte: UInt8):
        """
        Parse a packet byte by byte. Using a state machine to keep track of the current state.

        State machine states:
        IDLE: Waiting for start byte
        CMD: Waiting for command byte
        LEN: Waiting for length byte
        HRC: Waiting for header control byte
        DATA: Waiting for data bytes
        CRC: Waiting for CRC byte
        URC: Waiting for user return control byte

        In case of a malformed packet, the state machine will return to the IDLE state.
        Final messages are stored in the ros_msg_read_buffer.

        Args:
            byte (UInt8): The byte to parse.

        Returns:
            None
        """

        if self.binary_parser_state == BinaryParserState.IDLE:
            if byte == self._START_BYTE_ENCODED[0]:
                self.binary_parser_state = BinaryParserState.CMD

                # Reset parser to process new message
                self.binary_parser_to_read = 0
                self.binary_parser_msg_buffer.clear()

        elif self.binary_parser_state == BinaryParserState.CMD:
            self.binary_parser_msg_buffer.append(byte)

            self.binary_parser_state = BinaryParserState.LEN

        elif self.binary_parser_state == BinaryParserState.LEN:
            self.binary_parser_msg_buffer.append(byte)
            self.binary_parser_to_read = byte

            self.binary_parser_state = BinaryParserState.HRC

            # check if packet is not too long, only if not Custom packet
            if byte > 16 and (self.binary_parser_msg_buffer[-2] < 128 or self.binary_parser_msg_buffer[-2] > 131):
                self.get_logger().error(
                    f"Packet too long, not dropping {self.binary_parser_msg_buffer}")
                self.malformed_packets += 1
                # self.binary_parser_state = BinaryParserState.IDLE

        elif self.binary_parser_state == BinaryParserState.HRC:
            if byte == self._calc_hrc(self.binary_parser_msg_buffer):
                if self.binary_parser_msg_buffer[-1] > 0:
                    self.binary_parser_state = BinaryParserState.DATA
                else:
                    self.binary_parser_state = BinaryParserState.CRC
            else:
                self.get_logger().error(
                    f"HRC not valid for packet {self.binary_parser_msg_buffer}")
                self.get_logger().error(
                    f"HRC byte is {byte} , should be {self._calc_hrc(self.binary_parser_msg_buffer)}")
                
                self.malformed_packets += 1
                self.binary_parser_state = BinaryParserState.IDLE

        elif self.binary_parser_state == BinaryParserState.DATA:
            self.binary_parser_msg_buffer.append(byte)

            self.binary_parser_to_read -= 1
            if self.binary_parser_to_read == 0:
                self.binary_parser_state = BinaryParserState.CRC

        elif self.binary_parser_state == BinaryParserState.CRC:
            if byte == self._calc_crc(self.binary_parser_msg_buffer):
                self.binary_parser_crc_correct = True
                self.binary_parser_state = BinaryParserState.URC
            else:
                self.get_logger().error(
                    f"CRC not valid for packet {self.binary_parser_msg_buffer}")
                self.get_logger().error(
                    f"CRC byte is {byte} , should be {self._calc_crc(self.binary_parser_msg_buffer)}")
                
                self.malformed_packets += 1

                self.binary_parser_crc_correct = False
                self.binary_parser_state = BinaryParserState.URC
        elif self.binary_parser_state == BinaryParserState.URC:
            if not self.binary_parser_crc_correct:
                pass
            elif byte == self._calc_urc(self.binary_parser_msg_buffer):
                self.correct_bitrate_rx += len(
                    self.binary_parser_msg_buffer)*8 + 8

                uart_msg = UartMsg(
                    cmd=self.binary_parser_msg_buffer[0], argc=self.binary_parser_msg_buffer[
                        1], argv=self.binary_parser_msg_buffer[2:]
                )
                self.ros_msg_read_buffer.append(uart_msg)

            else:                
                self.get_logger().error(
                    f"URC not valid for packet {self.binary_parser_msg_buffer}")
                self.get_logger().error(
                    f"URC byte is {byte} , should be {self._calc_urc(self.binary_parser_msg_buffer)}")
                
                self.malformed_packets += 1

            self.binary_parser_state = BinaryParserState.IDLE

    def _write_to_serial(self):
        """
        Write bytes from the serial_write_buffer to the serial device.

        Returns:
            None
        """
        if len(self.serial_write_buffer) > 0:
            bytes_to_send = self.serial_write_buffer.popleft()
            self.bitrate_tx += len(bytes_to_send)*8
            self.serial.write(bytes_to_send)

    def _encode_msg(self, msg: UInt8MultiArray) -> bytes:
        """
        Encode a ROS message as a packet. The packet is encoded as follows:
        [START_BYTE, CMD, LEN, ARG_0, ..., ARG_N, CRC, STOP_BYTE]

        Args:
            msg (UInt8MultiArray): The ROS message to encode.

        Returns:
            bytes: The encoded message.
        """
        data: List[UInt8] = list(msg.data)
        crc = self._calc_crc(data)
        data.append(crc)
        payload = "".join([self._byte2hex(byte) for byte in data])
        payload = self._START_BYTE + payload + self._STOP_BYTE
        return payload.encode('ascii')

    def _byte2hex(self, _byte: UInt8) -> bytes:
        """
        Convert a byte to a hex string, without the '0x' prefix, and padded to 2 characters,
        so there are leading zeros. (e.g. 0x0A -> '0A')
        
        Args:
            _byte (UInt8): The byte to convert.

        Returns:
            bytes: The hex string.
        """
        return hex(_byte)[2:].rjust(2, '0')

    def _encode_msg_binary(self, msg: UInt8MultiArray) -> bytes:
        """
        Encode a ROS message as a binary packet. The packet is encoded as follows:
        [START_BYTE, CMD, LEN, HRC, ARG_0, ..., ARG_N, CRC, URC]

        Args:
            msg (UInt8MultiArray): The ROS message to encode.

        Returns:
            bytes: The encoded message.
        """
        data: List[UInt8] = list(msg.data)
        hrc = self._calc_hrc(data)
        crc = self._calc_crc(data)
        urc = self._calc_urc(data)
        data = [*data[0:2], hrc, *data[2:]]
        data.append(crc)
        data.append(urc)
        payload = [ord(self._START_BYTE), *data]
        return struct.pack("B"*len(payload), *payload)

    def _calc_hrc(self, data: List[UInt8]) -> UInt8:
        """
        Calculate control and return control sum for a new packet.

        Args:
            data (List[UInt8]): The data to calculate the HRC for.

        Returns:
            UInt8: The HRC.
        """
        hrc = 0x00
        for arg in data[0:2]:
            hrc += arg
            hrc %= 256
        return hrc % 256

    def _calc_crc(self, data: List[UInt8]) -> UInt8:
        """
        Calculate control and return control sum for a new packet.

        Args:
            data (List[UInt8]): The data to calculate the CRC for.

        Returns:
            UInt8: The CRC.
        """
        crc = 0x00
        for arg in data:
            crc ^= arg
        return crc

    def _calc_urc(self, data: List[UInt8]) -> UInt8:
        """
        Calculate control and return control sum for a new packet.

        Args:
            data (List[UInt8]): The data to calculate the URC for.

        Returns:
            UInt8: The URC.
        """
        crc = 0x00
        for i, arg in enumerate(data[2:]):
            crc += (((arg + i) * ((i % 4) + 1)))
            crc %= 256
        return crc % 256

    def _crc_valid(self, data: List[UInt8]) -> bool:
        """
        Check if the CRC of the data is valid.

        Args:
            data (List[UInt8]): The data to check the CRC of (last element is CRC).

        Returns:
            bool: Whether the CRC is valid.
        """
        crc = self._calc_crc(data[:-1])
        return crc == data[-1]
