#include <limits>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>

class LaserFilterNode : public rclcpp::Node
{
public:
  LaserFilterNode()
  : Node("laser_filter_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&LaserFilterNode::scanCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(), "LaserFilterNode has been started.");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan filtered_scan = *msg;
    std::vector<float> &original = msg->ranges;
    std::vector<float> &values = filtered_scan.ranges;
    const long len = values.size();
    for (long i = 0; i < len; i++){
        float min = std::numeric_limits<float>::infinity();
        for (long j = -2; j < 3; j++){
            long eff_ix = (i + j + len) % len;
            float v = original.at(eff_ix);
            min = std::min(v, min);
        }
        values.at(i) = min;
    }

    publisher_->publish(filtered_scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserFilterNode>());
  rclcpp::shutdown();
  return 0;
}
