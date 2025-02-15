## Start simulation:
```bash
ros2 launch unity_sim unity_sim.launch.py scene:=ARCh2024
```
Remember to run the scene in unity

## Run our slam2 package
```bash
ros2 launch kalman_bringup tarch_slam2.launch.py
```

## Run point cloud concatenation
```bash
ros2 launch pointcloud_concatenate_ros2 concat2.py
```

## Convert pointcloud to laserscan
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node use_inf:='0' scan_time:='0.1' --ros-args --remap cloud_in:=/filtered_cloud -r /scan:=/scan_raw
```

# Run laser filtering
```bash
ros2 run kalman_slam2 laser_filter_node
```

# Run slam toolbox
```bash
ros2 launch kalman_slam2 auto_mapper.launch.py map_path:=/
```
