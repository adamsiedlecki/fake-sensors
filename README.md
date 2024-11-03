Komendy:

```bash
colcon build --packages-select fake-sensors

source /opt/ros/jazzy/setup.bash


ros2 run fake-sensors fake_lidar --ros-args -p csv_lidar_mock_file:=mock-lidar.csv -p sampling_frequency:=1.0 -p csv_std_dev:=0.1

ros2 run fake-sensors fake_camera --ros-args -p mp4:=not-exist.mp4 -p fps:=29

```

