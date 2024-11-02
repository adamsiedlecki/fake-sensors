Komendy:

```bash
colcon build --packages-select fake-sensors

ros2 run fake-sensors fake_lidar --ros-args -p csv_lidar_mock_file:=mock-lidar.csv -p sampling_frequency:=1.0 -p csv_std_dev:=0.1

```

