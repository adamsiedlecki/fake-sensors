Komendy:

```bash
colcon build --packages-select fake-sensors

# to jest ten katalog install w katalogu projektowym fake-sensors
source install/setup.bash


ros2 run fake-sensors fake_lidar --ros-args -p csv_lidar_mock_file:=mock-lidar.csv -p sampling_frequency:=1.0 -p csv_std_dev:=0.1

// bardzo istotna jest rodzielczość
// prawdziwa kamera
ros2 run fake-sensors fake_camera --ros-args -p mp4:=not-exist.mp4 -p fps:=30 -p width:=640 -p height:=480

// mock kamery z plikiem me.mp4
ros2 run fake-sensors fake_camera --ros-args -p mp4:=me.mp4 -p fps:=15 -p width:=1280 -p height:=720

```

