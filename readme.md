pip install adafruit-pca9685

<!-- pip install RPi.GPIO -->
sudo apt-get install python-rpi.gpio python3-rpi.gpio -y


colcon build --packages-skip ydlidar_ros2_driver laser_scan_merger
colcon build --packages-select odom_node
colcon build --packages-select rc_bringup
colcon build --packages-select laser_scan_merger