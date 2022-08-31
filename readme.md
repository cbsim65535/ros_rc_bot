pip install adafruit-pca9685

<!-- pip install RPi.GPIO -->
sudo apt-get install python-rpi.gpio python3-rpi.gpio -y

colcon build --packages-above ydlidar_ros2_driver --packages-above laser_scan_merger