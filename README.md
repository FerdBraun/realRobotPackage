1. Install ROS2 JAZZY([Video guide fox foxy](https://www.youtube.com/watch?v=uWzOk0nkTcI))
2. Create a folder "ros_dev" for excample
3. make an ```src``` dir in it
5. Build an empty workspace by running ```colcon build --symlink-install``` while in ```ros_dev``` dir
6. Open ```src``` dir and clone this repo
7. Go to the ```ros_dev``` dir and rebuild the workspace by running the same command
8. source the dir by running ```source install/setup.bash```


dependencies:
1.
```
sudo apt-get install freeglut3-dev
sudo apt install libusb-dev
sudo apt-get update
sudo apt-get install libusb-1.0-0-dev
```
2.
```
sudo apt-get install ros-jazzy-depth-image-proc
sudo apt-get install ros-jazzy-camera-info-manager
```
3.
```
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-controller-manager
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-tf-transformations
```
4.
```
sudo pip3 install transforms3d
pip install pynput
```
5.
```
sudo apt install ros-jazzy-twist-mux
sudo apt install ros-jazzy-rtabmap-*
sudo apt install ros-jazzy-imu-tools
```
6.
```
sudo apt install ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-rviz-plugins
```

RUN
Kinect
```
ros2 launch kinect_ros2 pointcloud.launch.py 
```
RTABMap
```
ros2 launch real rmap.launch.py 
```
LIDAR
```
ros2 launch real lidar.launch.py 
```
Moving Nodes (gamepad)
```
ros2 launch real movement.launch.py 
```
SERVO
```
python3 server.py
```



sourses:

https://github.com/fadlio/kinect_ros2?tab=readme-ov-file

