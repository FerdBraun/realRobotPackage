1. Install ROS2 (foxy for 20.04) or other ([Video guide](https://www.youtube.com/watch?v=uWzOk0nkTcI))
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
sudo apt install ros-jazzy-nav2-*
sudo apt install ros-jazzy-rtabmap-*
sudo apt install ros-jazzy-imu-tools
```
6.
```
sudo apt install ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-rviz-plugins
sudo apt install ros-jazzy-joint-state-publisher-gui
```

RUN
Main file 
```
ros2 launch real realLife.launch.py 
```
Costmap + NAV
```
ros2 launch real nav2_bringup.launch.py 
```
Moving Nodes (currently for simulations)
```
ros2 launch real movement.launch.py 
```
RVIZ
```
ros2 launch real rviz.launch.py 
```
FOR AUTONOMOUS EXPLORATION 
```
ros2 launch real explorer.launch.py 
```




sourses:

https://github.com/fadlio/kinect_ros2?tab=readme-ov-file
https://github.com/dawan0111/Simple-2D-LiDAR-Odometry (modified)
https://github.com/107-systems/l3xz_sweep_scanner (modified)

