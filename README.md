# pan_tilt_ros
IQR gen4th mount，[manual](http://doc.iquotient-robotics.com/pan_tilt_unit_user_manual/)。

## Requirement
- Ubuntu 18.04
- ROS melodic

## Install and Compile
Update software
```shell
sudo apt-get update
```
Install dependency
```shell
sudo apt-get install ros-melodic-joy
```
Creat workspace
```shell
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
Clone
```shell
git clone https://github.com/I-Quotient-Robotics/pan_tilt_ros.git
cd ..
catkin_make
```
Setup environment
```shell
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Add rules file
```shell
roscd pan_tilt_bringup/config/
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/
```
**Notice, remember to replug the usb after adding rule**

## Suage
First, connect the power charge and usb of the mount
### 1.Normal Mode
Launch the node
```shell
roslaunch pan_tilt_bringup panTilt_view.launch
```
Send position
```shell
rostopic pub /pan_tilt_cmd_deg pan_tilt_msgs/PanTiltCmdDeg "yaw: 30.0
pitch: 30.0
speed: 10" 
```

### 2.Joy Control
Connect to the joy and launch the node
```shell
roslaunch pan_tilt_bringup panTilt_view.launch use_joy:=true
```
Use the *X* *Y* *A* *B* button to control rotations




**Notice!!! If the mount node didn't launch sucessfully, maybe it was caused by the usb port number in src/pan_tilt_ros/pan_tilt_driver/src/PanTiltDriverNode.cpp file line 88. Double check the port number usage and modify it accordingly**
