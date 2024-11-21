# Pan-Tilt ROS Docker Setup Guide
IQR gen4th mount，[manual](http://doc.iquotient-robotics.com/pan_tilt_unit_user_manual/)。

## Overview
This guide explains how to set up and run the IQR Pan-Tilt platform using Docker with ROS Noetic. Docker containerization ensures consistent environment across different host systems.

## Prerequisites
- Ubuntu host machine (tested on 20.04, 22.04)
- Docker installed
- IQR Pan-Tilt hardware
- USB connection to Pan-Tilt

## Setup Steps

### 1. USB Device Rules
Create udev rules on the host machine to allow access to the Pan-Tilt device:

```bash
# Create udev rule for Arduino-based Pan-Tilt
sudo bash -c 'echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"2341\", ATTRS{idProduct}==\"8037\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"pan_tilt\"" > /etc/udev/rules.d/56-pan-tilt.rules'

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify device connection
ls -l /dev/ttyACM*  # Should show the device
ls -l /dev/pan_tilt  # Should show symlink
```

### 2. Docker Setup

Create a new directory and save these files:

**Dockerfile**:
```dockerfile
FROM osrf/ros:noetic-desktop-full

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-joy \
    ros-noetic-serial \
    git \
    libmodbus-dev \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Create user with proper permissions
ARG USER=rosuser
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID -s /bin/bash $USER \
    && usermod -a -G dialout $USER

# Setup workspace
WORKDIR /ros_ws
RUN mkdir -p /ros_ws/src
COPY . /ros_ws/src/pan_tilt_ros/
RUN chown -R $USER:$USER /ros_ws

USER $USER

# Build workspace
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /ros_ws; catkin_make'
RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

# Setup entrypoint
COPY --chown=$USER:$USER ./docker-entrypoint.sh /home/$USER/
RUN chmod +x /home/$USER/docker-entrypoint.sh

ENTRYPOINT ["/home/rosuser/docker-entrypoint.sh"]
CMD ["bash"]
```

**docker-entrypoint.sh**:
```bash
#!/bin/bash
source "/opt/ros/noetic/setup.bash"
source "/ros_ws/devel/setup.bash"
exec "$@"
```

### 3. Build and Run

```bash
# Build Docker image
docker build -t pan_tilt_ros .

# Run container
docker run -it \
    --privileged \
    --device=/dev/ttyACM0:/dev/pan_tilt \
    --network=host \
    --env="DISPLAY" \
    --volume="$HOME/.Xauthority:/home/rosuser/.Xauthority:rw" \
    pan_tilt_ros
```

### 4. Testing the Setup

## Hardware Limits
The IQR Pan-Tilt platform has the following operation ranges:
- Yaw (horizontal rotation): -60° to +60°
- Pitch (vertical tilt): -60° to +60°
- Speed: 1 to 30 (unitless)

Exceeding these limits will result in a warning message and the command will be ignored.

## Testing Movement
Inside the container:
```bash
# First, verify device and build
ls -l /dev/pan_tilt
sed -i 's/xacro.py/xacro/g' /ros_ws/src/pan_tilt_ros/pan_tilt_bringup/launch/panTilt_bringup.launch
catkin_make
source devel/setup.bash

# Launch ROS nodes
roslaunch pan_tilt_bringup panTilt_view.launch

# In a new terminal (new container session)
docker exec -it <container_id> bash
source /ros_ws/devel/setup.bash

# Test movement (within valid ranges)
rostopic pub /pan_tilt_cmd_deg pan_tilt_msgs/PanTiltCmdDeg "yaw: 30.0
pitch: 30.0
speed: 15"

# Monitor current position
rostopic echo /pan_tilt_status
```

## Troubleshooting Movement Issues
1. Invalid Parameters:
   - Check warning messages for valid ranges
   - Ensure commands are within limits
   - Start with small movements first

2. No Visual Movement:
   - Check current position in /pan_tilt_status
   - Verify motor power supply
   - Check for physical obstructions
   - Monitor joint_states topic

3. Noise/Vibration without Movement:
   - Could indicate:
     - Motor stall condition
     - Mechanical binding
     - Power supply issues
     - Control loop instability

## Common Commands

```bash
# Check device status
rostopic echo /pan_tilt_status

# Monitor joint positions
rostopic echo /joint_states

# Check node status
rosnode info /pan_tilt_driver
```

## Notes
- The container preserves workspace between runs
- GUI applications (RViz) require proper X11 forwarding
- USB device must be connected before starting container