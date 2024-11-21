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

# FROM osrf/ros:noetic-desktop-full

# RUN apt-get update && apt-get install -y \
#     python3-pip \
#     python3-catkin-tools \
#     ros-noetic-joy \
#     ros-noetic-serial \
#     git \
#     libmodbus-dev \
#     udev \
#     usbutils \
#     && rm -rf /var/lib/apt/lists/*

# ARG USER=rosuser
# ARG UID=1000
# ARG GID=1000
# RUN groupadd -g $GID $USER \
#     && useradd -m -u $UID -g $GID -s /bin/bash $USER

# WORKDIR /ros_ws
# RUN mkdir -p /ros_ws/src
# COPY . /ros_ws/src/pan_tilt_ros/
# RUN chown -R $USER:$USER /ros_ws

# USER $USER

# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /ros_ws; catkin_make'
# RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

# COPY --chown=$USER:$USER ./docker-entrypoint.sh /home/$USER/
# RUN chmod +x /home/$USER/docker-entrypoint.sh

# ENTRYPOINT ["/home/rosuser/docker-entrypoint.sh"]
# CMD ["bash"]