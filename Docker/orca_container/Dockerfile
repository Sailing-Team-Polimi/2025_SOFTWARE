FROM osrf/ros:jazzy-desktop-full

# Setup ros user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir /home/${USERNAME}/.config \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config

# Setup sudo for ros user
RUN apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y x11-apps

# install pip and fix SetuptoolsDeprecationWarning for latest python on ros2
RUN apt-get update \
    && apt-get install -y python3-pip \
    && pip install setuptools==58.2.0

# make everything up to date
RUN apt-get update \
    && apt-get -y upgrade

RUN mkdir -p /home/ros/ros2_ws/src && \
    chmod 777 -R /home/ros/ros2_ws

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/${USERNAME}/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> /home/${USERNAME}/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /home/ros/ros2_ws/install/local_setup.bash" >> /home/${USERNAME}/.bashrc

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["/bin/bash"]

# raspberry specific packages installation
RUN apt-get update && apt-get install -y \
    gpiod libgpiod-dev \
    i2c-tools libi2c-dev \
    gpsd gpsd-clients python3-gps \
    wget nano tree minicom

## ROS PACKAGES #####

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-foxglove-bridge 
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros2-control \
                                         ros-${ROS_DISTRO}-ros2-controllers
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rviz-imu-plugin 
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-tf2 \
                                         ros-${ROS_DISTRO}-tf2-ros \
                                         ros-${ROS_DISTRO}-tf2-geometry-msgs \
                                         ros-${ROS_DISTRO}-tf-transformations 
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-robot-localization

