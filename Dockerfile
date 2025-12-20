FROM osrf/ros:jazzy-desktop-full
RUN apt-get update && apt-get install -y \
    nano \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-twist-mux \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-turtlebot3* \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /home/$USERNAME/venv && \
    /home/$USERNAME/venv/bin/pip install numpy scipy

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Create a non-root user to use if preferred
RUN mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Add sudo support
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc && \
    echo "alias inst_r='source install/setup.bash'" >> /home/$USERNAME/.bashrc && \
    echo "source /home/venv/bin/activate" >> /home/$USERNAME/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]