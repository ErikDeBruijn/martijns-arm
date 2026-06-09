FROM ros:jazzy-ros-base

ARG USERNAME=ros
ARG UID=1000
ARG GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo git vim less curl wget usbutils \
      python3-pip python3-colcon-common-extensions python3-rosdep \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
      ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-rviz2 \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-micro-ros-msgs \
      ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-controllers ros-jazzy-controller-manager \
      ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
      python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN existing_user=$(getent passwd ${UID} | cut -d: -f1 || true) \
    && if [ -n "$existing_user" ] && [ "$existing_user" != "${USERNAME}" ]; then \
         usermod -l ${USERNAME} -d /home/${USERNAME} -m "$existing_user" \
         && groupmod -n ${USERNAME} "$existing_user" 2>/dev/null || true; \
       elif [ -z "$existing_user" ]; then \
         (getent group ${GID} >/dev/null || groupadd --gid ${GID} ${USERNAME}) \
         && useradd --uid ${UID} --gid ${GID} -m -s /bin/bash ${USERNAME}; \
       fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "[ -f /home/${USERNAME}/ros2_ws/install/setup.bash ] && source /home/${USERNAME}/ros2_ws/install/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

CMD ["bash"]
