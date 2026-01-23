# 1. 使用 desktop 基础镜像（含 RViz, rqt），但不含 Gazebo Fortress
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 2. 安装 Gazebo Harmonic 官方源
RUN apt-get update && apt-get install -y wget gnupg lsb-release \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. 安装基础工具、ROS 组件和 Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-actuator-msgs \
    ros-humble-gps-msgs \
    gz-harmonic \
    build-essential \
    cmake \
    git \
    nano \
    tmux \
    iputils-ping \
    net-tools \
    python3-pip \
    python3-opencv \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 4. 安装 Python 算法依赖
RUN pip3 install --no-cache-dir \
    transforms3d \
    scipy \
    pyserial \
    pymavlink

# 5. 编译安装 Micro-XRCE-DDS-Agent
WORKDIR /tmp
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/ && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# 6. 源码编译支持 Harmonic 的 ros_gz_bridge
# 必须源码编译，因为 apt 里的二进制版只支持旧的 Fortress
WORKDIR /opt/ros_gz_ws/src
RUN git clone -b humble https://github.com/gazebosim/ros_gz.git
WORKDIR /opt/ros_gz_ws
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    export GZ_VERSION=harmonic && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y --rosdistro humble \
    --skip-keys="ignition-gazebo8 libignition-gazebo8-dev libignition-math7 libignition-msgs10 libignition-transport13" && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# 7. 设置环境变量
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 8. 创建用户并设置无密码 Sudo
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 自动 source ROS 和 Bridge 环境变量
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /opt/ros_gz_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# 准备工作空间目录
WORKDIR /home/ros2/workspace
RUN chown -R $USERNAME:$USERNAME /home/ros2/workspace

USER $USERNAME
CMD ["/bin/bash"]
