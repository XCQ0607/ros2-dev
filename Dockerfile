# 1. 使用 desktop 基础镜像（含 RViz, rqt）
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 2. 安装 Gazebo Harmonic 官方源
RUN apt-get update && apt-get install -y wget gnupg lsb-release \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. 安装基础工具、ROS 组件 (含 robot_localization) 和 Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-actuator-msgs \
    ros-humble-gps-msgs \
    ros-humble-robot-localization \
    ros-humble-plotjuggler-ros \
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

# 4. 安装 Python 依赖 (新增 PX4 消息生成所需的库)
# numpy, jinja2, kconfiglib, pyros-genmsg 是 px4_msgs 编译必须的
RUN pip3 install --no-cache-dir \
    transforms3d \
    scipy \
    pyserial \
    pymavlink \
    numpy \
    jinja2 \
    kconfiglib \
    jsonschema \
    pyros-genmsg

# 5. 编译安装 Micro-XRCE-DDS-Agent
# 这是 PX4 与 ROS 2 通讯的中间件代理
WORKDIR /tmp
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/ && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# ==============================================================================
# 6. 源码编译核心库：ros_gz, sdformat_urdf, px4_msgs, px4_ros_com
# 将目录更名为 overlay_ws，因为现在不仅仅是 gz 的东西了
# ==============================================================================
WORKDIR /opt/overlay_ws/src

# 6.1 ros_gz (Humble分支) - 连接 ROS 2 和 Gazebo Harmonic
RUN git clone -b humble https://github.com/gazebosim/ros_gz.git

# 6.2 sdformat_urdf (Humble分支) - 让 ROS 2 能解析新的 SDF
RUN git clone -b humble https://github.com/ros/sdformat_urdf.git

# 6.3 px4_msgs (Main分支) - PX4 的 ROS 2 消息定义
RUN git clone https://github.com/PX4/px4_msgs.git

# 6.4 px4_ros_com (Main分支) - PX4 的 ROS 2 转换逻辑
RUN git clone https://github.com/PX4/px4_ros_com.git

WORKDIR /opt/overlay_ws

# 6.5 安装依赖并编译所有包
# export GZ_VERSION=harmonic 对 ros_gz 和 sdformat_urdf 生效
# px4 包会忽略这个变量，不受影响
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    export GZ_VERSION=harmonic && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y --rosdistro humble \
    --skip-keys="ignition-gazebo8 libignition-gazebo8-dev libignition-math7 libignition-msgs10 libignition-transport13" && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*
# ==============================================================================

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

# 自动 source ROS 和 Overlay Workspace
# 这样用户一进入容器，ros2 topic list 就能看到 px4 的消息类型，也能运行 bridge
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /opt/overlay_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# 准备用户工作空间目录
WORKDIR /home/ros2/workspace
RUN chown -R $USERNAME:$USERNAME /home/ros2/workspace

USER $USERNAME
CMD ["/bin/bash"]
