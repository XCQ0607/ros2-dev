# -------------------------------------------
# Image A: PC 开发专用 (x86_64) - Modified
# -------------------------------------------
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# 1. 安装基础工具、ROS组件、CycloneDDS
# [修改]: 添加了 'sudo' 包
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    build-essential \
    cmake \
    git \
    wget \
    nano \
    tmux \
    iputils-ping \
    net-tools \
    python3-pip \
    python3-opencv \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 2. 安装 Python 算法依赖
RUN pip3 install --no-cache-dir \
    transforms3d \
    scipy \
    pyserial \
    pymavlink

# 3. 编译安装 Micro-XRCE-DDS-Agent (核心组件)
WORKDIR /tmp
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/ && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# 4. 设置环境变量
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# =========================================================
# 5. [新增] 创建用户并设置无密码 Sudo
# =========================================================
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 创建用户组和用户
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    # 将用户添加到 sudo 组
    && usermod -aG sudo $USERNAME \
    # 配置 sudoers 文件实现无密码 sudo
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 自动 source ROS 环境变量 (这样每次打开终端都会生效)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# 准备工作空间目录并修正权限
WORKDIR /home/ros2/workspace
RUN chown -R $USERNAME:$USERNAME /home/ros2/workspace

# 切换到非 root 用户
USER $USERNAME

# 默认指令
CMD ["/bin/bash"]
