# 1. 使用 desktop 基础镜像（含 RViz, rqt）
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 2. 安装 Gazebo Harmonic 官方源
RUN apt-get update && apt-get install -y wget gnupg lsb-release \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. 安装基础工具、ROS 组件
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
    # python3-opencv \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================
# 4. Python 依赖分层安装 (解决空间不足和冲突警告)
# ==============================================================================

# 4.1: 轻量级工具和 PX4 编译依赖
# 使用 --ignore-installed 避免 pip 试图卸载 apt 安装的包
RUN pip3 install --no-cache-dir --ignore-installed \
    jinja2 \
    kconfiglib \
    jsonschema \
    pyros-genmsg \
    pyserial \
    pymavlink \
    # pyyaml \
    requests \
    tqdm \
    termcolor

# 4.2: 科学计算库 (Numpy/Scipy/Pandas)
# 这些库经常和 apt 的 python3-numpy 冲突，使用 --ignore-installed 强制覆盖
RUN pip3 install --no-cache-dir --ignore-installed \
    "numpy<2.0.0" \
    "opencv-python<=4.8.1.78" \
    pandas \
    scipy \
    matplotlib \
    seaborn \
    transforms3d \
    shapely \
    scikit-learn

# 4.3: 巨型 AI 库 (单独一层，防止构建崩溃)
# PyTorch/Ultralytics 体积巨大，单独放在最后
RUN pip3 install --no-cache-dir --ignore-installed \
    lapx \
    supervision \
    ultralytics

# ==============================================================================

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

# 6. 源码编译核心库 (Overlay Workspace)
WORKDIR /opt/overlay_ws/src

# 6.1 Clone 仓库
RUN git clone -b humble https://github.com/gazebosim/ros_gz.git && \
    git clone -b humble https://github.com/ros/sdformat_urdf.git && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git

WORKDIR /opt/overlay_ws

# 6.2 编译所有包
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    export GZ_VERSION=harmonic && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y --rosdistro humble \
    --skip-keys="ignition-gazebo8 libignition-gazebo8-dev libignition-math7 libignition-msgs10 libignition-transport13" && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# 7. 设置环境
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 8. 用户权限设置
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /opt/overlay_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

WORKDIR /home/ros2/workspace
RUN chown -R $USERNAME:$USERNAME /home/ros2/workspace

USER $USERNAME
CMD ["/bin/bash"]
