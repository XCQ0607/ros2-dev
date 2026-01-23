# 使用你之前的镜像作为基础
FROM ghcr.io/xcq0607/ros2-dev

USER root

# 1. 添加 Gazebo 官方源 (为了安装 Harmonic 依赖库)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 2. 安装 Harmonic 核心开发库并卸载旧版 Bridge 
# 注意：我们必须卸载二进制的 ros-humble-ros-gz，否则会发生头文件冲突
RUN apt-get update && apt-get install -y \
    libgz-transport13-dev \
    libgz-sim8-dev \
    libgz-msgs10-dev \
    libgz-math7-dev \
    lsb-release \
    && apt-get remove -y ros-humble-ros-gz* \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# 3. 创建独立的编译工作空间 (不占用用户的 /home/ros2/workspace)
WORKDIR /opt/ros_gz_ws/src
RUN git clone -b humble https://github.com/gazebosim/ros_gz.git

# 4. 编译 ros_gz (强制指定版本为 Harmonic)
WORKDIR /opt/ros_gz_ws
RUN . /opt/ros/humble/setup.sh && \
    export GZ_VERSION=harmonic && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y --rosdistro humble --skip-keys="ignition-gazebo8 libignition-gazebo8-dev" && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. 环境自动化配置
# 将编译好的 bridge 自动加入 dev 用户的环境变量
RUN echo "source /opt/ros_gz_ws/install/setup.bash" >> /home/dev/.bashrc

# 修正权限，确保 dev 用户可以访问此空间
RUN chown -R dev:dev /opt/ros_gz_ws

# 切回默认开发目录和用户
WORKDIR /home/ros2/workspace
USER dev

# 默认指令
CMD ["/bin/bash"]
