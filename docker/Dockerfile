FROM nvcr.io/nvidia/tensorrt:23.03-py3

ENV DEBIAN_FRONTEND=noninteractive

# 基础依赖
RUN apt-get update && apt-get install -y \
    wget curl git cmake build-essential pkg-config \
    python3 python3-pip python3-dev \
    lsb-release gnupg2 software-properties-common \
    libeigen3-dev libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------
# ROS Noetic 源
# -------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros.gpg] http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros1.list

# 安装 ROS (最小依赖)
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-pcl-ros \
    ros-noetic-jsk-recognition-msgs \
    python3-catkin-tools python3-rosdep \
    libopencv-dev \ 
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# rosdep 初始化
RUN rosdep init || true
#RUN rosdep update

# -------------------------------------------------------
# CMake 3.22.3
# -------------------------------------------------------
RUN wget https://github.com/Kitware/CMake/releases/download/v3.22.3/cmake-3.22.3-linux-x86_64.sh \
    && chmod +x cmake-3.22.3-linux-x86_64.sh \
    && ./cmake-3.22.3-linux-x86_64.sh --skip-license --prefix=/usr/local \
    && rm cmake-3.22.3-linux-x86_64.sh
#------------------------------------------------
# MS-mapping geographic安装
#------------------------------------------------

RUN apt-get update && apt-get install -y \
    libgeographic-dev \
    geographiclib-tools && \
    geographiclib-get-geoids egm96-5

# -------------------------------------------------------
# 安装 TensorRT 8.5 (for CUDA 11.8)
# -------------------------------------------------------
#RUN wget https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2004/x86_64/nvidia-machine-learning-repo-ubuntu2004_1.0.0-1_amd64.deb && \
#    dpkg -i nvidia-machine-learning-repo-ubuntu2004_1.0.0-1_amd64.deb && \
#    apt-get update && apt-get install -y \
#        tensorrt \
#        python3-libnvinfer \
#        libnvinfer-dev \
#        libnvinfer-plugin-dev \
#        libnvonnxparsers-dev \
#        libnvparsers-dev \
#    && rm -rf /var/lib/apt/lists/* *.deb
# --------------------------
# 5. TensorRT 头文件路径添加
# --------------------------
ENV TensorRT_DIR=/usr/lib/x86_64-linux-gnu
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
#ENV CPLUS_INCLUDE_PATH=/usr/include/x86_64-linux-gnu:$CPLUS_INCLUDE_PATH
#RUN pip3 install numpy scipy pybind11 matplotlib pyyaml tqdm
# -------------------------------------------------------
# 设置 CUDA Environment
# -------------------------------------------------------
#ENV CUDA_HOME=/usr/local/cuda
#ENV PATH=$CUDA_HOME/bin:$PATH
#ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# -------------------------------------------------------
# 创建工作空间
# -------------------------------------------------------
RUN mkdir -p /home/auto/TRLO_ws/src
WORKDIR /home/auto/TRLO_ws
RUN useradd -m -s /bin/bash auto || true && \
    chown -R auto:auto /home/auto


