#!/usr/bin/env bash

CONTAINER_NAME=trlo_container
IMAGE_NAME=my-noetic-cuda-image:latest
SHARED_DIR=/home/auto//TRLO_ws

# -------------------------------------------------------
# 1. 允许 X11 显示
# -------------------------------------------------------
echo ">>> 配置 X11 显示权限..."
xhost +local:root > /dev/null 2>&1

# -------------------------------------------------------
# 2. 创建共享目录（如果不存在）
# -------------------------------------------------------
if [ ! -d "$SHARED_DIR" ]; then
    echo ">>> 创建共享目录：$SHARED_DIR"
    mkdir -p "$SHARED_DIR"
fi

# -------------------------------------------------------
# 3. 如果容器已存在，启动并进入
# -------------------------------------------------------
if sudo docker ps -a --format '{{.Names}}' | grep -qw "$CONTAINER_NAME"; then
    echo ">>> 启动已有容器：$CONTAINER_NAME"
    sudo docker start "$CONTAINER_NAME"

    echo ">>> 进入容器..."
    exec sudo docker exec -it "$CONTAINER_NAME" bash
fi

# -------------------------------------------------------
# 4. 创建新容器
# -------------------------------------------------------
echo ">>> 创建并启动新容器：$CONTAINER_NAME"

sudo docker run -it --name "$CONTAINER_NAME" \
    --gpus all \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$SHARED_DIR":/home/auto/TRLO_ws \
    -v ~/.bashrc:/home/auto/.bashrc_host:ro \
    -w /home/auto/TRLO_ws \
    "$IMAGE_NAME" bash -c "echo 'source \$ROS_SETUP' >> ~/.bashrc && bash"

