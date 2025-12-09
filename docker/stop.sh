#!/bin/bash

CONTAINER_NAME="ros2_humble_arm"

echo ">>> 停止容器：$CONTAINER_NAME"

# 如果容器运行中，停止
if docker ps --format '{{.Names}}' | grep -w "$CONTAINER_NAME" >/dev/null; then
    docker stop "$CONTAINER_NAME"
    echo ">>> 容器已停止"
else
    echo ">>> 容器未运行"
fi

# 如果容器存在（即使已停止），删除
if docker ps -a --format '{{.Names}}' | grep -w "$CONTAINER_NAME" >/dev/null; then
    docker rm "$CONTAINER_NAME"
    echo ">>> 容器已删除"
else
    echo ">>> 容器不存在，无需删除"
fi

echo ">>> stop_arm.sh 执行完成"

