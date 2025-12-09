#!/bin/bash
#
# Setup script for Dynamic Cloud Filter
# Helps configure model paths and dependencies
#
# Copyright (c) 2024 AutoCity
# Author: Sonny (sonnygonnarich@gmail.com)
#

set -e

echo "===================================================================="
echo " Dynamic Cloud Filter - Setup Script"
echo " Copyright (c) 2024 AutoCity"
echo "===================================================================="
echo ""

# Check CUDA
echo "[1/5] Checking CUDA installation..."
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
    echo "✓ CUDA is available"
else
    echo "✗ CUDA not found! Please install NVIDIA drivers and CUDA toolkit"
    exit 1
fi

# Check TensorRT
echo ""
echo "[2/5] Checking TensorRT..."
TENSORRT_PATHS=(
    "/usr/local/TensorRT-8.5.3.1"
    "/usr/local/tensorrt"
    "/opt/tensorrt"
    "$HOME/TensorRT-8.5.3.1"
)

TENSORRT_FOUND=false
for path in "${TENSORRT_PATHS[@]}"; do
    if [ -d "$path" ]; then
        echo "✓ Found TensorRT at: $path"
        echo "  Please update CMakeLists.txt line 41:"
        echo "  set(TENSORRT_ROOT $path ...)"
        TENSORRT_FOUND=true
        break
    fi
done

if [ "$TENSORRT_FOUND" = false ]; then
    echo "✗ TensorRT not found in common locations"
    echo "  Please install TensorRT and update CMakeLists.txt"
fi

# Check spconv library
echo ""
echo "[3/5] Checking spconv library..."
PACKAGE_PATH=$(rospack find dynamic_cloud_filter 2>/dev/null || echo ".")
SPCONV_LIB="$PACKAGE_PATH/3rdparty/libspconv/lib/x86_64/libspconv.so"

if [ -f "$SPCONV_LIB" ]; then
    echo "✓ spconv library found"
else
    echo "✗ spconv library not found"
    echo "  Expected at: $SPCONV_LIB"
    echo "  Please copy from TRLO project:"
    echo "  cp -r /path/to/TRLO/3rdparty/libspconv ./3rdparty/"
fi

# Setup model directory
echo ""
echo "[4/5] Setting up model directory..."
MODEL_DIR="$HOME/models/centerpoint"

if [ ! -d "$MODEL_DIR" ]; then
    echo "Creating model directory: $MODEL_DIR"
    mkdir -p "$MODEL_DIR"
fi

echo "Model directory: $MODEL_DIR"
echo ""
echo "Required model files:"
echo "  - rpn.plan (RPN network)"
echo "  - head.plan (Detection head)"
echo "  - sparse_shape.txt (Sparse tensor shape)"
echo ""

if [ -f "$MODEL_DIR/rpn.plan" ] && [ -f "$MODEL_DIR/head.plan" ]; then
    echo "✓ Model files found"
else
    echo "✗ Model files missing"
    echo ""
    echo "Please copy model files from TRLO:"
    echo "  cp /path/to/TRLO/model/center_pointpillars/* $MODEL_DIR/"
    echo ""
    echo "Or download pre-trained models and convert to TensorRT format"
fi

# Check ROS dependencies
echo ""
echo "[5/5] Checking ROS dependencies..."
DEPS=("pcl_ros" "jsk_recognition_msgs" "tf")
MISSING_DEPS=()

for dep in "${DEPS[@]}"; do
    if rospack find $dep &> /dev/null; then
        echo "✓ $dep"
    else
        echo "✗ $dep (missing)"
        MISSING_DEPS+=($dep)
    fi
done

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo ""
    echo "Install missing dependencies:"
    echo "  sudo apt-get install ros-\$ROS_DISTRO-${MISSING_DEPS[@]}"
fi

# Summary
echo ""
echo "===================================================================="
echo " Setup Summary"
echo "===================================================================="
echo ""
echo "Next steps:"
echo "1. Update TensorRT path in CMakeLists.txt (line 41)"
echo "2. Copy model files to: $MODEL_DIR"
echo "3. Copy spconv library if missing"
echo "4. Build: catkin_make"
echo "5. Run: roslaunch dynamic_cloud_filter complete_pipeline.launch"
echo ""
echo "For detailed instructions, see README_COMPLETE.md"
echo ""
