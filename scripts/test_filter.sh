#!/bin/bash
#
# Test script for dynamic_cloud_filter
# This script helps verify the installation and basic functionality
#

set -e  # Exit on error

echo "================================================"
echo " Dynamic Cloud Filter - Installation Test"
echo "================================================"
echo ""

# Check ROS environment
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS environment not set!"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi
echo "✓ ROS Distribution: $ROS_DISTRO"

# Check if package is built
if rospack find dynamic_cloud_filter > /dev/null 2>&1; then
    echo "✓ Package found: dynamic_cloud_filter"
    PACKAGE_PATH=$(rospack find dynamic_cloud_filter)
    echo "  Location: $PACKAGE_PATH"
else
    echo "✗ Package not found!"
    echo "  Please build the package with: catkin_make"
    exit 1
fi

# Check dependencies
echo ""
echo "Checking dependencies..."
DEPS=("pcl_ros" "pcl_conversions" "jsk_recognition_msgs")
for dep in "${DEPS[@]}"; do
    if rospack find $dep > /dev/null 2>&1; then
        echo "✓ $dep"
    else
        echo "✗ $dep (missing)"
        echo "  Install with: sudo apt-get install ros-$ROS_DISTRO-${dep//_/-}"
    fi
done

# Check executable
echo ""
echo "Checking executables..."
if [ -f "$PACKAGE_PATH/../../devel/lib/dynamic_cloud_filter/dynamic_cloud_filter_node" ]; then
    echo "✓ Main node executable found"
else
    echo "✗ Executable not found - rebuild required"
fi

# Check configuration files
echo ""
echo "Checking configuration..."
if [ -f "$PACKAGE_PATH/config/default_params.yaml" ]; then
    echo "✓ Configuration file found"
else
    echo "✗ Configuration file missing"
fi

# Check launch files
if [ -f "$PACKAGE_PATH/launch/dynamic_filter.launch" ]; then
    echo "✓ Launch file found"
else
    echo "✗ Launch file missing"
fi

echo ""
echo "================================================"
echo " Installation Check Complete!"
echo "================================================"
echo ""
echo "Next steps:"
echo "1. Prepare your data:"
echo "   - Point cloud topic (sensor_msgs/PointCloud2)"
echo "   - Detection boxes topic (jsk_recognition_msgs/BoundingBoxArray)"
echo ""
echo "2. Run the filter:"
echo "   roslaunch dynamic_cloud_filter dynamic_filter.launch \\"
echo "     input_cloud_topic:=/your/cloud \\"
echo "     input_bbox_topic:=/your/boxes"
echo ""
echo "3. Check output:"
echo "   rostopic echo /filtered/static_cloud"
echo ""
