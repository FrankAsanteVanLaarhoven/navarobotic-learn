#!/bin/bash

# Auto-Setup Script
# This script is automatically run by the services manager
# No manual execution needed - it's integrated into the dev process

echo "🔧 Auto-Setup: Checking prerequisites..."

# Check if ROS is installed (optional - services manager handles this)
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 found"
    
    # Check if rosbridge is installed
    if ros2 pkg list | grep -q rosbridge; then
        echo "✅ ROSBridge installed"
    else
        echo "⚠️  ROSBridge not installed"
        echo "   Install with: sudo apt-get install ros-$(rosversion -d)-rosbridge-suite"
    fi
else
    echo "ℹ️  ROS2 not found (optional for WebROS IDE)"
fi

echo "✅ Auto-setup complete"
echo "   All services will start automatically with 'bun run dev'"
