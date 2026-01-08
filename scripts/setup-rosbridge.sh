#!/bin/bash

# ROSBridge Setup Script
# This script helps set up ROSBridge for WebROS IDE integration

echo "🔧 ROSBridge Setup for WebROS IDE"
echo "=================================="
echo ""

# Check if ROS is installed
if ! command -v roscore &> /dev/null; then
    echo "❌ ROS is not installed or not in PATH"
    echo ""
    echo "To install ROS2 (recommended):"
    echo "  Ubuntu/Debian:"
    echo "    sudo apt update"
    echo "    sudo apt install ros-humble-desktop"
    echo "    source /opt/ros/humble/setup.bash"
    echo ""
    echo "  macOS:"
    echo "    brew install ros-humble-desktop"
    echo "    source /opt/homebrew/opt/ros/humble/setup.bash"
    echo ""
    exit 1
fi

echo "✅ ROS found: $(rosversion -d)"
echo ""

# Check if rosbridge is installed
if ! ros2 pkg list | grep -q rosbridge; then
    echo "📦 Installing ROSBridge..."
    echo ""
    
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        sudo apt-get update
        sudo apt-get install -y ros-$(rosversion -d)-rosbridge-suite
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        echo "For macOS, install via:"
        echo "  brew install ros-$(rosversion -d)-rosbridge-suite"
    fi
    
    echo ""
    echo "✅ ROSBridge installed"
else
    echo "✅ ROSBridge is already installed"
fi

echo ""
echo "🚀 Starting ROSBridge WebSocket Server..."
echo ""
echo "This will start ROSBridge on ws://localhost:9090"
echo "Press Ctrl+C to stop"
echo ""

# Source ROS setup
if [ -f /opt/ros/$(rosversion -d)/setup.bash ]; then
    source /opt/ros/$(rosversion -d)/setup.bash
elif [ -f /opt/homebrew/opt/ros/$(rosversion -d)/setup.bash ]; then
    source /opt/homebrew/opt/ros/$(rosversion -d)/setup.bash
fi

# Start ROSBridge
ros2 launch rosbridge_server rosbridge_websocket.launch.py
