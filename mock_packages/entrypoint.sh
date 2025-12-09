#!/bin/bash
# Entrypoint script for zj_humanoid_mock Docker container

set -e

# Source ROS setup
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Function to cleanup on exit
cleanup() {
    echo "Shutting down..."
    if [ ! -z "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    exit 0
}

# Trap signals
trap cleanup SIGTERM SIGINT

# Default: start roscore in background and then mock server
if [ "$1" = "mock" ] || [ -z "$1" ]; then
    echo "Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    
    # Wait for roscore to be ready
    echo "Waiting for roscore to be ready..."
    sleep 3
    
    # Check if roscore is running
    if ! kill -0 $ROSCORE_PID 2>/dev/null; then
        echo "Error: roscore failed to start"
        exit 1
    fi
    
    echo "Starting mock server..."
    rosrun zj_humanoid_mock mock_server.py "${@:2}" &
    MOCK_PID=$!
    
    # Wait for mock server
    wait $MOCK_PID
    
elif [ "$1" = "bash" ] || [ "$1" = "sh" ]; then
    # Interactive shell
    exec "$@"
    
else
    # Execute custom command
    exec "$@"
fi

