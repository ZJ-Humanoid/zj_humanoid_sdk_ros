#!/bin/bash
# Quick test script for Docker mock package

set -e

echo "=========================================="
echo "Testing zj_humanoid_mock Docker setup"
echo "=========================================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed. Please install Docker first."
    exit 1
fi

echo "✅ Docker is installed"

# Check if docker compose is installed (optional)
# Docker Compose V2 is a plugin, check with 'docker compose version'
if docker compose version &> /dev/null 2>&1; then
    echo "✅ docker compose is installed"
    USE_COMPOSE=true
else
    echo "⚠️  docker compose not found, will use docker commands"
    USE_COMPOSE=false
fi

# Build the image
echo ""
echo "Building Docker image..."
if [ "$USE_COMPOSE" = true ]; then
    # docker compose will use the context from docker-compose.yml
    docker compose build
else
    # Build from project root directory (parent of mock_packages)
    cd "$(dirname "$0")/.." || exit 1
    docker build -f mock_packages/Dockerfile -t zj_humanoid_mock:latest .
    cd - || exit 1
fi

if [ $? -eq 0 ]; then
    echo "✅ Docker image built successfully"
else
    echo "❌ Failed to build Docker image"
    exit 1
fi

# Test if we can start the container (briefly)
echo ""
echo "Testing container startup (5 seconds)..."
if [ "$USE_COMPOSE" = true ]; then
    timeout 5 docker compose up || true
    docker compose down
else
    timeout 5 docker run --rm --network host \
        -e ROS_MASTER_URI=http://localhost:11311 \
        zj_humanoid_mock:latest || true
fi

echo ""
echo "=========================================="
echo "✅ Docker setup test completed!"
echo ""
echo "To start the mock server, run:"
if [ "$USE_COMPOSE" = true ]; then
    echo "  docker compose up"
else
    echo "  docker run -it --rm --network host \\"
    echo "    -e ROS_MASTER_URI=http://localhost:11311 \\"
    echo "    zj_humanoid_mock:latest"
fi
echo "=========================================="

