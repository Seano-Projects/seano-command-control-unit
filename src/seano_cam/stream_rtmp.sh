#!/bin/bash

# RTMP Streaming Helper Script untuk SEANO Camera
# Usage: ./stream_rtmp.sh [vehicle_id] [rtmp_url]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}   SEANO Camera RTMP Streaming Setup${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# Default values
VEHICLE_ID="${1:-SEANO001}"
RTMP_URL="${2:-rtmp://72.61.141.126:1935/live/usv-seano}"

echo -e "${YELLOW}Configuration:${NC}"
echo -e "Vehicle ID: ${GREEN}${VEHICLE_ID}${NC}"
echo -e "RTMP URL: ${GREEN}${RTMP_URL}${NC}"
echo ""

# Check if ffmpeg is installed
echo -e "${YELLOW}[1/4] Checking FFmpeg installation...${NC}"
if ! command -v ffmpeg &> /dev/null; then
    echo -e "${RED}✗ FFmpeg not found!${NC}"
    echo -e "${YELLOW}Installing FFmpeg...${NC}"
    sudo apt-get update && sudo apt-get install -y ffmpeg
    echo -e "${GREEN}✓ FFmpeg installed${NC}"
else
    FFMPEG_VERSION=$(ffmpeg -version | head -n1)
    echo -e "${GREEN}✓ FFmpeg found: ${FFMPEG_VERSION}${NC}"
fi
echo ""

# Check if ROS2 workspace is sourced
echo -e "${YELLOW}[2/4] Checking ROS2 environment...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS2 not sourced!${NC}"
    echo -e "${YELLOW}Sourcing workspace...${NC}"
    source ~/Seano_ws/install/setup.bash
    echo -e "${GREEN}✓ ROS2 environment sourced${NC}"
else
    echo -e "${GREEN}✓ ROS2 ${ROS_DISTRO} detected${NC}"
fi
echo ""

# Test RTMP server connectivity (optional)
echo -e "${YELLOW}[3/4] Testing RTMP server connectivity...${NC}"
RTMP_HOST=$(echo $RTMP_URL | sed -E 's|rtmp://([^:/]+).*|\1|')
RTMP_PORT=$(echo $RTMP_URL | sed -E 's|rtmp://[^:]+:([0-9]+).*|\1|')

if [ "$RTMP_PORT" = "$RTMP_URL" ]; then
    RTMP_PORT=1935  # Default RTMP port
fi

echo -e "Testing connection to ${RTMP_HOST}:${RTMP_PORT}..."
if timeout 5 bash -c "cat < /dev/null > /dev/tcp/${RTMP_HOST}/${RTMP_PORT}" 2>/dev/null; then
    echo -e "${GREEN}✓ RTMP server is reachable${NC}"
else
    echo -e "${YELLOW}⚠ Warning: Cannot connect to RTMP server${NC}"
    echo -e "${YELLOW}  This might be normal if server requires authentication${NC}"
fi
echo ""

# Check if camera node is running
echo -e "${YELLOW}[4/4] Checking camera node status...${NC}"
CAMERA_TOPIC="/seano/${VEHICLE_ID}/camera/image"
if ros2 topic list 2>/dev/null | grep -q "$CAMERA_TOPIC"; then
    echo -e "${GREEN}✓ Camera node is publishing to ${CAMERA_TOPIC}${NC}"
    
    # Check FPS
    echo -e "${YELLOW}Checking camera FPS...${NC}"
    timeout 3 ros2 topic hz $CAMERA_TOPIC --window 5 2>&1 | grep "average rate" | tail -n1 || echo "  (checking...)"
else
    echo -e "${RED}✗ Camera node not running or not publishing!${NC}"
    echo -e "${YELLOW}Start camera node first with:${NC}"
    echo -e "  ros2 run seano_cam camera_node --ros-args -p vehicle.id:=${VEHICLE_ID}"
    exit 1
fi
echo ""

# Ready to stream
echo -e "${BLUE}=============================================${NC}"
echo -e "${GREEN}✓ All checks passed! Ready to stream${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""
echo -e "${YELLOW}Starting RTMP streamer...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop streaming${NC}"
echo ""

# Start RTMP streamer
ros2 run seano_cam rtmp_streamer --ros-args \
    -p vehicle.id:=${VEHICLE_ID} \
    -p rtmp.url:=${RTMP_URL} \
    -p rtmp.fps:=30 \
    -p rtmp.width:=1280 \
    -p rtmp.height:=720 \
    -p rtmp.bitrate:=2500k \
    -p rtmp.preset:=ultrafast
