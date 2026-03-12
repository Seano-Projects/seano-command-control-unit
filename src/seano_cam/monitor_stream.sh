#!/bin/bash

# RTMP Stream Health Monitor & Auto-Restart
# Will check stream health and restart if needed

LOG_FILE="/tmp/rtmp_streamer.log"
CHECK_INTERVAL=30  # Check every 30 seconds
MAX_NO_FRAMES=60   # Restart if no new frames in 60 seconds

echo "========================================"
echo "  RTMP Stream Health Monitor"
echo "  Checking every ${CHECK_INTERVAL}s"
echo "========================================"
echo ""

get_last_frame_count() {
    if [ -f "$LOG_FILE" ]; then
        grep "Streamed.*frames" "$LOG_FILE" | tail -n 1 | grep -oP '\d+(?= frames)'
    else
        echo "0"
    fi
}

restart_stream() {
    echo "[$(date)] Restarting stream..."
    
    # Kill existing streamer
    pkill -9 -f rtmp_streamer
    pkill -9 ffmpeg
    sleep 2
    
    # Start new streamer
    cd ~/Seano_ws
    source install/setup.bash
    nohup ros2 run seano_cam rtmp_streamer --ros-args \
        -p vehicle.id:=SEANO001 \
        -p rtmp.url:=rtmp://72.61.141.126:1935/live/usv-seano \
        -p rtmp.preset:=ultrafast \
        -p rtmp.bitrate:=2000k \
        -p rtmp.width:=640 \
        -p rtmp.height:=480 \
        -p rtmp.fps:=30 > "$LOG_FILE" 2>&1 &
    
    sleep 3
    echo "[$(date)] Stream restarted"
}

LAST_FRAME_COUNT=$(get_last_frame_count)
LAST_CHECK_TIME=$(date +%s)

while true; do
    sleep $CHECK_INTERVAL
    
    CURRENT_TIME=$(date +%s)
    CURRENT_FRAME_COUNT=$(get_last_frame_count)
    
    echo "[$(date)] Frames: $CURRENT_FRAME_COUNT (prev: $LAST_FRAME_COUNT)"
    
    # Check if ffmpeg is running
    if ! pgrep -f "ffmpeg.*rtmp" > /dev/null; then
        echo "[$(date)] ✗ FFmpeg not running!"
        restart_stream
        LAST_FRAME_COUNT=$(get_last_frame_count)
        continue
    fi
    
    # Check if rtmp_streamer is running
    if ! pgrep -f "rtmp_streamer" > /dev/null; then
        echo "[$(date)] ✗ RTMP Streamer not running!"
        restart_stream
        LAST_FRAME_COUNT=$(get_last_frame_count)
        continue
    fi
    
    # Check if frames are being sent
    if [ "$CURRENT_FRAME_COUNT" -eq "$LAST_FRAME_COUNT" ]; then
        TIME_DIFF=$((CURRENT_TIME - LAST_CHECK_TIME))
        if [ $TIME_DIFF -gt $MAX_NO_FRAMES ]; then
            echo "[$(date)] ✗ No new frames in ${TIME_DIFF}s!"
            restart_stream
            LAST_FRAME_COUNT=$(get_last_frame_count)
            LAST_CHECK_TIME=$(date +%s)
            continue
        fi
    else
        # Frames are being sent, update counters
        LAST_FRAME_COUNT=$CURRENT_FRAME_COUNT
        LAST_CHECK_TIME=$CURRENT_TIME
        echo "[$(date)] ✓ Stream healthy"
    fi
done
