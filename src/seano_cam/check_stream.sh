#!/bin/bash

# RTMP Stream Troubleshooting Script
# Untuk cek kenapa web player blank

echo "========================================"
echo "  RTMP Stream Diagnostics"
echo "========================================"
echo ""

# 1. Check camera node
echo "[1] Checking Camera Node..."
if ros2 topic list 2>/dev/null | grep -q "/seano/SEANO001/camera/image"; then
    echo "✓ Camera node is running"
    FPS=$(timeout 2 ros2 topic hz /seano/SEANO001/camera/image --window 5 2>&1 | grep "average rate" | tail -n1 | awk '{print $3}' || echo "N/A")
    echo "  FPS: $FPS"
else
    echo "✗ Camera node NOT running!"
    echo "  Fix: ros2 run seano_cam camera_node --ros-args -p vehicle.id:=SEANO001"
fi
echo ""

# 2. Check RTMP streamer
echo "[2] Checking RTMP Streamer..."
if ps aux | grep -q "[r]tmp_streamer"; then
    echo "✓ RTMP streamer is running"
    PID=$(ps aux | grep "[r]tmp_streamer" | awk '{print $2}')
    echo "  PID: $PID"
    
    # Check last log
    if [ -f /tmp/rtmp_streamer.log ]; then
        LAST_LOG=$(tail -n 1 /tmp/rtmp_streamer.log)
        echo "  Last log: $LAST_LOG"
    fi
else
    echo "✗ RTMP streamer NOT running!"
    echo "  Fix: cd ~/Seano_ws/src/seano_cam && ./stream_rtmp.sh"
fi
echo ""

# 3. Check RTMP server connectivity
echo "[3] Testing RTMP Server..."
RTMP_HOST="72.61.141.126"
RTMP_PORT="1935"
if timeout 3 bash -c "cat < /dev/null > /dev/tcp/${RTMP_HOST}/${RTMP_PORT}" 2>/dev/null; then
    echo "✓ RTMP server is reachable (${RTMP_HOST}:${RTMP_PORT})"
else
    echo "✗ Cannot reach RTMP server!"
    echo "  - Check internet connection"
    echo "  - Verify server is running"
    echo "  - Check firewall rules"
fi
echo ""

# 4. Check FFmpeg process
echo "[4] Checking FFmpeg..."
if ps aux | grep -q "[f]fmpeg.*rtmp"; then
    echo "✓ FFmpeg is encoding and streaming"
    FFMPEG_PID=$(ps aux | grep "[f]fmpeg.*rtmp" | awk '{print $2}')
    echo "  PID: $FFMPEG_PID"
    
    # CPU usage
    CPU=$(ps aux | grep "[f]fmpeg.*rtmp" | awk '{print $3}')
    echo "  CPU: ${CPU}%"
else
    echo "✗ FFmpeg NOT running!"
    echo "  This may indicate streaming failed to start"
fi
echo ""

# 5. Web player tips
echo "[5] Web Player Troubleshooting:"
echo "---"
echo "Jika web masih blank, coba:"
echo ""
echo "A. Hard Refresh Browser:"
echo "   - Windows/Linux: Ctrl + F5"
echo "   - Mac: Cmd + Shift + R"
echo ""
echo "B. Clear Browser Cache:"
echo "   - Settings > Privacy > Clear browsing data"
echo ""
echo "C. Try Different Browser:"
echo "   - Chrome, Firefox, Safari, Edge"
echo ""
echo "D. Check Web Console (F12):"
echo "   - Look for error messages"
echo "   - Check Network tab for failed requests"
echo ""
echo "E. Check Stream Latency:"
echo "   - RTMP streams have 5-15 second delay (normal)"
echo "   - Wait 10-20 seconds after refresh"
echo ""
echo "F. Test with VLC:"
echo "   vlc rtmp://72.61.141.126:1935/live/usv-seano"
echo ""
echo "G. Restart Everything:"
echo "   # Stop streamer"
echo "   pkill -f rtmp_streamer"
echo "   sleep 2"
echo "   # Start again"
echo "   cd ~/Seano_ws/src/seano_cam && ./stream_rtmp.sh"
echo ""
echo "========================================"
echo "Stream URL: rtmp://72.61.141.126:1935/live/usv-seano"
echo "========================================"
