#!/bin/bash
# Installation script for seano_communication sudoers configuration
# This allows the communication node to switch network routes without password

echo "Installing sudoers configuration for seano_communication..."

SUDOERS_FILE="/etc/sudoers.d/seano-network-switch"
SOURCE_FILE="$(dirname "$0")/seano-network-switch"

if [ ! -f "$SOURCE_FILE" ]; then
    echo "Error: seano-network-switch file not found!"
    exit 1
fi

# Check sudoers file syntax
if ! visudo -cf "$SOURCE_FILE"; then
    echo "Error: Sudoers file has syntax errors!"
    exit 1
fi

# Copy and set permissions
sudo cp "$SOURCE_FILE" "$SUDOERS_FILE"
sudo chmod 440 "$SUDOERS_FILE"
sudo chown root:root "$SUDOERS_FILE"

echo "✓ Sudoers configuration installed successfully!"
echo "✓ User 'seano' can now run 'ip route' commands without password"
echo ""
echo "You can now run the communication node without password prompts:"
echo "  ros2 run seano_communication communication_node"
