#!/bin/bash

set -e

echo ""
echo "📦 Copying ffw.rules to /etc/udev/rules.d/"
echo ""

# Set the path (assuming ffw.rules is in the current directory)
RULE_SOURCE="./ffw.rules"
RULE_TARGET="/etc/udev/rules.d/99-ffw.rules"

# Check if the ffw.rules file exists
if [ ! -f "$RULE_SOURCE" ]; then
    echo "❌ Error: $RULE_SOURCE not found!"
    exit 1
fi

# Copy the file with sudo
sudo cp "$RULE_SOURCE" "$RULE_TARGET"

echo ""
echo "🔄 Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "✅ Udev rule installed successfully!"
echo "🔌 Please replug your USB devices to apply the new settings."
