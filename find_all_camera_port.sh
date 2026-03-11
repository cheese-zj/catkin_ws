#!/usr/bin/env bash
set -euo pipefail

if ! command -v udevadm >/dev/null 2>&1; then
    echo "Error: udevadm is required but not found."
    exit 1
fi

if [ ! -d /dev/v4l/by-id ]; then
    echo "No /dev/v4l/by-id entries found. Are camera devices connected?"
    exit 1
fi

echo "Detected cameras:"
printf "%-48s %-12s %-s\n" "Camera ID" "Device" "USB port"
printf "%-48s %-12s %-s\n" "---------" "------" "--------"

found=0
for link in /dev/v4l/by-id/*; do
    [ -e "$link" ] || continue
    [ -L "$link" ] || continue

    target="$(readlink -f "$link")"
    dev_name="$(basename "$target")"
    sys_path="$(udevadm info --query=path --name="$target" 2>/dev/null || true)"
    usb_port="$(echo "$sys_path" | sed -n 's|.*/\([^/]*:[0-9.]*\)/video4linux/.*|\1|p')"
    [ -n "$usb_port" ] || usb_port="unknown"

    printf "%-48s %-12s %-s\n" "$(basename "$link")" "$dev_name" "$usb_port"
    found=1
done

if [ "$found" -eq 0 ]; then
    echo "No camera symlinks found under /dev/v4l/by-id."
    exit 1
fi

echo
echo "Use the /dev/v4l/by-id/... path in launch args for stable camera assignment:"
echo "  roslaunch piper_teleop teleop_collect.launch \\"
echo "      wrist_left_device:=/dev/v4l/by-id/<left_cam> \\"
echo "      wrist_right_device:=/dev/v4l/by-id/<right_cam>"
echo
echo "For OPP USB cam stable /dev name, generate a udev symlink once:"
echo "  bash /home/jameszhao2004/catkin_ws/workspaces/scripts/install_opp_camera_udev_rule.sh --device /dev/videoN --symlink wrist_opp_camera"
