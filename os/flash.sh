#!/usr/bin/env bash
set -e

echo "Looking for built SD image..."
IMG=$(ls -t result/sd-image/*.img result/sd-image/*.img.zst 2>/dev/null | head -n 1)

if [ -z "$IMG" ]; then
    echo "Error: Could not find an .img or .img.zst in result/sd-image/!"
    echo "Make sure you run ./build.sh first."
    exit 1
fi

echo "Found image: $IMG"
echo ""

# Robustly find the first removable drive that isn't empty (SIZE!="0B")
BEST_GUESS=$(lsblk -d -p -P -o NAME,SIZE,RM | grep 'RM="1"' | grep -v 'SIZE="0B"' | grep -o 'NAME="[^"]*"' | cut -d'"' -f2 | head -n 1)
TARGET_DEV=""

if [ -n "$BEST_GUESS" ]; then
    GUESS_INFO=$(lsblk -d -p -P -o NAME,SIZE,MODEL | grep "NAME=\"$BEST_GUESS\"")
    G_SIZE=$(echo "$GUESS_INFO" | grep -o 'SIZE="[^"]*"' | cut -d'"' -f2)
    G_MODEL=$(echo "$GUESS_INFO" | grep -o 'MODEL="[^"]*"' | cut -d'"' -f2)
    
    echo "Best guess for target drive:"
    echo "-> $BEST_GUESS ($G_SIZE) $G_MODEL"
    echo ""
    read -p "Do you want to flash to this drive? [y/N]: " USE_GUESS
    if [[ "$USE_GUESS" =~ ^[Yy](es)?$ ]]; then
        TARGET_DEV="$BEST_GUESS"
    fi
fi

if [ -z "$TARGET_DEV" ]; then
    echo ""
    echo "Looking for removable drives (USB / SD Cards)..."
    echo "------------------------------------------------"
    # Use -P for parseable key=value pairs, handle empty/spaced MODELs safely
    lsblk -d -p -P -o NAME,SIZE,MODEL,RM | grep 'RM="1"' | grep -v 'SIZE="0B"' | while read -r line; do
        D_NAME=$(echo "$line" | grep -o 'NAME="[^"]*"' | cut -d'"' -f2)
        D_SIZE=$(echo "$line" | grep -o 'SIZE="[^"]*"' | cut -d'"' -f2)
        D_MODEL=$(echo "$line" | grep -o 'MODEL="[^"]*"' | cut -d'"' -f2)
        echo "-> $D_NAME ($D_SIZE) $D_MODEL"
    done
    echo "------------------------------------------------"
    echo ""
    read -p "Enter the exact device path you want to flash (e.g. /dev/sdb): " TARGET_DEV
fi

# Verify the user input is actually a block device
if [ ! -b "$TARGET_DEV" ]; then
    echo "Error: '$TARGET_DEV' is not a valid block device!"
    exit 1
fi

echo ""
echo "🚨 WARNING 🚨"
echo "This will COMPLETELY WIPE everything on $TARGET_DEV."
read -p "Are you absolutely sure? (Type 'yes' to continue): " CONFIRM

if [ "$CONFIRM" != "yes" ]; then
    echo "Aborting. Stay safe out there!"
    exit 1
fi

echo ""
echo "Flashing to $TARGET_DEV (you may be prompted for your sudo password)..."

# Unmount any partitions on the target device before flashing
echo "Unmounting $TARGET_DEV partitions..."
sudo umount ''${TARGET_DEV}* 2>/dev/null || true

if [[ "$IMG" == *.zst ]]; then
    zstdcat "$IMG" | sudo dd of="$TARGET_DEV" bs=4M status=progress
else
    sudo dd if="$IMG" of="$TARGET_DEV" bs=4M status=progress
fi
sync

# Inform the kernel of partition changes
sudo partprobe "$TARGET_DEV" 2>/dev/null || true

echo "Flashing complete! You can safely remove the SD card."
