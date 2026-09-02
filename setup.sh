#!/bin/bash

#For WSL, uncomment if unnecessary
# sudo usbip attach -r 127.0.0.1 -b 4-1

# Add a restricted udev rule for the camera.
# udevadm info --name /dev/bus/usb/003/026 --attribute-walk
echo SUBSYSTEM=='"usb"', ATTR{idVendor}=='"2e1a"', SYMLINK+='"insta"', GROUP='"plugdev"', MODE='"0660"' | sudo tee /etc/udev/rules.d/99-insta.rules
#Reload and trigger udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Wait for the symlink to be created
for i in {1..5}; do
    if [ -e /dev/insta ]; then
        echo "Camera is available at /dev/insta for members of the plugdev group."
        echo "If access is denied, run: sudo usermod -aG plugdev $USER"
        echo "Then log out and back in."
        break
    else
        echo "Waiting for /dev/insta to be created... (attempt $i/5)"
        sleep 1
    fi
done

if [ ! -e /dev/insta ]; then
    echo "Error: /dev/insta was not created. Please check if the camera is connected and in the correct mode."
    exit 1
fi
