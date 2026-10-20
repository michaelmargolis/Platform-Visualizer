#!/bin/sh
# install_pi3.sh ver 2
set -eu
sudo apt update
sudo apt install -y python3-pip python3-numpy libgl1 libegl1 libgles2
ARCH="$(uname -m)"
echo "Architecture: $ARCH"
if [ "$ARCH" = "aarch64" ]; then
  python3 -m pip install --user --upgrade panda3d
elif sudo apt install -y python3-panda3d 2>/dev/null; then
  echo "Installed distro Panda3D package."
else
  echo "No Panda3D package found for $ARCH. For this POC use Raspberry Pi OS 64-bit (aarch64), then rerun this script."
  exit 1
fi
python3 -c 'import numpy, panda3d; print("Panda3D and NumPy import OK")'
