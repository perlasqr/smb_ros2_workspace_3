#!/usr/bin/env bash
set -euo pipefail

# Verify root privileges before proceeding
if [[ "$EUID" -ne 0 ]] ; then
  echo "ERROR: kasmvnc install must be run as root, please run:"
  echo "  sudo $0"
  exit 1
fi

# Get the workspace root directory
ROOT=$(dirname $(dirname $(dirname $(readlink -f $0))))
echo "Workspace root directory: ${ROOT}"

KASMVNC_VERSION="1.3.4"
source "/etc/lsb-release"

# find the architecture
ARCH=$(uname -m)
if [ "$ARCH" == "x86_64" ]; then
    ARCH="amd64"
elif [ "$ARCH" == "aarch64" ]; then
    ARCH="arm64"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

KASMVNC_DEB_FILE="https://github.com/kasmtech/KasmVNC/releases/download/v${KASMVNC_VERSION}/kasmvncserver_${DISTRIB_CODENAME}_${KASMVNC_VERSION}_${ARCH}.deb"

# Pre-configure lightdm to skip keyboard setup
echo "keyboard-configuration keyboard-configuration/layoutcode string us" | debconf-set-selections
echo "keyboard-configuration keyboard-configuration/modelcode string pc105" | debconf-set-selections
echo "keyboard-configuration keyboard-configuration/optionscode string " | debconf-set-selections
echo "keyboard-configuration keyboard-configuration/variantcode string " | debconf-set-selections
echo "keyboard-configuration keyboard-configuration/xkb-keymap select " | debconf-set-selections
echo "keyboard-configuration keyboard-configuration/unsupported_config boolean true" | debconf-set-selections
echo "keyboard-configuration keyboard-configuration/unsupported_config_options boolean true" | debconf-set-selections

wget $KASMVNC_DEB_FILE -O /tmp/kasmvnc.deb
apt-get update && apt-get install -y /tmp/kasmvnc.deb ubuntu-mate-core mate-desktop-environment-core curl wget net-tools x11-xserver-utils xserver-xorg-video-dummy expect
rm -rf /var/lib/apt/lists/*

sudo tee /etc/X11/xorg.conf > /dev/null <<EOF
Section "Device"
    Identifier  "Configured Video Device"
    Driver      "dummy"
EndSection

Section "Monitor"
    Identifier  "Configured Monitor"
    HorizSync   31.5-48.5
    VertRefresh 50-70
EndSection

Section "Screen"
    Identifier  "Default Screen"
    Monitor     "Configured Monitor"
    Device      "Configured Video Device"
    DefaultDepth 24
    SubSection "Display"
        Depth   24
        Modes   "1920x1080"
    EndSubSection
EndSection
EOF
