---
title: "Hardware: Edge Kit Setup – NVIDIA Jetson Orin Nano ko configure karna aur OS install karna"
sidebar_label: "Hardware: Edge Kit Setup"
description: "Configuring NVIDIA Jetson Orin Nano and installing the operating system"
---

# Hardware: Edge Kit Setup

## Introduction

This module covers the setup and configuration of the NVIDIA Jetson Orin Nano, a powerful edge computing platform for robotics and AI applications.

## Unboxing and Initial Inspection

Before starting the setup, carefully unbox your Jetson Orin Nano development kit and check that all components are present:

- Jetson Orin Nano Developer Kit
- Power adapter (official NVIDIA power supply recommended)
- Micro-USB or USB-C cable
- MicroSD card (if using SD card boot)
- Quick start guide
- Heat spreader/heat sink (may be pre-attached)

## Hardware Requirements

### Minimum Requirements
- Host computer (Windows, Linux, or macOS) for flashing
- MicroSD card (at least 16GB, Class 10 recommended)
- High-speed USB cable
- Monitor with HDMI input
- USB keyboard and mouse
- Ethernet cable (recommended for initial setup)

### Power Requirements
The Jetson Orin Nano has specific power requirements:
- 19V DC power supply
- Minimum 6A for 15W mode
- Minimum 10A for 25W mode
- Official NVIDIA power adapter recommended

## Initial Hardware Setup

### 1. Attach Heat Spreader
If not pre-attached, carefully attach the heat spreader to the SoC (System on Chip):
- Remove protective film from thermal pad
- Align heat spreader with the SoC
- Secure with provided screws (do not over-tighten)

### 2. Prepare Power Supply
- Use the official NVIDIA power adapter
- Connect the barrel jack to the Jetson board
- Ensure the power switch is in the OFF position initially

### 3. Connect Peripherals
- HDMI display
- USB keyboard and mouse
- Ethernet cable (recommended for internet access)
- Power adapter

## Flashing the Operating System

### Method 1: Using NVIDIA SDK Manager (Recommended)

1. **Install SDK Manager** on your host computer:
   - Download from NVIDIA Developer website
   - Install on Windows, Linux, or macOS

2. **Prepare your host computer**:
   - Enable USB debugging if required
   - Install required drivers

3. **Connect Jetson to host**:
   - Use USB cable to connect Jetson to host computer
   - Connect power supply to Jetson

4. **Boot Jetson in recovery mode**:
   - Ensure power switch is OFF
   - Press and hold REC (recovery) button
   - While holding REC, press and release the power button
   - Continue holding REC for 3-5 seconds after power LED lights up
   - Release REC button

5. **Flash using SDK Manager**:
   - Launch SDK Manager
   - Sign in with NVIDIA Developer account
   - Select Jetson Orin Nano target
   - Select JetPack version
   - Start flashing process

### Method 2: Using SD Card Image

1. **Download JetPack SD Card Image** from NVIDIA Developer website
2. **Flash the image** to your microSD card using tools like:
   - balenaEtcher
   - Rufus
   - dd command (Linux/macOS)

3. **Insert SD card** into Jetson Orin Nano
4. **Configure boot switch** to SD card mode (if applicable)
5. **Power on** the device

## Initial Configuration

### First Boot
- Connect to HDMI display
- System will boot and show initial setup wizard
- Follow prompts to:
  - Select language
  - Connect to Wi-Fi or Ethernet
  - Create user account
  - Set up password

### Initial Commands
After initial boot, open a terminal and run:

```bash
# Check system information
uname -a
cat /etc/os-release
sudo jetson_release

# Update package list
sudo apt update

# Upgrade system packages (optional but recommended)
sudo apt upgrade
```

## Essential Software Installation

### Install Development Tools
```bash
# Install build essentials
sudo apt install build-essential

# Install Python development tools
sudo apt install python3-dev python3-pip

# Install version control
sudo apt install git

# Install ROS 2 dependencies
sudo apt install python3-rosdep python3-rosinstall python3-vcstool
```

### Install Jetson Inference
For AI and computer vision applications:
```bash
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```

## Network Configuration

### Wired Connection
The Jetson Orin Nano typically connects automatically via Ethernet. Check connection:
```bash
ip addr show
```

### Wireless Connection
To connect to Wi-Fi:
```bash
nmcli device wifi list
nmcli device wifi connect "Network_Name" password "Password"
```

## Performance Modes

The Jetson Orin Nano supports different power modes:

### Check Current Mode
```bash
sudo nvpmodel -q
```

### Set Performance Mode
```bash
# Set to MAXN mode (highest performance)
sudo nvpmodel -m 0

# Set to 15W mode (lower power)
sudo nvpmodel -m 1
```

### Check Jetson Clocks
```bash
sudo jetson_clocks
```

## Common Troubleshooting

### Boot Issues
- Ensure proper power supply (use official adapter)
- Check all connections
- Verify SD card is properly seated

### Overheating
- Ensure heat spreader is properly attached
- Use active cooling if needed
- Monitor temperature: `cat /sys/devices/virtual/thermal/thermal_zone*/temp`

### Network Issues
- Check physical connections
- Verify network configuration
- Try connecting via Ethernet first

## Verification Steps

After setup, verify your Jetson Orin Nano is working correctly:

```bash
# Check system status
sudo systemctl status

# Check GPU status
nvidia-smi

# Check available storage
df -h

# Check memory
free -h

# Test Python
python3 --version

# Test camera (if connected)
nvgstcapture-1.0
```

## Next Steps

Your Jetson Orin Nano is now ready for robotics and AI development:
- Install ROS 2 for robotics applications
- Set up development environment
- Connect sensors and actuators
- Start developing robotics applications

This completes the setup of your NVIDIA Jetson Orin Nano for robotics applications.