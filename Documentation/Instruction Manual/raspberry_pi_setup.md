# Raspberry Pi 5 Setup

This section sets up your Raspberry Pi for the E-Rocket project. 
Note that ROS 2 versions target particular Ubuntu versions. 
As of now, the oldest supported version of Ubuntu for this setup is 24.04 LTS. 
We're using ROS 2 "Jazzy" to match Ubuntu 24.04 LTS. 

Note that some of these command only work for the Raspberry Pi 5, and may not work on previous versions of the Raspberry Pi. 

## Requirements:

    - Raspberry Pi 5 with 8GB RAM
    - MicroSD card (at least 32GB)
    - USB-C power cable for Raspberry Pi
    - Monitor, keyboard, and mouse (for initial setup)
    - Internet connection (via Ethernet or Wi-Fi). 
    - Computer with SSH access to pi, for remote management. 
    - Cable connecting the Pixhawk TELEM2 port to the RPi GPIO Pins. [link](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html\#serial-connection)

## Install Ubuntu on the RPi

This tutorial is heavily inspired in [PX4 Docs: Ubuntu Setup on RPi](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html#ubuntu-setup-on-rpi). 
The PX4 tutorial also includes configuration for the PX4 to connect via ROS2. 
Make sure the PX4 is propertly configured before attempting to communicate with it via uXRCE-DDS. 

Follow the official instructions to flash Ubuntu onto your MicroSD card, in [How to install Ubuntu Desktop on Raspberry Pi 4](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4).

Update and Upgrade the System
Begin by updating the package list and upgrading installed packages. Open a terminal and run the following commands:

```
    sudo apt update
    sudo apt upgrade -y
```

### Remove unnecessary Packages that come with Ubuntu by default

Honorable mention to PegasusResearch software setup tutorial. [link](https://pegasusresearch.github.io/pegasus/source/vehicles/pegasus/software.html#removing-pre-installed-software)

```
    sudo apt purge thunderbird libreoffice-* onboard aisleriot gnome-sudoku gnome-mines gnome-mahjongg cheese gnome-calculator gnome-todo shotwell gnome-calendar rhythmbox simple-scan remmina transmission-gtk -y 
    sudo apt autoremove 
    sudo apt clean 
```

### Install Required Packages

```
    sudo apt install openssh-server nano python3 python3-pip python3-colcon-common-extensions git curl nmap -y
```

### Enable Serial Port

Based on [video](https://www.youtube.com/watch?v=muOwRRSm2do). 
1. Open the firmware boot configuration file in the nano editor on RPi

```
    sudo nano /boot/firmware/config.txt
```

2. Append the following text to the end of the file (after the last line):

```
    dtparam=uart0_console
```

3. Check that the serial port is available. In this case we use the following terminal commands to list the serial devices:

```
    cd /
    ls /dev/ttyAMA0
```
The result of the command should include the RX/TX connection /dev/ttyAMA0.

### Enable SSH

Run the following commands on the terminal:

```
    sudo touch /boot/firmware/ssh
    sudo systemctl enable ssh
    sudo systemctl start ssh
    sudo reboot
```

### (Optional) Install mprocs CLI
The CLI is a quality of life tool used by the code author to create multiple terminals. This tool is optional, and only serves to automate setup, compilation and launch of ros2 workspace. 
The author will be installing the tool using [brew](https://brew.sh/), but other instalation methods exist. 
The instalation command is present in [mprocs Github: Instalation](https://github.com/pvolok/mprocs?tab=readme-ov-file#installation). 

## Ros2 Setup on RPi

The steps to setup ROS 2 and the Micro XRCE-DDS Agent on the RPi are:

1. Install ROS 2 Jazzy by following the [official tutorial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

2. Install the uXRCE_DDS agent:

```
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
```
See [uXRCE-DDS > Micro XRCE-DDS Agent Installation](https://docs.px4.io/main/en/middleware/uxrce_dds.html#micro-xrce-dds-agent-installation) for alternative ways of installing the agent.

3. Start the agent in the RPi terminal:

```
    sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

Now that both the agent and client are running, you should see activity on the terminal. You can view the available topics using the following command on the RPi:

```
    source /opt/ros/jazzy/setup.bash
    ros2 topic list
```

## MAVLink Setup

The team used MAVLink to forward QGroundControl (QGC) telemetry data from the Pixhawk through the RPi to the PC.
First, we need to enable the serial port on the RPi: 

```
!TODO!

```

To install, follow the official instructions in [MAVLink Router Website](https://ardupilot.org/mavproxy/docs/getting_started/index.html). 


## Clone the Project Repository

```
    git clone https://github.com/PedromcaMartins/e-rocket.git
    cd e-rocket/
    mprocs
```
Now, test if the setup was successful. 

## Reboot the Raspberry Pi

Finally, reboot the Raspberry Pi to apply all changes:

```
    sudo reboot
```

Your Raspberry Pi is now set up and ready for the E-Rocket project.
