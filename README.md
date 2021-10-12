# Kvaser ROS Interface API

[![CircleCI](https://circleci.com/gh/astuff/kvaser_interface/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/kvaser_interface/tree/master)

This package was developed as a standardized way to access Kvaser CAN devices from ROS. It can either be used as a development API
by including the header <kvaser_interface/kvaser_interface.h> and linking against `libros_linuxcan.so` or the stand-alone node
`kvaser_can_bridge` can communicate with a CAN device independently.

## Installation

The `kvaser_interface` package depends on the Kvaser CANLIB API. 
You can install the Kvaser CANLIB from source [directly from Kvaser](https://www.kvaser.com/downloads/), however the easiest way to install is using our ppa which distributes them as deb packages:

```sh
sudo apt-add-repository ppa:astuff/kvaser-linux
sudo apt update
sudo apt install kvaser-canlib-dev kvaser-drivers-dkms
```

Now that the dependencies are installed, we can install `kvaser_interface`:


```sh
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-kvaser-interface
```

## The `kvaser_can_bridge` Node

**TOPICS**

*can_tx* [can_msgs::Frame]

This topic is published by the node. It expects to have other nodes subscribe to it to receive data which are *sent by the CAN device*.

*can_rx* [can_msgs::Frame]

This topic is subscribed to by the node. It expects to have data published to it which are intended to be *received by the CAN device*.

**PARAMETERS**

*~can_hardware_id*

This is the Kvaser Hardware ID (serial number) of the connected device.

*~can_circuit_id*

This is the 0-based index of the channel number *on the specific hardware device* designated by the *~can_hardware_id*.

*~can_bit_rate*

This is the communication rate to be used on the CAN channel in bits per second (default: 500000).

**TOOLS**

*list_channels*

A simple tool with no ROS dependencies which returns a list of all Kvaser devices and channels available on those devices.

*canmonitor*

A simple tool with no ROS depedencies to output the CAN messages received on a given channel index (-i - default: 0) and bitrate (-b - default: 500000) to the terminal.
