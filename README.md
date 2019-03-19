# Kvaser ROS Interface API

[![CircleCI](https://circleci.com/gh/astuff/kvaser_interface/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/kvaser_interface/tree/master)

This package was developed as a standardized way to access Kvaser CAN devices from ROS. It can either be used as a development API
by including the header <kvaser_interface/kvaser_interface.h> and linking against `libros_linuxcan.so` or the stand-alone node
`kvaser_can_bridge` can communicate with a CAN device independently.

The following are required prerequisites:

* The Kvaser CANLIB API (https://www.kvaser.com/downloads/)
    * Can be [downloaded directly](https://www.kvaser.com/kvaser-downloads) or installed through PPA:

        `sudo apt-add-repository ppa:jwhitleyastuff/linuxcan-dkms`

        `sudo apt update && sudo apt install -y linuxcan-dkms`
    * For Linux kernel 4.13 or higher, version 5.21 or higher of CANLIB is required
* `can_msgs`

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
