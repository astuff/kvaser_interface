# Kvaser ROS Interface API

[![CircleCI](https://circleci.com/gh/astuff/kvaser_interface/tree/dashing-devel.svg?style=svg)](https://circleci.com/gh/astuff/kvaser_interface/tree/dashing-devel)

This package was developed as a standardized way to access Kvaser CAN devices from ROS 2. It can either be used as a development API
by including the appropriate header <kvaser_interface/kvaser_reader_node.hpp> or <kvaser_interface/kvaser_writer_node.hpp> and linking
against `kvaser_interface::kvaser_reader_node` or `kvaser_interface::kvaser_writer_node` or by using the stand-alone node `kvaser_can_bridge`.

The following are required prerequisites:

* The Kvaser CANLIB API (https://www.kvaser.com/downloads/)
    * Can be [downloaded directly](https://www.kvaser.com/kvaser-downloads) or installed through PPA:

        `sudo apt-add-repository ppa:jwhitleyastuff/kvaser-linuxcan`

        `sudo apt update && sudo apt install -y kvaser-canlib-dev`
    * You can optionally install the Kvaser drivers from the PPA with the following command:

        `sudo apt install -y kvaser-drivers-dkms`
* `can_msgs` from [`ros_canopen`](https://github.com/JWhitleyAStuff/ros_canopen/tree/dashing-devel) (not yet available on the Build Farm)

## TOPICS

*can_tx* [can_msgs::msg::Frame]

This topic is published by the kvaser_reader_node. It expects to have other nodes subscribe to it to receive data which are *sent by the CAN device*.

*can_rx* [can_msgs::msg::Frame]

This topic is subscribed to by the kvaser_writer_node. It expects to have data published to it which are intended to be *received by the CAN device*.

## NODE PARAMETERS

### kvaser_reader_node / kvaser_writer_node

*hardware_id*

This is the Kvaser Hardware ID (serial number) of the connected device.

*circuit_id*

This is the 0-based index of the channel number *on the specific hardware device* designated by the *hardware_id*.

*bit_rate*

This is the communication rate to be used on the CAN channel in bits per second (default: 500000).

*enable_echo*

If this parameter is set to *true* and both the `kvaser_reader_node` and `kvaser_writer_node` are connected to the same
*hardware_id* and *circuit_id*, then messages sent by the `kvaser_writer_node` will be echoed to the `kvaser_reader_node` (default: false).

## NODES

### kvaser_reader_node (Component)

Reads data from a Kvaser CAN device.

### kvaser_writer_node (Component)

Writes data to a Kvaser CAN device.

### kvaser_can_bridge

Composes both of the above components into a single thread/node.

### list_channels

A simple tool with no ROS dependencies which returns a list of all Kvaser devices and channels available on those devices.

### canmonitor

A simple tool with no ROS depedencies to output the CAN messages received on a given channel index.

#### Flags

- *-i* - Index - default: 0
- *-b* - Bitrate - default: 500000
