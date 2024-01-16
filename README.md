# Kvaser ROS Interface API

[![CircleCI](https://circleci.com/gh/astuff/kvaser_interface/tree/ros2_master.svg?style=svg)](https://circleci.com/gh/astuff/kvaser_interface/tree/ros2_master)

This package was developed as a standardized way to access Kvaser CAN devices from ROS 2. It can either be used as a development API
by including the appropriate header <kvaser_interface/kvaser_reader_node.hpp> or <kvaser_interface/kvaser_writer_node.hpp> and linking
against `kvaser_interface::kvaser_reader_node` or `kvaser_interface::kvaser_writer_node` or by using the stand-alone node `kvaser_can_bridge`.

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

## TOPICS

*can_tx* [can_msgs::msg::Frame]
*can_tx* [can_msgs::FrameFd]

This topic is published by the kvaser_reader_node. It expects to have other nodes subscribe to it to receive data which are *sent by the CAN device*.

*can_rx* [can_msgs::msg::Frame]
*can_rx* [can_msgs::FrameFd]

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

*canfd*

This is the canfd flag (0 : not canfd, 1 : canfd).

*canfd_tseg1*

Time segment 1, that is, the number of quanta from (but not including) the Sync Segment to the sampling point (default: 15).

*canfd_tseg2*

Time segment 2, that is, the number of quanta from the sampling point to the end of the bit (default: 4).

*canfd_sjw*

The Synchronization Jump Width (default: 4).

*canfd_data_rate*

The canfd data rate (defalut : 2000000)


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
