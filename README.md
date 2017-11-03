# Kvaser ROS Interface API

[![Build Status](https://travis-ci.org/astuff/kvaser_interface.svg?branch=master)](https://travis-ci.org/astuff/kvaser_interface)

This package was developed as a standardized way to access Kvaser CAN devices from ROS. It can either be used as a development API
by including the header <kvaser_interface/kvaser_interface.h> and linking against `libros_linuxcan.so` or the stand-alone node
`kvaser_can_bridge` can communicate with a CAN device independently.

The following are required prerequisites:

* The Kvaser CANLIB API (https://www.kvaser.com/downloads/)
    * For Ubuntu 14.04/16.04 - Latest version (tested with at least v5.20.814)
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

This is the communication rate to be used on the CAN channel in bits per second.
