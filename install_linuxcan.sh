#!/bin/bash
set -ex

apt-get install -y software-properties-common linux-image-generic linux-headers-generic
apt-add-repository -y ppa:jwhitleyastuff/kvaser-linuxcan-dkms
apt-get update
apt-get install -y linuxcan-dkms
