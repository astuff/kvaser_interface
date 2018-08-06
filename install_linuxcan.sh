#!/bin/bash
set -ex

apt-get install -y software-properties-common
apt-add-repository -y ppa:jwhitleyastuff/linuxcan-dkms
apt-get update
apt-get install -y linux-image-generic linux-headers-generic linuxcan-dkms
