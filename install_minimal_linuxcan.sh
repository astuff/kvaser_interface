#!/bin/bash
set -ex

#CANLIB_LIBNAME=libcanlib.so
#CANLIB_SONAME=libcanlib.so.1
#CANLIB_LIBRARY=libcanlib.so.1.4.0

#LINLIB_LIBNAME=liblinlib.so
#LINLIB_SONAME=liblinlib.so.1
#LINLIB_LIBRARY=liblinlib.so.1.4.0

#cd /tmp
#wget https://www.kvaser.com/downloads-kvaser/?d_version_id=1193 -O linuxcan.tar.gz
#tar xvf linuxcan.tar.gz
#cd /tmp/linuxcan
#make canlib linlib

#cd /tmp/linuxcan/canlib
#install  -m 644 $CANLIB_LIBRARY /usr/lib/
#ln -sf $CANLIB_LIBRARY /usr/lib/$CANLIB_LIBNAME
#ln -sf $CANLIB_LIBRARY /usr/lib/$CANLIB_SONAME
#/sbin/ldconfig
#install -m 644 ../include/canlib.h /usr/include
#install -m 644 ../include/canstat.h /usr/include
#install -m 644 ../include/obsolete.h /usr/include

#cd /tmp/linuxcan/linlib
#install  -m 644 $LINLIB_LIBRARY /usr/lib/
#ln -sf $LINLIB_LIBRARY /usr/lib/$LINLIB_LIBNAME
#ln -sf $LINLIB_LIBRARY /usr/lib/$LINLIB_SONAME
#/sbin/ldconfig -X
#install -m 644 ../include/linlib.h /usr/include

apt-get install -y software-properties-common
apt-add-repository ppa:jwhitleyastuff/kvaser-linuxcan-dkms
apt-get update
apt-get install -y linuxcan-dkms
