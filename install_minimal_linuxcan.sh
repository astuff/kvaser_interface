#!/bin/bash
set -ex

CANLIB_LIBNAME=libcanlib.so
CANLIB_SONAME=libcanlib.so.1
CANLIB_LIBRARY=libcanlib.so.1.2.0

LINLIB_LIBNAME=liblinlib.so
LINLIB_SONAME=liblinlib.so.1
LINLIB_LIBRARY=liblinlib.so.1.2.0

cd /tmp
wget http://www.kvaser.com/software/7330130980754/V5_20_0/linuxcan.tar.gz
tar xvf linuxcan.tar.gz
cd /tmp/linuxcan
make canlib linlib

cd /tmp/linuxcan/canlib
sudo install  -m 644 $CANLIB_LIBRARY /usr/lib/
sudo ln -sf $CANLIB_LIBRARY /usr/lib/$CANLIB_LIBNAME
sudo ln -sf $CANLIB_LIBRARY /usr/lib/$CANLIB_SONAME
sudo /sbin/ldconfig
sudo install -m 644 ../include/canlib.h /usr/include
sudo install -m 644 ../include/canstat.h /usr/include
sudo install -m 644 ../include/obsolete.h /usr/include

cd /tmp/linuxcan/linlib
sudo install  -m 644 $LINLIB_LIBRARY /usr/lib/
sudo ln -sf $LINLIB_LIBRARY /usr/lib/$LINLIB_LIBNAME
sudo ln -sf $LINLIB_LIBRARY /usr/lib/$LINLIB_SONAME
sudo /sbin/ldconfig -X
sudo install -m 644 ../include/linlib.h /usr/include
