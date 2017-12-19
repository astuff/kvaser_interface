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
install  -m 644 $CANLIB_LIBRARY /usr/lib/
ln -sf $CANLIB_LIBRARY /usr/lib/$CANLIB_LIBNAME
ln -sf $CANLIB_LIBRARY /usr/lib/$CANLIB_SONAME
/sbin/ldconfig
install -m 644 ../include/canlib.h /usr/include
install -m 644 ../include/canstat.h /usr/include
install -m 644 ../include/obsolete.h /usr/include

cd /tmp/linuxcan/linlib
install  -m 644 $LINLIB_LIBRARY /usr/lib/
ln -sf $LINLIB_LIBRARY /usr/lib/$LINLIB_LIBNAME
ln -sf $LINLIB_LIBRARY /usr/lib/$LINLIB_SONAME
/sbin/ldconfig -X
install -m 644 ../include/linlib.h /usr/include
