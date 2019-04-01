#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  sudo /usr/sbin/virtualcan.sh ${1}
else
  /usr/sbin/virtualcan.sh ${1}
fi
