#!/bin/bash

LSMOD_RET=$(lsmod | grep kvvirtualcan)

if [ -z "$LSMOD_RET" ]; then
  if [ "$EUID" -ne 0 ]; then
    sudo /usr/sbin/virtualcan.sh ${1}
  else
    /usr/sbin/virtualcan.sh ${1}
  fi
fi
