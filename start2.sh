#!/bin/bash
router_address="172.20.10.1"
while :
do
  sudo ping -c 2 -w 1 $router_address > /dev/null 2>&1 
  if [ $? -ne 0 ]; then
  sleep 30
    sudo ifconfig wlan0 up
  fi
  sleep 5
done
