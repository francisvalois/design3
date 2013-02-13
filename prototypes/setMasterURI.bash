#!/bin/bash

ipAddress=$(ifconfig wlan0 | grep 'inet ' | awk '{ print $2}')
echo $ipAddress
export -p ROS_IP=$ipAddress
echo $ROS_IP
