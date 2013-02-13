source /opt/ros/groovy/setup.bash

ipAddress=$(ifconfig wlan0 | grep 'inet ' | awk '{ print $2}')
export -p ROS_IP=$ipAddress
echo $ROS_IP
export -p ROS_MASTER_URI=http://$ipAddress:11311
echo $ROS_MASTER_URI

source /home/design3/design3/projet/devel/setup.bash
roscore &
sleep 5
rosrun kinocto talker  &

