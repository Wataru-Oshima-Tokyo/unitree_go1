#!/usr/bin/env python
import subprocess
import os
import signal
import psutil
import time

import rospy
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

import sys

rospy.init_node('process_manager')
subprocess.Popen('gnome-terminal -- bash -c "cd $HOME/catkin_ws; source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; roslaunch unitree_a1 a1_control.launch; exec bash"', shell=True)
time.sleep(3)
subprocess.Popen('gnome-terminal -- bash -c "cd $HOME/catkin_ws; source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; roslaunch unitree_a1 navigation.launch; exec bash"', shell=True)

rospy.spin()