#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from pf_localisation.util import getHeading

""" The navigation stack assumes that it can send velocity 
commands using a geometry_msgs/Twist message assumed 
to be in the base coordinate frame of the robot on 
the "cmd_vel" topic. This means there must be a node 
subscribing to the "cmd_vel" topic that is capable of 
taking (vx, vy, vtheta) <==> 
(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z) 
velocities and converting them into motor commands to 
send to a mobile base. """

class BaseController:
    def __init__(self):
        #TODO - this node might not be needed