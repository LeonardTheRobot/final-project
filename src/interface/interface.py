#!/usr/bin/env python
import rospy
import json
import requests
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, PoseStamped

class RobotInterface:
    def __init__(self):
        rospy.init_node("robot_interface")
        rospy.Subscriber("/interface_request", String, on_request)
        
        self.response_publisher = rospy.Publisher("/interface_response", String, queue_size=10)
        

    def on_request(msg):
        return
        

if __name__ == "__main__":
    rospy.init_node(name='robot_interface')
    interface = RobotInterface()
    rospy.spin()
