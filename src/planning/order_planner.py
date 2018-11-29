#!/usr/bin/env python
import rospy
import json
import requests
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, PoseStamped

class OrderPlanner:
    def __init__(self):
        rospy.init_node("order_planner")
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, got_pose)
        rospy.Subscriber("/order_list", String, plan_orders)
        rospy.Subscriber("/found_faces", String, found_faces)
        rospy.Subscriber("/interface_response", String, on_int_response)
        
        self.order_status_publisher = rospy.Publisher("/order_status", String, queue_size=10)
        self.robot_status_publisher = rospy.Publisher("/robot_status", String, queue_size=10)
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.interface_publisher = rospy.Publisher("/interface_request", String, queue_size=10)
        
        self.x = 0.0
        self.y = 0.0
        self.order_queue = []
        

    def got_pose(msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rospy.loginfo('x: {}, y: {}'.format(x,y))
        return
        
    def plan_orders(msg):
        return
        
    def found_faces(msg):
        return

    def on_int_response(msg):
        return
        

if __name__ == "__main__":
    rospy.init_node(name='order_planner')
    order_planner = OrderPlanner()
    rospy.spin()
