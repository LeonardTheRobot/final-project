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
        
        self.order_status_publisher = rospy.Publisher("/order_status", String, queue_size=10)
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        
        self.x = 0.0
        self.y = 0.0
        self.order_queue = [] # list of order ids
        
        
        

    def got_pose(msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rospy.loginfo('x: {}, y: {}'.format(x,y))
        return
        
    def plan_orders(msg):
        orders = json.loads(msg)
        for order in orders:
            order_x = order["location"]["x"]
            order_y = order["location"]["y"]
        return
        
    def found_faces(msg):
        user = msg
        users_orders = [item for item in order_queue if item["user"] == user]
        for order in users_orders:
            order["status"] = "COLLECTION"
            order_status_publisher.publish(json.dumps(order))
        return
        

if __name__ == "__main__":
    rospy.init_node(name='order_planner')
    order_planner = OrderPlanner()
    rospy.spin()
