#!/usr/bin/env python
import rospy
import json
import requests
import actionlib
import thread
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

# Client used for web communication with the server
class Client:
    def __init__(self):
        rospy.init_node("Client")
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.send_pose)
        rospy.Subscriber("/order_status", String, self.update_status)

        self.pub = rospy.Publisher("/order_list", String, queue_size=10)

        #self.url = "http://coffeebot.samchatfield.com/api/robot"
        self.pose_url = "http://52.56.153.134/api/robot"
        self.order_url = "http://52.56.153.134/api/orders"

        self.x = 0.0
        self.y = 0.0
        self.delay = 5 # seconds
        self.order_status = ""
        self.poseSet = False
        self.statusSet = False
        
        try:
            thread.start_new_thread(self.client_thread, ())
        except Exception as e:
            rospy.logerr("Error: Unable to start client thread")
            rospy.logerr(e)
        rospy.spin()


    def send_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.poseSet = True
        #rospy.loginfo("x: {}, y: {}".format(self.x, self.y))
        return

    def update_status(self, msg):
        self.order_status = msg.data
        self.statusSet = True
        
        print(self.order_status)
        jsonDict = json.loads(self.order_status)
        statusUrl = self.order_url + '/' + jsonDict["_id"]
        status = {"status": jsonDict["status"]}
        status_request = requests.put(statusUrl, json=status)
        status_request.raise_for_status()
        
        return

    def client_thread(self):
        while(True):
            header = {"Content-Type": "application/json"}
            if self.poseSet:
                data = {"location": {"x": self.x, "y": self.y}}
                # pose_request = requests.put(self.pose_url, data=data, allow_redirects=True, headers=header)
                pose_request = requests.put(self.pose_url, json=data)
                pose_request.raise_for_status()
                
            """if self.statusSet:
                print(self.order_status)
                jsonDict = json.loads(self.order_status)
                statusUrl = self.order_url + '/' + jsonDict["_id"]
                status = {"status": jsonDict["status"]}
                status_request = requests.put(statusUrl, json=status)
                status_request.raise_for_status()"""

            orders_request = requests.get(self.order_url)
            self.pub.publish(orders_request.text)
            time.sleep(5)
        return

if __name__ == "__main__":
    client = Client()
