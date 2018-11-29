#!/usr/bin/env python
import rospy
import json
import requests
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose

def send_pose(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rospy.loginfo('x: {}, y: {}'.format(x,y))

    data = {"location": {"x": x, "y": y}}

    #url = 'http://coffeebot.samchatfield.com/api/robot'
    url = 'http://52.56.153.134/api/robot'
    header = {'Content-Type': 'application/json'}

    r = requests.put(url, data=json.dumps(data), allow_redirects=True, headers=header)
    r.raise_for_status()

def main():
    rospy.init_node('Client')
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, send_pose)
    rospy.spin()

if __name__ == "__main__":
    main()