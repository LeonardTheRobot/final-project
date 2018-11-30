#!/usr/bin/env python
import rospy
import json
import requests
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def send_pose(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rospy.loginfo('x: {}, y: {}'.format(x, y))

    data = {"location": {"x": x, "y": y}}

    #url = 'http://coffeebot.samchatfield.com/api/robot'
    url = 'http://52.56.153.134/api/robot'
    header = {'Content-Type': 'application/json'}

    r = requests.put(url, data=json.dumps(
        data), allow_redirects=True, headers=header)
    r.raise_for_status()

def send_goal():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -7.0
    goal.target_pose.pose.position.y = 3.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action Server Not Available!")
    else:
        client.get_result()
        rospy.loginfo("Goal Execution done!")


def main():
    rospy.init_node('Client')
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, send_pose)
    send_goal()
    rospy.spin()


if __name__ == "__main__":
    main()
