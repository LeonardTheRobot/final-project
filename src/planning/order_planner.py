#!/usr/bin/env python
import rospy
import json
import requests
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, PoseStamped
from datetime import datetime
import math
import time
import copy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

class OrderPlanner:
    def __init__(self):
        rospy.init_node('order_planner')
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.got_pose)
        rospy.Subscriber('/order_list', String, self.update_order_list)
        rospy.Subscriber('/faceID', String, self.found_faces)
        
        self.order_status_publisher = rospy.Publisher('/order_status', String, queue_size=10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        self.x = 0.0
        self.y = 0.0
        self.order_list = []
        self.collection_list = []
        self.zones = {}
        self._get_zones()

        while(True):
            self.order_loop()
    
    def _get_zones(self):
        res = requests.get('http://52.56.153.134/javascripts/zones.json')
        res.raise_for_status()
        for zone in res.json():
            self.zones[zone['name']] = zone

    def got_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        return
        
    def update_order_list(self, msg):
        """Update our internal order list every time we get a new message on the topic"""
        self.order_list = json.loads(msg.data)
        
    def found_faces(self, msg):
        if len(self.collection_list) > 0:
            user = msg.data
            users_orders = [item for item in self.collection_list if item['user'] == user]
            if len(users_orders) == 0:
                print "You haven't placed an order %s!" % (user)
            else:
                raw_input("Hi %s, pick up your order now! Once you've got it, press enter" % (user))
                for order in users_orders:
                    order['status'] = 'COMPLETED'
                    self.order_status_publisher.publish(json.dumps(order))
                    self.collection_list.remove(order)
    
    def order_loop(self):
        """The main loop which performs order planning which loops on order completion"""
        
        in_progress_orders = filter(lambda o: o['status'] == 'IN_PROGRESS', self.order_list)
        pending_orders = filter(lambda o: o['status'] == 'PENDING', self.order_list)
        # Plan the pending orders setting order_queue
        order_queue = in_progress_orders + self.plan_orders(pending_orders)

        if len(order_queue) == 0:
            return

        print('Order queue: {}'.format(order_queue))
        goal_zone = order_queue[0]['zone']
        for order in order_queue:
            if order['zone'] == goal_zone:
                order['status'] = 'IN_PROGRESS'
                self.order_status_publisher.publish(json.dumps(order))
            else:
                break
        self.set_goal(order_queue[0])
        print('Out of set goal function')

        # Change the status of the orders at the goal to COLLECTION
        if order_queue[0]['zone'] == 'Pickup':
            for entry in self.inventory:
                entry["quantity"] = 5
            raw_input("Please refill me, then press enter")
            
        else:
            for order in order_queue:
                if order['zone'] == goal_zone:
                    order['status'] = 'COLLECTION'
                    self.order_status_publisher.publish(json.dumps(order))
                    self.collection_list.append(order)
                else:
                    break
            # Wait for the collection list to be emptied by the facial recognition
            t0 = time.time()
            while len(self.collection_list) > 0:
                t1 = time.time()
                if t1 - t0 > 30:
                    break
                time.sleep(2)
            for order in self.collection_list:
                order['status'] = 'FAILED'
                self.order_status_publisher.publish(json.dumps(order))
            self.collection_list = []
    
    def plan_orders(self, pending_orders):
        # TODO: account for inventory, and refilling when required
        current_time = datetime.now()
        order_queue = []
        
        res = requests.get('http://52.56.153.134/api/robot')
        res.raise_for_status()
        self.inventory = res.json()["inventory"]
        working_inv = copy.deepcopy(self.inventory)

        while pending_orders:
            if not order_queue:
                # From the robot's current position
                current_x, current_y = self.x, self.y
            else:
                # From the previous order in the queue
                previous_order_zone = self.zones[order_queue[-1]['zone']]
                current_x, current_y = previous_order_zone['x'], previous_order_zone['y']
            
            best_order = None # Tuple (order, weight)
            for order in pending_orders:
                required_items = {}
                refill = False
                for item in order["items"]:
                    try:
                        required_items[item] += 1
                    except KeyError:
                        required_items[item] = 1
                
                for item, quantity in required_items.iteritems():
                    for entry in working_inv:
                        if entry["item"] == item and quantity > entry["quantity"]:
                            refill = True
            
                weight = self.compute_weight(current_x, current_y, current_time, order, refill)
                if not best_order or weight < best_order[1]:
                    best_order = (order, weight, refill)
            
            if best_order[2]:
                for entry in working_inv:
                    entry["quantity"] = 5
                order_queue.append({"zone": "Pickup"})
                
            required_items = {}
            for item in best_order[0]:
                try:
                    required_items[item] += 1
                except KeyError:
                    required_items[item] = 1
            
            for item, quantity in required_items.iteritems():
                for entry in working_inv:
                    if entry["item"] == item:
                        entry["quantity"] -= quantity
            
            order_queue.append(best_order[0])
            pending_orders.remove(best_order[0])

        return order_queue

    def compute_weight(self, current_x, current_y, current_t, order, refill):
        order_zone = self.zones[order['zone']]
        
        if refill:
            distance = math.sqrt((self.zones["Pickup"]["x"] - current_x) ** 2 + (self.zones["Pickup"]["y"] - current_y) ** 2)
            distance += math.sqrt((order_zone['x'] - self.zones["Pickup"]["x"]) ** 2 + (order_zone['y'] - self.zones["Pickup"]["y"]) ** 2)
        else:
            distance = math.sqrt((order_zone['x'] - current_x) ** 2 + (order_zone['y'] - current_y) ** 2)

        order_datetime = datetime.strptime(order['orderedAt'], '%Y-%m-%dT%H:%M:%S.%fZ')
        elapsed_seconds = (current_t - order_datetime).total_seconds()

        return distance - 0.0075 * elapsed_seconds

    def set_goal(self, order):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal_zone = self.zones[order['zone']]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_zone['x']
        goal.target_pose.pose.position.y = goal_zone['y']
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action Server Not Available!")
        else:
            client.get_result()
            rospy.loginfo("Goal Execution done!")

if __name__ == '__main__':
    rospy.init_node(name='order_planner')
    order_planner = OrderPlanner()
    rospy.spin()
