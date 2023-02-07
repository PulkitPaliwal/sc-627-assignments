#!/usr/bin/env python3

import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

STUCK_THRESHOLD = 0.1
LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.1

class bug1():

    def __init__(self) -> None:
        self.current_position = [0, 0, 0] # initialize the current position to origin
        self.goal_position = [0, 0, -1.2] # initialize the goal position to origin
        self.dist_to_goal = 1.2 # initialize the distance to the goal to zero
        self.dist_to_goal_prev = self.dist_to_goal
        self.is_stuck = False # initialize the stuck flag to False
        self.stuck_count = 0
        self.heading = [] # initialize the heading array to empty
        self.hit_point = [0, 0, 1.2] # initialize the hit point to origin

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.dist_to_goal_prev = self.dist_to_goal
        self.current_position = [position.x, position.y, position.z]
        self.dist_to_goal = np.sqrt((self.current_position[0]-self.goal_position[0])**2 + (self.current_position[1]-self.goal_position[1])**2) # update the distance to the goal

    def laserdata_callback(self, msg):
        # Check if the robot is stuck, i.e., the distance to the goal has not decreased for a certain period of time
        if self.dist_to_goal >= self.dist_to_goal_prev:
            self.stuck_count += 1
            if self.stuck_count >= STUCK_THRESHOLD:
                self.is_stuck = True
                self.hit_point = self.current_position # set the hit point to the current position
        else:
            move_the_bot.linear.x = 0.0
            move_the_bot.angular.z = 0.0
            self.stuck_count = 0
            self.is_stuck = False
        
        # if the robot is stuck, perform the bug-1 algorithm
        if self.is_stuck:
            if self.dist_to_goal >= np.sqrt((self.hit_point[0]-self.goal_position[0])**2 + (self.hit_point[1]-self.goal_position[1])**2):
                # if the robot has passed the hit point, move towards the goal
                move_the_bot.linear.x = LINEAR_SPEED
                move_the_bot.angular.z = 0.0
            else:
                # if the robot has not passed the hit point, move along the boundary
                move_the_bot.linear.x = LINEAR_SPEED
                move_the_bot.angular.z = ANGULAR_SPEED
        elif self.dist_to_goal > 0.2:
            # if the robot is not stuck, move towards the goal
            move_the_bot.linear.x = 0.0
            move_the_bot.angular.z = 0.0

        publish_to_cmd_vel.publish(move_the_bot) 


if __name__ == "__main__":
    bug1follower = bug1()
    rospy.init_node('turtlebot_controller_node')
    subscribe_to_odom = rospy.Subscriber('odom', Odometry, callback= bug1follower.odom_callback)
    subscribe_to_laser = rospy.Subscriber('/scan', LaserScan, callback = bug1follower.laserdata_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    move_the_bot = Twist()
    rospy.spin()


