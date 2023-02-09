#!/usr/bin/env python3

import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

STUCK_THRESHOLD = 0.2
LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.2

class bug1():

    def __init__(self) -> None:
        self.current_position = [0, 0, 0] # initialize the current position to origin
        self.goal_position = [1.5, -1.5, 0] # initialize the goal position to origin
        self.dist_to_goal = 1.5*np.sqrt(2)# initialize the distance to the goal to zero
        self.dist_to_goal_prev = self.dist_to_goal
        self.is_stuck = False # initialize the stuck flag to False
        self.goal_reached = False
        self.stuck_count = 0
        self.heading = 0 # initialize the heading array to 0
        self.hit_point = [0, 0, 0] # initialize the hit point to origin
        self.move_the_bot = None
        self.circumnaviting = False
        self.rearrival = False
        self.start_circum_pt = None
        self.exit_circum_pt = None

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.dist_to_goal_prev = self.dist_to_goal
        self.current_position = [position.x, position.y, position.z]
        self.dist_to_goal = np.sqrt((self.current_position[0]-self.goal_position[0])**2 + (self.current_position[1]-self.goal_position[1])**2) # update the distance to the goal
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.heading = yaw

    def laserdata_callback(self, msg):
        rt = 180
        # self.move_the_bot.angular.z = 0.0
        # print(self.heading)
        msg.ranges = np.array(msg.ranges)
        print(np.count_nonzero(msg.ranges > STUCK_THRESHOLD))
        IDEAL_HEADING = np.arctan2((self.goal_position[0] - self.current_position[0]),(self.goal_position[1] - self.current_position[1]))
        # print(IDEAL_HEADING)
        if self.dist_to_goal < STUCK_THRESHOLD*3:
            print("arrived and stopped")
            self.goal_reached = True
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = 0.0

        elif not self.circumnaviting and np.abs(self.heading - IDEAL_HEADING) > np.pi/33 and np.min(msg.ranges) > STUCK_THRESHOLD:
            print("turning to head directly")
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = ANGULAR_SPEED
            
        elif not self.circumnaviting and not self.goal_reached and np.min(msg.ranges) > STUCK_THRESHOLD:
            print("moving directly towards goal")
            self.move_the_bot.linear.x = LINEAR_SPEED
            self.move_the_bot.angular.z = 0.0

        elif not self.circumnaviting and np.min(msg.ranges) < STUCK_THRESHOLD and not self.goal_reached and not self.rearrival:
            self.circumnaviting = True
            print(self.circumnaviting)
            self.start_circum_pt = self.current_position
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = 0.0
    
        elif self.circumnaviting and np.count_nonzero(msg.ranges > STUCK_THRESHOLD) < rt and not self.rearrival:
            if(self.dist_to_goal < self.dist_to_goal_prev):
                self.exit_circum_pt = self.current_position
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = ANGULAR_SPEED
        
        elif self.circumnaviting and not self.rearrival:
            if(self.dist_to_goal < self.dist_to_goal_prev):
                self.exit_circum_pt = self.current_position
            self.move_the_bot.linear.x = LINEAR_SPEED
            self.move_the_bot.angular.x = 0.0

        elif self.circumnaviting and np.sqrt((self.current_position[0]-self.start_circum_pt[0])**2 + (self.current_position[1]-self.start_circum_pt[1])**2) < STUCK_THRESHOLD:
            self.rearrival = True
            if self.circumnaviting and np.count_nonzero(msg.ranges > STUCK_THRESHOLD) >= rt and self.rearrival and np.sqrt((self.current_position[0]-self.exit_circum_pt[0])**2 + (self.current_position[1]-self.exit_circum_pt[1])**2) > STUCK_THRESHOLD:
                self.move_the_bot.linear.x = LINEAR_SPEED
                self.move_the_bot.angular.x = 0.0
            
            elif self.circumnaviting and np.count_nonzero(msg.ranges > STUCK_THRESHOLD) < rt and self.rearrival:
                self.move_the_bot.linear.x = 0.0
                self.move_the_bot.angular.z = ANGULAR_SPEED

            elif np.sqrt((self.current_position[0]-self.exit_circum_pt[0])**2 + (self.current_position[1]-self.exit_circum_pt[1])**2) < STUCK_THRESHOLD:
                self.rearrival = False
                self.circumnaviting = False
                self.move_the_bot.linear.x = 0.0
                self.move_the_bot.angular.z = ANGULAR_SPEED
        else:
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = 0.0
            print(np.min(msg.ranges))
        publish_to_cmd_vel.publish(self.move_the_bot) 


if __name__ == "__main__":
    bug1follower = bug1()
    bug1follower.move_the_bot = Twist()
    rospy.init_node('turtlebot_controller_node')
    subscribe_to_odom = rospy.Subscriber('odom', Odometry, callback= bug1follower.odom_callback)
    subscribe_to_laser = rospy.Subscriber('/scan', LaserScan, callback = bug1follower.laserdata_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.spin()


