#!/usr/bin/env python3

import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

STUCK_THRESHOLD = 0.2
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.2

class bug1():

    def __init__(self) -> None:
        self.LINE_IDENTIFIER = 0
        self.current_position = [0, 0, 0] # initialize the current position to origin
        self.goal_position = [2.5, -2.5, 0] 
        self.dist_to_goal = np.sqrt((self.goal_position[0]-self.current_position[0])**2 + (self.goal_position[1]-self.current_position[1])**2)
        self.dist_to_goal_prev = self.dist_to_goal
        self.is_stuck = False # initialize the stuck flag to False
        self.goal_reached = False
        self.found_exit = False
        self.stuck_count = 0
        self.heading = 0 # initialize the heading array to 0
        self.hit_point = [0, 0, 0] # initialize the hit point to origin
        self.move_the_bot = None
        self.circumnavigating = False
        self.rearrival = False
        self.start_circum_pt = None
        self.exit_circum_pt = None
        self.count = 0
        self.IDEAL_HEADING = None
        self.turning = False
        self.distance_to_closest_obs = None
        self.angle_of_closest_obs = None

    def wall_follow(self, laserdata):
        # need to maintain left region values
        # if not self.turning:
        #     self.angle_of_closest_obs = np.argmin(laserdata.ranges)
        #     print(self.angle_of_closest_obs)
        #     self.IDEAL_HEADING  = (self.heading + self.angle_of_closest_obs*np.pi/180 - np.pi/2 - np.pi/90) %(2*np.pi)
        #     print(self.IDEAL_HEADING*180/np.pi)
        #     print(self.heading*180/np.pi)

        # if np.abs((self.heading - self.IDEAL_HEADING)%(2*np.pi)) > np.pi/36:
        #     self.turning = True
        #     print("turning to head along wall")
        #     print("Ideal Heading: " + str(self.IDEAL_HEADING) + " Current Heading: " + str(self.heading))
        #     self.move_the_bot.linear.x = 0.0
        #     self.move_the_bot.angular.z = ANGULAR_SPEED

        if np.abs(np.argmin(laserdata.ranges) - 80) > 5:
            self.turning = True
            print(np.argmin(laserdata.ranges))
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = ANGULAR_SPEED

        else:
            self.turning = False
            print("moving directly towards next point on boundary")
            self.move_the_bot.linear.x = 0.5*LINEAR_SPEED
            self.move_the_bot.angular.z = 0.0

        # print("using this")
        
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.dist_to_goal_prev = self.dist_to_goal
        self.current_position = [position.x, position.y, position.z]
        self.dist_to_goal = np.sqrt((self.current_position[0]-self.goal_position[0])**2 + (self.current_position[1]-self.goal_position[1])**2) # update the distance to the goal
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.heading = yaw%(2*np.pi)
        # print(yaw)

    def laserdata_callback(self, msg):
        rt = 180
        # self.move_the_bot.angular.z = 0.0
        # print(self.heading)
        # print(self.current_position)
        msg.ranges = np.array(msg.ranges)
        # print(np.count_nonzero(msg.ranges > STUCK_THRESHOLD))
        if not self.circumnavigating:
            self.IDEAL_HEADING = np.arctan2((self.goal_position[1] - self.current_position[1]),(self.goal_position[0] - self.current_position[0]))
            if(self.IDEAL_HEADING < 0):
                self.IDEAL_HEADING = 2*np.pi + self.IDEAL_HEADING
        # print(IDEAL_HEADING)
        if self.dist_to_goal < 0.05:
            print("arrived and stopped")
            self.goal_reached = True
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = 0.0

        elif not self.circumnavigating and np.abs(self.heading - self.IDEAL_HEADING) > np.pi/30 and ((np.min(msg.ranges) > STUCK_THRESHOLD or self.count > 500)):
            # print("turning to head directly")
            # print("Ideal Heading: " + str(self.IDEAL_HEADING) + " Current Heading: " + str(self.heading))
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = ANGULAR_SPEED
            # if self.count > 300: self.count -= 100
            # if self.count <= 300: self.count = 0
            
        elif not self.circumnavigating and not self.goal_reached and (np.min(msg.ranges) > STUCK_THRESHOLD or self.count > 500):
            print("moving directly towards goal")
            # print(np.min(msg.ranges))
            self.move_the_bot.linear.x = LINEAR_SPEED
            self.move_the_bot.angular.z = 0.0
            if np.min(msg.ranges) > STUCK_THRESHOLD: self.count = 0

        elif not self.circumnavigating and np.min(msg.ranges) < STUCK_THRESHOLD and not self.goal_reached and not self.found_exit and self.count == 0:
            self.circumnavigating = True
            print(self.circumnavigating)
            self.start_circum_pt = self.current_position
            self.exit_circum_pt = self.start_circum_pt
            self.count = 0
            self.move_the_bot.linear.x = 0.0
            self.move_the_bot.angular.z = 0.0
            self.count += 1
    
        elif self.circumnavigating:
            print("curr_pos: " + str(self.current_position))
            print("exit: " + str(self.exit_circum_pt))
            print("found exit: " + str(self.found_exit))
            print("count: " + str(self.count))

            if self.found_exit and self.count > 300 and np.sqrt((self.current_position[0]-self.exit_circum_pt[0])**2 + (self.current_position[1]-self.exit_circum_pt[1])**2) < 0.1:
                self.circumnavigating = False
                self.found_exit = False
                self.move_the_bot.linear.x = 0.0
                self.move_the_bot.angular.z = ANGULAR_SPEED

            elif np.sqrt((self.current_position[0]-self.start_circum_pt[0])**2 + (self.current_position[1]-self.start_circum_pt[1])**2) < 0.1 and self.count > 300 and not self.found_exit:
                self.found_exit = True
                self.wall_follow(msg)
                self.count += 1

            else:
                self.wall_follow(msg)
                self.count += 1
                if(self.dist_to_goal < np.sqrt(  (self.exit_circum_pt[0] - self.goal_position[0])**2 + (self.exit_circum_pt[1] - self.goal_position[1])**2  )) or self.count == 1:
                    self.exit_circum_pt = self.current_position

        else:
            self.move_the_bot.linear.x = 0.01
            self.move_the_bot.angular.z = 0.01
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


