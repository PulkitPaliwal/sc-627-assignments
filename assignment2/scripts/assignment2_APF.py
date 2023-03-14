#!/usr/bin/env python3

# import ros stuff
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math

# robot state variables
# state = 0 -- heading
# state = 1 -- move
# state = 2 -- goal
permitted_yaw_error = math.pi / 60
permitted_distance_error = 0.1
zeta = 0.05
eta = 0.05

class APF():
    def __init__(self) -> None:
        self.path = []
        position = Point()
        self.move_the_bot = Twist()
        self.yaw = 0
        self.state = 0
        self.current_position = np.array([position.x, position.y])
        self.goal_position = np.array([0.5, 0])
        obstacle1 = np.array([0,2])
        obstacle2 = np.array([1.5,0])
        obstacle3 = np.array([0,-2])
        obstacle4 = np.array([-1.5,0])
        self.obstacle = [obstacle1, obstacle2, obstacle3, obstacle4]
      
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.path.append((position.x, position.y))
        self.current_position = np.array([position.x, position.y])

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
    

    ###############################################################################
    ############################## basic functions ################################
    ###############################################################################

    def get_distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def angle_in_minpi_to_pi(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    ################################################################################
    ############################## force calculation ###############################
    ################################################################################

    def F_att(self):
        distance_to_goal = self.get_distance(self.current_position, self.goal_position)
        if distance_to_goal < 1:
            return zeta*(self.goal_position - self.current_position)
        else:
            return zeta*(self.goal_position - self.current_position)/distance_to_goal

    def F_rep(self):
        f_rep = []
        for i in range(len(self.obstacle)):
            distance_to_obstacle = self.get_distance(self.current_position, self.obstacle[i])
            q = 2
            if distance_to_obstacle < q:
                f_rep.append(eta*((1/q)-(1/distance_to_obstacle))*((1/distance_to_obstacle)**3)*(self.obstacle[i] - self.current_position))
            else:
                f_rep.append(0)
        return f_rep

    ################################################################################
    ################################################################################
    ################################################################################

    def change_state(self, new_state):
        self.state = new_state
        print('State changed to [%s]' % new_state)

    def heading(self):
        f =  self.F_rep()
        F_rep_total = f[0]

        for i in range(len(f)):
            F_rep_total = F_rep_total + f[i]
        F_rep_total = F_rep_total- f[0]
        F_total = self.F_att() + F_rep_total
        desired_yaw = math.atan2(F_total[1], F_total[0])
        err_yaw = self.angle_in_minpi_to_pi(desired_yaw - self.yaw)
        rospy.loginfo(err_yaw)
        if math.fabs(err_yaw) > permitted_yaw_error:
            self.move_the_bot.angular.z = 0.233 if err_yaw > 0 else -0.233
        
        self.pub.publish(self.move_the_bot)
        
        # if yaw set, move straight
        if math.fabs(err_yaw) <= permitted_yaw_error:
            print('Yaw error: [%s]' % err_yaw)
            self.change_state(1)

    def move(self):
        global yaw, pub, permitted_yaw_error, state
        
        f =  self.F_rep()
        F_rep_total = f[0]

        for i in range(len(f)):
            F_rep_total = F_rep_total + f[i]
        F_rep_total = F_rep_total- f[0]
        F_total = self.F_att() + F_rep_total
        desired_yaw = math.atan2(F_total[1], F_total[0])
        err_yaw = desired_yaw - self.yaw
        distance_to_goal = self.get_distance(self.current_position, self.goal_position)
        self.move_the_bot.linear.x = min(self.get_distance(F_total, [0,0]), 0.3)
        self.move_the_bot.angular.z = 0.033 if err_yaw > 0 else -0.033
        self.pub.publish(self.move_the_bot)
        

        # if reached goal, set state to "2"
        if math.fabs(distance_to_goal) < permitted_distance_error:
            self.change_state(2)

        # if yaw not set, set yaw
        if math.fabs(err_yaw) > permitted_yaw_error:
            # print ('Yaw error: [%s]' % err_yaw)
            self.change_state(0)

    def reached_goal(self):
        self.move_the_bot = Twist()
        self.move_the_bot.linear.x = 0
        self.move_the_bot.angular.z = 0
        print('Arrived')
        self.pub.publish(self.move_the_bot)

    def main(self):
        rospy.init_node('APF')
        # This node publishes to the /cmd_vel topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # This node subscribes to the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #Service Declaration 
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # print(F_att(), F_rep(obstacle),current_position)
            if self.state == 0:
                self.heading()
            elif self.state == 1:
                self.move()
            elif self.state== 2:
                self.reached_goal()
                print(self.path)
                with open("odom_data.txt", "w") as f:
                    f.write(str(self.path))
                exit()
            rate.sleep()
            rospy.spin()
APF_bot = APF()
APF_bot.main()
