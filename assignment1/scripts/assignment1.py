#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def laserdata_callback(msg):
    #Prints the length of ranges array, in our case there are total 360 readings, one reading for each 1 degree

    #print(len(msg.ranges)) 

    #To make use of laser data, we will need directional readings. For example, front, back, left and right of the robot. In our case, 0: Front, 180: Back, 90: Right, 270: Left,  are directions of laserbeam for robot

    #print(msg.ranges[0], msg.ranges[90], msg.ranges[180], msg.ranges[270])
    
    # sample head collision avoidance algorithm(You will have to write your algorithm here)

    if msg.ranges[0] > 0.2 :
        print('moving')
        move_the_bot.linear.x = 0.1
        move_the_bot.angular.z = 0.0
    else :
        print('stopped')
        move_the_bot.linear.x = 0.0
        move_the_bot.angular.z = 0.0

    publish_to_cmd_vel.publish(move_the_bot) 





if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')
    subscribe_to_laser = rospy.Subscriber('/scan', LaserScan, callback = laserdata_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    #create an object of Twist data

    move_the_bot = Twist()
    rospy.spin()


