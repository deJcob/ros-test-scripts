#!/usr/bin/env python
from xmlrpc.client import Boolean
import rospy
import copy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

odom = Odometry()
start_pos = Odometry()
inited = Boolean(False)
inited = False
distance_to_run = 0.1


def callback(data):
    global inited, odom, start_pos
    if not inited:
        start_pos = copy.copy(data)
        inited = True
    odom = data 


def talker():
    global odom, start_pos, distance_to_run
    rospy.init_node('lab_node', anonymous=True)

    rospy.Subscriber("/diff_drive/odom", Odometry, callback)

    pub = rospy.Publisher('/diff_drive/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0

        x = abs(start_pos.pose.pose.position.x - odom.pose.pose.position.x)
        y = abs(start_pos.pose.pose.position.y - odom.pose.pose.position.y)
        distance = math.sqrt(x*x+y*y)
        if not distance > distance_to_run:
            cmd_vel.linear.x = 0.5
        else:
            cmd_vel.linear.x = -0.1
            print("Reached distance ", distance_to_run)
            print(x)
            print(y)
            print(distance)
            pub.publish(cmd_vel)
            cmd_vel.linear.x = 0.0
            pub.publish(cmd_vel)
            exit()
        pub.publish(cmd_vel)
        # print(info)
        rate.sleep()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
