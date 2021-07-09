#!/usr/bin/env python
# from xmlrpc.client import Boolean
import rospy
import copy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

odom = Odometry()
range0 = Range()
range1 = Range()
range2 = Range()
range3 = Range()
start_pos = Odometry()
inited = False
distance_to_run = 0.1
distance_to_stop = 0.15

odom = Odometry()
start_pos = Odometry()
joint_states = JointState()
inited = False
distance_to_run = 2.0


def callback(data):
    global inited, odom, start_pos
    if not inited:
        start_pos = copy.copy(data)
        print("Start!")
        print(start_pos.pose.pose.position.x, start_pos.pose.pose.position.y)
        inited = True
    odom = data 

def callbackJoints(data):
    global joint_states
    joint_states = data

def rulerCallback0(data):
    global range0
    range0 = data.range

def rulerCallback1(data):
    global range1
    range1 = data.range

def rulerCallback2(data):
    global range2
    range2 = data.range

def rulerCallback3(data):
    global range3
    range3 = data.range

def talker():
    global odom, start_pos, distance_to_run, range

    rospy.init_node('lab_node', anonymous=True)

    rospy.Subscriber("/robot_driver/laser_ruler/scan_0", Range, rulerCallback0)
    rospy.Subscriber("/robot_driver/laser_ruler/scan_1", Range, rulerCallback1)
    rospy.Subscriber("/robot_driver/laser_ruler/scan_2", Range, rulerCallback2)
    rospy.Subscriber("/robot_driver/laser_ruler/scan_3", Range, rulerCallback3)
    rospy.Subscriber("/diff_drive/odom", Odometry, callback)
    rospy.Subscriber("/joint_states/", JointState, callbackJoints)

    pub = rospy.Publisher('/diff_drive/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        print("0: ",range0, "1: ",range1, "2: ",range2, "3: ",range3)

        x = abs(start_pos.pose.pose.position.x - odom.pose.pose.position.x)
        y = abs(start_pos.pose.pose.position.y - odom.pose.pose.position.y)
        len = math.sqrt((x*x)+(y*y))

        if not range0 < distance_to_stop:
        # if not range1 < distance_to_stop:
        # if not range2 < distance_to_stop:
        # if not range3 < distance_to_stop:
            cmd_vel.linear.x = 0.4
        elif joint_states.velocity[0] != 0.0 or joint_states.velocity[1] != 0.0:
            cmd_vel.linear.x = 0.0
        else:
            print("x: ",start_pos.pose.pose.position.x - odom.pose.pose.position.x,
                "y: ", start_pos.pose.pose.position.y - odom.pose.pose.position.y,
                "len: ", len, 
                "z: ", math.degrees(start_pos.pose.pose.orientation.z - odom.pose.pose.orientation.z))
            print(odom.pose.pose.position.x, odom.pose.pose.position.y)
            # cmd_vel.linear.x = -0.1
            print("Reached distance 0:", range0)
            # print("Reached distance 1:", range1)
            # print("Reached distance 2:", range2)
            # print("Reached distance 3:", range3)
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