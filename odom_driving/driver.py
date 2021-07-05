#!/usr/bin/env python
import rospy
import math
import copy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

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


def talker():
    global odom, start_pos, distance_to_run
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/diff_drive/odom", Odometry, callback)
    rospy.Subscriber("/joint_states/", JointState, callbackJoints)

    pub = rospy.Publisher('/diff_drive/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        # info = "Distance %s" % abs(start_pos.pose.pose.position.x - odom.pose.pose.position.x)
        x = abs(start_pos.pose.pose.position.x - odom.pose.pose.position.x)
        y = abs(start_pos.pose.pose.position.y - odom.pose.pose.position.y)
        len = math.sqrt((x*x)+(y*y))
        if not len > distance_to_run:
            cmd_vel.linear.x = 0.5
        elif joint_states.velocity[0] != 0.0 or joint_states.velocity[1] != 0.0:
            cmd_vel.linear.x = 0.0
        else:
            print("Reached distance", distance_to_run)
            print(start_pos.pose.pose.position.x - odom.pose.pose.position.x,
                start_pos.pose.pose.position.y - odom.pose.pose.position.y, len, 
                math.degrees(start_pos.pose.pose.orientation.z - odom.pose.pose.orientation.z))
            print(odom.pose.pose.position.x, odom.pose.pose.position.y)
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
