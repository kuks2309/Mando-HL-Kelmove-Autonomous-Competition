#!/usr/bin/env python
'''
hack to publish some 0 odometry
'''

# Python
import sys
from math import *
import tf
import math
# ROS
import rospy
from nav_msgs.msg import Odometry


def clamp(val):
    while val < -pi/2.0:
        val += 2.0*pi
    while val > pi/2.0:
        val -= 2.0*pi
    return val

def talker():
    pub = rospy.Publisher('nav_odom', Odometry, queue_size=10)  # /odometry/nav
    rospy.init_node('odomtester', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    lat = 36.6137
    dlat = 0.0 #0.000001
    lon = -121.912
    dlon = 0.0 #0.000001

    roll = 0.0
    droll = 0.0 #0.01
    pitch = 0.0
    dpitch = 0.0 #0.01
    yaw = pi/4.0
    dyaw = 0.0 #0.03
    
    radius = 0.00002
    theta = 0

    N = 0.0
    while not rospy.is_shutdown():
        
        omsg = Odometry()
        omsg.header.seq += 1
        omsg.header.stamp = rospy.get_rostime()
        omsg.header.frame_id = 'frame'
        omsg.pose.pose.position.x = lon+N*dlon
        omsg.pose.pose.position.y = lat+N*dlat
        omsg.pose.pose.position.z = 0.0 #1.0

        r = clamp(roll+N*droll)
        p = clamp(pitch+N*dpitch)
        y = clamp(yaw+N*dyaw)
        q = tf.transformations.quaternion_from_euler(r,p,y)
        omsg.pose.pose.orientation.x = q[0]
        omsg.pose.pose.orientation.y = q[1]
        omsg.pose.pose.orientation.z = q[2]
        omsg.pose.pose.orientation.w = q[3]

        omsg.pose.covariance = range(36)

        omsg.twist.twist.linear.x=1.0
        omsg.twist.twist.linear.y=2.0
        omsg.twist.twist.linear.z=3.0
        omsg.twist.twist.angular.x=4.0
        omsg.twist.twist.angular.y=5.0
        omsg.twist.twist.angular.z=6.0

        omsg.twist.covariance = range(36,36+36)
        
        lat = 35.824800   + radius * cos(theta * math.pi/180)
        lon = 128.753000  + radius * sin(theta * math.pi/180)
        theta +=1
        theta = theta%360
        
        #print(theta)
        rospy.loginfo("publishing odom (%.10f, %.10f, %.2f)"%(omsg.pose.pose.position.y,omsg.pose.pose.position.x,omsg.pose.pose.position.z))
        pub.publish(omsg)
        
        N += 1.0
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
