#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

pub = rospy.Publisher('angle_of_rotation', Float32, queue_size=10)


def angleCallback(msg):
    z = (msg.pose.position.z) * (0.5/7)
    x = msg.pose.position.x
    radian = math.atan2(x,z)
    degree=math.degrees(radian)
    degree = -degree
    #print("theta: " + str(degree) + " x: " + str(x) + " z: " + str(z))
    pub.publish(degree)
def main():
    rospy.init_node('marker_follower', anonymous=True)
    rospy.loginfo("marker_follower...")
    f = rospy.Subscriber("/aruco_single/pose", PoseStamped, angleCallback) #Subscriber(node subscriber, msg_type, func_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

