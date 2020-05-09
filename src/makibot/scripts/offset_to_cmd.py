#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import String

pub = rospy.Publisher('/cmd', String, queue_size=1000)

s = ""
limit = 90
def offsetCallback(msg):
    global s
    if msg.data < limit and msg.data > -limit and msg.data != 0:
        s = "F"
        rospy.loginfo(s)
        pub.publish(s)
    elif msg.data >= limit :
        s = "G"
        rospy.loginfo(s)
        pub.publish(s)
    elif msg.data <= -limit:
        s = "I"
        rospy.loginfo(s)
        pub.publish(s)
    elif msg.data == 0:
        s = "S"
        rospy.loginfo(s)
        pub.publish(s)

def main():
    rospy.init_node('offset_to_command')
    rospy.loginfo("cmd started...")
    rospy.Subscriber("/angle_of_rotation", Float32, offsetCallback) #Subscriber(node subscriber, msg_type, func_callback)


    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
