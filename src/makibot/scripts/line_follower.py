#!/usr/bin/env python

# Real-time Color Segmentation 
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

image_pub = rospy.Publisher('/line_detect_result', Image, queue_size=10)
twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

MAKIBOT_MAX_LIN_VEL = 0.22
MAKIBOT_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def imageCallback(msg):
    twist = Twist()
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    bridge = CvBridge()
    spd = 0.0
    ang = 0.0
    try:
      cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
      print(e)

    frame = cv_image
    x, y, c = frame.shape

    # Crop the image
    crop_img = frame[int(0.3*x):x, 0:y]
    # Convert to grayscale
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    # Gaussian blur
    blur = cv2.GaussianBlur(gray,(5,5),0)
    # Color thresholding
    ret,thresh = cv2.threshold(blur,90,255,cv2.THRESH_BINARY_INV)
    # Find the contours of the frame
    _ ,contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
    # Find the biggest contour (if detected)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.line(crop_img,(cx,0),(cx,x),(255,0,0),1)
        cv2.line(crop_img,(0,cy),(y,cy),(255,0,0),1)
        cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)
        if cx >= (0.8*y):
            print("Turn Right!")
            spd = 0.1
            ang = 2.0
        if cx < (0.8*y) and cx > (0.3*y):
            print("On Track!")
            spd = 0.22
            ang = 0
        if cx <= (0.3*y):
            print("Turn Left")
            spd = 0.1
            ang = -2.0
    else:
        print("I don't see the line")
        spd = 0.0
        ang = 0.0
    #Display the resulting frame
    control_linear_vel = spd
    control_angular_vel = ang
    # control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0

    # control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    twist_pub.publish(twist)
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main():
    rospy.init_node('line_follower', anonymous=True)
    rospy.loginfo("line_follower started...")
    f = rospy.Subscriber("/usb_cam/image_raw", Image, imageCallback) #Subscriber(node subscriber, msg_type, func_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

