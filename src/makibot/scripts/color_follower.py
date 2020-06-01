#!/usr/bin/env python

# Real-time Color Segmentation 
import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import Float32

cap = cv2.VideoCapture(0)
offset = 0
time.sleep(1)
prev_offset = 0
p_w = 0

def segment():
    global offset
    global prev_offset
    pub = rospy.Publisher('center_offset', Float32, queue_size=10)
    rospy.init_node('color_segmentation', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        _, frame = cap.read()
        prev_offset = offset
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_yellow = np.array([10,130,100])
        upper_yellow = np.array([25,255,255])


        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)#(src1,src2,  )
        
        ret,thrshed = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
        image, contours, hierarchy = cv2.findContours(thrshed,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x,y,w,h = cv2.boundingRect(cnt)
                frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.circle(frame,(int(x+(w/2)),int(y+(h/2))), 10, (0,0,255), -1)
                offset = int( (frame.shape[1]/2) - (x+(w/2)))
        if len(contours) < 2:
            offset = 0

        cv2.line(frame,(int(frame.shape[1]/2),0),(int(frame.shape[1]/2),511),(255,0,0),1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame,str(offset) + "px",(10,400), font, 1 ,(255,0,255),1,cv2.LINE_AA)
        
        #cv2.imshow('frame',frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

        if prev_offset != offset:
            rospy.loginfo(str(offset))
            pub.publish(offset)
        rate.sleep()

    cv2.destroyAllWindows()
        

if __name__ == '__main__':
    try:
        segment()
    except rospy.ROSInterruptException:
        pass
