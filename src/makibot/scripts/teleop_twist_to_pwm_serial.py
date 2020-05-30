#!/usr/bin/env python

import rospy
import serial
from geometry_msgs.msg import Twist
ser = serial.Serial(
    port='/dev/ttyUSB0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )
limit = 90
#m,dirL, spdL, dirR,  spdR, servo_pos, buzz_state
def sendCallback(msg):
    global ser
    
    spd = msg.linear.x;
    ang = msg.angular.z;

    pwm_l = 1160 * spd + (ang * 100)
    pwm_r = 1160 * spd - (ang * 100)

    pwm_l = abs(pwm_l)
    pwm_r = abs(pwm_r)
  
  
    if ang >= 0:
        pwm_l -= ang * 80
        pwm_r += ang * 80

    elif ang < 0:
        pwm_l += (ang * -1) * (80)
        pwm_r -= (ang * -1) * (80)
  
    if pwm_r > 255:
        pwm_r = 255
    if pwm_l > 255:
        pwm_l = 255
    if pwm_r < 0:
        pwm_r = 0;
    if pwm_l < 0:
        pwm_l = 0

    print("%f %d %d \n"%(spd, pwm_l, pwm_r))


    if spd >= 0:
        ser.write('m,%d,%d,%d,%d,%d,%d \n'%(0,pwm_l,0,pwm_r,0,0))

    elif spd < 0.0:
        ser.write('m,%d,%d,%d,%d,%d,%d \n'%(1,pwm_l,1,pwm_r,0,0))
 
    # if msg.linear.y > 0:
    #     ser.write('m,%d,%d,%d,%d,%d,%d \n'%(0,pwm_l,0,pwm_r,130,0))
    # elif msg.linear.y == 0:
    #     ser.write('m,%d,%d,%d,%d,%d,%d \n'%(0,pwm_l,0,pwm_r,90,0))
    

def main():
    rospy.init_node('twist_to_pwm')
    rospy.loginfo("twist_to_pwm started...")
    rospy.Subscriber("/cmd_vel", Twist, sendCallback) 
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
