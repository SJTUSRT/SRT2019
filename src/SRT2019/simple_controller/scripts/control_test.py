#!/usr/bin/env python

import rospy
from simple_controller.msg import Srt_Control

def main():
    rospy.init_node("controltest", anonymous = True)
    pub = rospy.Publisher('/srt_control', Srt_Control, queue_size=10)
    test_angle = [1.0, 2.0, 3.0, 4.0, 5.0]   # test angle set
    angle = 0.0
    while not rospy.is_shutdown():
        for i in range(-30,30,1):
            angle = -i
            rate = rospy.Rate(1)
            test_order = Srt_Control(0, angle)
            pub.publish(test_order)
            rate.sleep()

if __name__ == "__main__":
    main()
