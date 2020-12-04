#!/usr/bin/env python

import rospy
from simple_controller.msg import Srt_Control

def main():
    rospy.init_node("controltest", anonymous = True)
    pub = rospy.Publisher('/srt_control', Srt_Control, queue_size=10)
    speed = 0.5
    rate = rospy.Rate(0.5)
    for i in range(0, 5):
        pub.publish(Srt_Control(speed, 0.0))
        rate.sleep()
    while not rospy.is_shutdown():
        pub.publish(Srt_Control(0.0, 0.0))
        rate.sleep()

if __name__ == "__main__":
    main()
