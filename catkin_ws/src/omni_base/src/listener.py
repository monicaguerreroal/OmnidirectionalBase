#!/usr/bin/env python

import rospy
from  std_msgs.msg import Float64

def callback(data):
    print   data.data

def listener ():
    rospy.init_node('talker2', anonymous = True)
    rospy.Subscriber("topic2", Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
