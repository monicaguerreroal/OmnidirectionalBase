#! /usr/bin/env python
import rospy
from std_msgs.msg import  Float64 


def talker():
    pub = rospy.Publisher('topic', Float64, queue_size=10)
    rospy.init_node('talker', anonymous = True)
    print "nodo creado con exito"
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello =rospy.get_time()
        pub.publish(hello)
        print hello
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
