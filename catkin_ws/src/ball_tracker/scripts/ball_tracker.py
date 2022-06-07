#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

def callback_ball_position(msg):
    global pub_cmd_vel
    print(msg.data)
    K = 5.0/320.0
    e = msg.data[0]-320
    cmd_vel = Twist()
    cmd_vel.angular.z = -K * e
    pub_cmd_vel.publish (cmd_vel)

def main ():
    global pub_cmd_vel
    print("initializing ball tracker")
    rospy.init_node("ball_tracker")
    pub_cmd_vel  = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
    rospy.Subscriber("/ball_position",Float32MultiArray,callback_ball_position)
    
    
    rospy.spin()

if __name__== "__main__":
    main()
