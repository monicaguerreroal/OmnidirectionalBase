#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

def callback_ball_position(msg):
    global pub_cmd_vel,new_data
    print(msg.data)
    K = 2.0/320.0
    e = msg.data[0]-320
    cmd_vel = Twist()
    cmd_vel.angular.z = -K * e
    pub_cmd_vel.publish (cmd_vel)
    new_data = True
    

def main ():
    global pub_cmd_vel, new_data
    print("initializing ball tracker")
    rospy.init_node("ball_tracker")
    pub_cmd_vel  = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
    rospy.Subscriber("/ball_position",Float32MultiArray,callback_ball_position)
    new_data = False
    loop = rospy.Rate(50)
    counter = 0
    while not rospy.is_shutdown():
        if not new_data:
            counter+=1
            if counter>10:
                counter = 0
                pub_cmd_vel.publish(Twist())
        else:
            counter = 0
            new_data = False
        loop.sleep() 

if __name__== "__main__":
    main()
