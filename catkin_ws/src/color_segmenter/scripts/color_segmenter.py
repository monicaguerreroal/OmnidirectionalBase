#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

def mouse_callback(event,x,y,flags,param):
    global hsv
    if event == cv2.EVENT_LBUTTONDOWN:
            #cv2.circle(img,(x,y),100,(255,0,0),-1)
            print(hsv[y,x])
def callback_image(msg):
    global hsv
    global pub_ball, use_gui
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    bin = cv2.inRange(hsv,(70,180,150),(77,255,255))
    idx =  cv2.findNonZero(bin)
    [cx,cy,var1,var2] = cv2.mean(idx)
    msg_ball = Float32MultiArray()
    msg_ball.data = [cx,cy]
    if [cx,cy]!= [0,0]:
        pub_ball.publish(msg_ball)
    if use_gui:
        cv2.circle(img,(int(cx),int(cy)),100,(255,0,0),10)
        cv2.imshow("Binary", bin)
        cv2.imshow("Image",img)
        cv2.imshow("HSV",hsv)
        cv2.waitKey(10)
        
def main ():
    global pub_ball, use_gui
    use_gui= True
    print("initializing color segmenter")
    rospy.init_node("color_segmenter")
    if rospy.has_param("~gui"):
        use_gui = rospy.get_param("~gui")
    rospy.Subscriber("/camera/color/image_raw", Image, callback_image)
    pub_ball = rospy.Publisher("/ball_position",Float32MultiArray,queue_size=10)
    cv2.namedWindow('HSV')
    cv2.setMouseCallback('HSV',mouse_callback)
    rospy.spin()

if __name__== "__main__":
    main()
