#!/usr/bin/env python
import math
import rospy
import cv2
import ros_numpy
import numpy
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
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
    bin = cv2.inRange(hsv,(65,162,101),(83,207,255))
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

def callback_point_cloud(msg):
    global use_gui
    points = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    rgb_arr = points['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img = cv2.merge((numpy.asarray(b,dtype='uint8'),numpy.asarray(g,dtype='uint8'),numpy.asarray(r,dtype='uint8')))
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    bin = cv2.inRange(hsv,(65,162,101),(83,207,255))
    idx =  cv2.findNonZero(bin)
    [cx,cy,var1,var2] = cv2.mean(idx)
    [x,y,z] = [0,0,0]
    counter = 0
    if idx is None:
        return
    for [[c, r]] in idx:
        xt, yt, zt = points[r,c][0], points[r,c][1], points[r,c][2]
        if math.isnan(xt) or math.isnan(yt) or math.isnan(zt):
            continue
        [x,y,z,counter] = [x+xt, y+yt, z+zt, counter+1]
    [x,y,z] = [x/counter, y/counter, z/counter] if counter > 0 else [0,0,0]
    print[x,y,z]
    
    if use_gui:
        cv2.circle(img,(int(cx),int(cy)),100,(255,0,0),10)
        cv2.imshow("Binary", bin)
        cv2.imshow("Image",img)
        cv2.imshow("HSV",hsv)
        
def main ():
    global pub_ball, use_gui
    use_gui= False
    print("initializing color segmenter")
    rospy.init_node("color_segmenter")
    if rospy.has_param("~gui"):
        use_gui = rospy.get_param("~gui")
    #rospy.Subscriber("/camera/color/image_raw", Image, callback_image)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2,callback_point_cloud)
    pub_ball = rospy.Publisher("/ball_position",Float32MultiArray,queue_size=10)
    cv2.namedWindow('HSV')
    cv2.setMouseCallback('HSV',mouse_callback)
    rospy.spin()

if __name__== "__main__":
    main()
