#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()
edge_val = [0,0]
dims = [0,0]
def callback(data):
    global dims
    count = 0
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    dim = cv_image.shape
    height, width = dim[0], dim[1]
    dims = dim
    blur = cv2.GaussianBlur(cv_image, (25, 25), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    
    retVal, binary = cv2.threshold(gray, 64, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(binary, 100, 200)
    pixel_array = np.asarray(edges)
    for i in range(0, width - 1):
        if(pixel_array.item(height - 10, i) != 0):
            if(count == 0):
                edge_val[0] = i
            else:
                edge_val[1] = i
            count = 1
    #print("{}, {}".format(edge_val[0], edge_val[1]))
    cv2.circle(cv_image, (edge_val[0], height-3), 15, (0, 0, 255), -1)
    cv2.circle(cv_image, (edge_val[1], height-3), 15, (0, 255, 0), -1)
    cv2.imshow('blob', cv_image)
    cv2.waitKey(3)


sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, callback, queue_size = 10)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
rospy.init_node('vel_adjuster', anonymous=True)

rate = rospy.Rate(10)

vel_msg = Twist()


vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0

while not rospy.is_shutdown():
    pos = (edge_val[0] + edge_val[1])/2.0
    h = dims[0]
    w = dims[1]
    print(dims)
    center = float(w)/2.0
    quart = float(w)/4.0

    speed = 0.1
    dir1 = 3
    dir2 = 6

    vel_msg.linear.x = speed
    vel_msg.angular.z = 0

    print("{}, {}, {}".format(center-quart, pos, center+quart))
    if(pos < (center - quart)):
        print("L")
        vel_msg.linear.x = 0
        vel_msg.angular.z = -1#-1*dir2
    # elif(pos < center):
    #     vel_msg.linear.x = 00
    #     vel_msg.angular.z = -1*dir1
    
    if(pos > (center + quart)):
        print("R")
        vel_msg.linear.x = 0
        vel_msg.angular.z = 1#dir2
    # elif(pos > center):
    #     vel_msg.linear.x = 0
    #     vel_msg.angular.z = dir1

    if ((pos >= (center - quart)) and (pos <= (center + quart))):
        print("F")
        vel_msg.linear.x = speed
        vel_msg.angular.z = 0
    pub.publish(vel_msg)

    rate.sleep()

