#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

hsv_color_dict = {
    "white": ([0,0,0], [0,0,255]),
    "red": ([161,155,84], [179,255,255]),
    "green": ([25, 52, 72], [102, 255, 255]),
    "blue": ([110, 50, 50], [130, 255, 255]),
}

def forward(publisher, msg, speed):
    msg.linear.x = speed
    msg.angular.z = 0
    publisher.publish(msg)

def turn(publisher, msg, turn_rate):
    msg.linear.x = 0
    msg.angular.z = turn_rate
    publisher.publish(msg)

def stop(publisher, msg):
    msg.linear.x = 0
    msg.angular.z = 0
    publisher.publish(msg)

def hsv_filter(or_img, color):
    hsv = cv2.cvtColor(or_img, cv2.COLOR_BGR2HSV)
    limits = hsv_color_dict[color]
    lower = limits[0]
    upper = limits[1]
    mask = cv2.inRange(hsv, lower, upper)
    return mask


bridge = CvBridge()
edge_val = [0,0]
dims = [0,0]

def callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # cv2.circle(cv_image, (edge_val[0], height-3), 15, (0, 0, 255), -1)
    # cv2.circle(cv_image, (edge_val[1], height-3), 15, (0, 255, 0), -1)
    # cv2.imshow('blob', cv_image)
    cv2.waitKey(3)

sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback, queue_size = 10)
pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size = 10)
rospy.init_node('navigation', anonymous= True)

vel_msg = Twist()
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0

rate = rospy.Rate(10)

speed = 3
turn_rate = 1
forward_time = 3
turn_time = 3
start_time = time.time() 
while not rospy.is_shutdown():
    if(time.time() < start_time + forward_time):
        forward(pub, vel_msg, speed)
    elif(time.time() < start_time + forward_time + turn_time):
        turn(pub, vel_msg, turn_rate)
    else: 
        stop(pub, vel_msg)