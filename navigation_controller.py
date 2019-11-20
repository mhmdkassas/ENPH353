#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

# bridge = CvBridge()


class NavigationController():

    def __init__(self):
        self.vel_pub = rospy.Publisher('/R1/cmd_vel', Twist,
                                       queue_size=10)
        self.img_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image,
                                        self.img_callback, queue_size=10)
        self.bridge = CvBridge()
        self.cv_image = Image
        self.speed = 3
        self.turn_rate = 1
        self.vel_msg = Twist()
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.img_dim = None
        self.img_height, self.img_width = None, None
        self.edge_height = None
        self.edge_val = [0, 0]

    def img_callback(self, data):
        if data is not None:
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.img_dim = self.cv_image.shape
                self.img_height, self.img_width = self.img_dim[0], self.img_dim[1]
                self.edge_height = self.img_height - 100
            except CvBridgeError as e:
                print(e)

    def forward(self):
        # msg = Twist()
        self.vel_msg.linear.x = self.speed
        self.vel_msg.angular.z = 0
        self.vel_pub.publish(self.vel_msg)

    def turn(self):
        # msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = self.turn_rate
        self.vel_pub.publish(self.vel_msg)

    def stop(self):
        # msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_pub.publish(self.vel_msg)

    def hsv_filter(self, color):
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        hsv_color_dict = {
                "white": ([0, 0, 0], [0, 0, 255]),
                "red": ([161, 155, 84], [179, 255, 255]),
                "green": ([25, 52, 72], [102, 255, 255]),
                "blue": ([110, 50, 50], [130, 255, 255]),
            }
        limits = hsv_color_dict[color]
        lower = np.asarray(limits[0])
        upper = np.asarray(limits[1])
        mask = cv2.inRange(hsv, lower, upper)
        return mask

    def edge_detector(self):
        hsv_img = self.hsv_filter("white")
        retVal, binary = cv2.threshold(hsv_img, 64, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(binary, 100, 200)
        # cv2.imshow('testy', edges)
        # cv2.waitKey(0)
        pixel_array = np.asarray(edges)
        count = 0
        for i in range(0, self.img_width - 1):
            if(pixel_array.item(self.edge_height, i) != 0):
                if(count == 0):
                    self.edge_val[0] = i
                else:
                    self.edge_val[1] = i
            count = 1

    def img_test(self):
        right_blob = cv2.circle(self.cv_image, (self.edge_val[0],
                                                self.edge_height), 15,
                                (255, 0, 0), -1)
        left_blob = cv2.circle(right_blob, (self.edge_val[1],
                                            self.edge_height), 15,
                               (0, 0, 255), -1)
        cv2.imshow('blob', left_blob)
        cv2.waitKey(0)


rospy.init_node('navigation', anonymous=True)
rate = rospy.Rate(10)
robot = NavigationController()
forward_time = 1.25
turn_time = 1.125
test_time = 5.0
left_intersection = False
while not rospy.is_shutdown():
    if(left_intersection is False):
        rospy.sleep(rospy.Duration(test_time))
        robot.forward()
        print("forward")
        rospy.sleep(rospy.Duration(forward_time))
        robot.turn()
        print("turn")
        rospy.sleep((rospy.Duration(turn_time)))
        left_intersection = True
    else:
        robot.edge_detector()
        robot.stop()
        print("stop")
        robot.img_test()
