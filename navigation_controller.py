#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# import time

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
        self.edge_val = [0]
        self.pos = 0
        self.white_val = []
        #self.last_edge = 0

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
                "white": ([0, 0, 168], [172, 111, 255]),
                "red": ([161, 155, 84], [179, 255, 255]),
                "green": ([25, 52, 72], [102, 255, 255]),
                "blue": ([110, 50, 50], [130, 255, 255]),
            }
        limits = hsv_color_dict[color]
        lower = np.asarray(limits[0])
        upper = np.asarray(limits[1])
        mask = cv2.inRange(hsv, lower, upper)
        return mask

    def pos_finder(self):
        self.white_val = []
        start = 0
        hsv_img = self.hsv_filter("white")
        retVal, binary = cv2.threshold(hsv_img, 64, 255, cv2.THRESH_BINARY)
        pixel_arr = np.asarray(binary)
        cv2.imshow("binary", binary)
        cv2.waitKey(0)
        pos = 0
        for i in range(0, self.img_width -1):
            if(pixel_arr.item(self.edge_height, i) != 0):
                self.white_val.append(i)
        print("white_val: {}" .format(self.white_val))
        pos = max(self.white_val)  
        cv2.circle(self.cv_image, (pos, self.edge_height), 5, (0,0,255), -1)
        return pos

    def edge_detector(self):
        hsv_img = self.hsv_filter("white")
        retVal, binary = cv2.threshold(hsv_img, 64, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(binary, 100, 200)
        self.edge_val = [0]
        # cv2.imshow('testy', edges)
        # cv2.waitKey(0)
        pixel_array = np.asarray(edges)
        for i in range(0, self.img_width - 1):
            if(pixel_array.item(self.edge_height, i) != 0):
                self.edge_val.append(i)
        num_edges = len(self.edge_val)
        #print("number edges: {}" .format(num_edges))

    def path_follower(self):
        self.speed = 1.0
        tolerance = 10
        dead_zone = 1000
        center = float(self.img_width)/2.0
        quart = float(self.img_width)/4.0
        position = self.pos_finder()
        print(position)
        self.vel_msg.linear.x = self.speed
        self.vel_msg.angular.z = 0

        if(center > position):
            print("EDGE")
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 1

        if(center < position - tolerance):
            print("L")
            self.vel_msg.linear.x = 0.5
            self.vel_msg.angular.z = -4

        if (center <= position and center >= position - tolerance):
            print("F")
            self.vel_msg.linear.x = self.speed
            self.vel_msg.angular.z = 0

        if(1200 > position > dead_zone):
            print("yikes")
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = -0.3

        self.vel_pub.publish(self.vel_msg)

    def img_test(self):
        # edge_sum = self.edge_val[1] + self.edge_val[0]
        # edge_diff = self.edge_val[1] - self.edge_val[0]
        hsv_img = self.hsv_filter("white")
        retVal, binary = cv2.threshold(hsv_img, 64, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(binary, 100, 200)
        right_index = max(self.edge_val)
        for i in self.edge_val:
            cv2.circle(self.cv_image, (i,
                                       self.edge_height), 5,
                                      (255, 0, 0), -1)
        cv2.imshow('blob', binary)
        cv2.waitKey(3)
        #print("image width is {}" .format(self.img_width))
        #print("Right edge is {}" .format(right_index))
        #print("number of edges is {}" .format(len(self.edge_val)))


rospy.init_node('navigation', anonymous=True)
rate = rospy.Rate(10)
robot = NavigationController()
forward_time = 1.25
turn_time = 1.125
test_time = 3.0
left_intersection = False
robot.stop()
#truncated = 0
#last_updated = 0
while not rospy.is_shutdown():
    if(left_intersection is False):
        rospy.sleep(rospy.Duration(test_time))
        robot.forward()
        print("forward")
        rospy.sleep(rospy.Duration(forward_time))
        robot.turn()
        print("turn")
        rospy.sleep((rospy.Duration(turn_time)))
        #robot.edge_detector()
        # last_edge = max(robot.edge_val)
        # print("last edge is: {}" .format(last_edge))
        left_intersection = True
        robot.stop()
        print("stop")
    else:
        
        # print(robot.pos_finder())
        robot.img_test()
        robot.path_follower()