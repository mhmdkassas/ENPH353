#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import random
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
        self.ped = False
        self.flag = True
        self.cropped_edge = []
        self.p_counter = 1
        self.seen_car = False
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
        # cv2.imshow("a", self.cv_image)
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        lower = np.asarray([110, 50, 50])
        upper = np.asarray([130, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        # cv2.imshow("mask", mask)
        retVal, binary = cv2.threshold(self.cv_image, 64, 250, cv2.THRESH_BINARY)
        # cv2.imshow("real", self.cv_image)
        # lower_bgr = np.uint8([[[41, 20, 14]]])
        # upper_bgr = np.uint8([[[158, 134, 107]]])
        # l = np.asarray([104, 82, 41])
        # u = np.asarray([113, 168, 158])
        # hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv, l, u)
        # gb = cv2.GaussianBlur(mask, (5,5), cv2.BORDER_DEFAULT)
        # r, b = cv2.threshold(gb, 64, 240, cv2.THRESH_BINARY)
        # h_center = int(float(b.shape[0])/2.0)
        # w_center = int(float(b.shape[1])/2.0)
        # cropped_img = b[h_center:, w_center:]
        # dims = cropped_img.shape
        # h, w = dims[0], dims[1]
        # crp = np.asarray(cropped_img)
        # count = 0
        # for i in range(0, h):
        #     for z in range(0, w):
        #         if(crp.item(i, z) != 0):
        #             # self.ped = True
        #             count = 1
        # if(count == 0):
        #     self.ped = False
        # print(self.ped)
        # cv2.imshow("b", cropped_img)
        # cv2.waitKey(3)
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
        # retVal, binary = cv2.threshold(self.cv_image, 64, 246, cv2.THRESH_BINARY)
        # cv2.imshow("a", self.cv_image)
        # cv2.waitKey(3)

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
        self.white_val = [0]
        start = 0
        hsv_img = self.hsv_filter("white")
        retVal, binary = cv2.threshold(hsv_img, 64, 255, cv2.THRESH_BINARY)
        pixel_arr = np.asarray(binary)
        pos = 0
        for i in range(0, self.img_width -1):
            if(pixel_arr.item(self.edge_height, i) != 0):
                self.white_val.append(i)
        pos = max(self.white_val)
        cv2.circle(self.cv_image, (pos, self.edge_height), 5, (0,0,255), -1)
        # cv2.imshow("real",self.cv_image)
        # cv2.waitKey(3)
        return pos

    def edge_detector(self):
        hsv_img = self.hsv_filter("white")
        retVal, binary = cv2.threshold(hsv_img, 64, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(binary, 100, 200)
        self.edge_val = [0]
        pixel_array = np.asarray(edges)
        for i in range(0, self.img_width - 1):
            if(pixel_array.item(self.edge_height, i) != 0):
                self.edge_val.append(i)
        # print("num of edges {}" .format(len(self.edge_val)))
    def ped_finder(self):
        center = int(float(self.img_width)/4.0)
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        lower = np.asarray([110, 50, 50])
        upper = np.asarray([130, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        # cv2.imshow("mask", mask)
        retVal, binary = cv2.threshold(self.cv_image, 64, 250, cv2.THRESH_BINARY)
        # cv2.imshow("real", self.cv_image)
        # lower_bgr = np.uint8([[[41, 20, 14]]])
        # upper_bgr = np.uint8([[[158, 134, 107]]])
        l = np.asarray([104, 82, 41])
        u = np.asarray([113, 168, 158])
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, l, u)
        gb = cv2.GaussianBlur(mask, (5,5), cv2.BORDER_DEFAULT)
        r, b = cv2.threshold(gb, 64, 240, cv2.THRESH_BINARY)
        cropped_img = b[:, center:]
        cropped_img_1 = self.cv_image[:, center:]
        dims = cropped_img.shape
        h, w = dims[0], dims[1]
        crp = np.asarray(cropped_img)
        count = 0
        for i in range(0, h):
            for z in range(0, w):
                if(crp.item(i, z) != 0):
                    self.ped = True
                    count = 1
        if(count == 0):
            self.ped = False
        print(self.ped)
        # cv2.imshow("b", cropped_img)
        # cv2.imshow("r", cropped_img_1)
        # cv2.waitKey(3)

    def path_follower(self):
        # self.ped_finder()
        self.speed = 1.0
        tolerance = 10
        dead_zone = 1000
        center = float(self.img_width)/2.0
        position = self.pos_finder()
        self.vel_msg.linear.x = self.speed
        self.vel_msg.angular.z = 0

        if(center > position):
            # print("edge")
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 1

        if(center < position - tolerance):
            # print("left")
            self.vel_msg.linear.x = 0.5
            self.vel_msg.angular.z = -4

        if (center <= position and center >= position - tolerance):
            # print("f")
            self.vel_msg.linear.x = self.speed
            self.vel_msg.angular.z = 0

        if(1200 > position > dead_zone):
            # print("yikes")
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = -0.3

        self.vel_pub.publish(self.vel_msg)

    def img_test(self):
        # seen_car = False
        left_x_offset = -200
        bottom_y_offset = 175
        right_x_offset = -250
        top_y_offset = 325
        top_left = (int(float(self.img_height) / 2.0) + left_x_offset, 0 + top_y_offset)
        bottom_left = (int(float(self.img_height) / 2.0) + left_x_offset, int(float(self.img_width) / 4.0) + bottom_y_offset)
        top_right = (int(float(self.img_height) * 0.75) + right_x_offset, 0 + top_y_offset)
        bottom_right = (int(float(self.img_height) * 0.75) + right_x_offset, int(float(self.img_width) / 4.0) + bottom_y_offset)
        hsv_img = self.hsv_filter("blue")
        retVal, binary = cv2.threshold(hsv_img, 100, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(binary, 100, 200)
        cropped = edges[top_left[1]:bottom_left[1], top_left[0]:top_right[1]]
        dims = cropped.shape
        h, w = dims[0], dims[1]
        pixel_array = np.asarray(cropped)
        self.cropped_edge = []
        for i in range(0, w - 1):
            for z in range(0, h -1):
                if(pixel_array.item(z, i) != 0):
                    self.cropped_edge.append(i)
        print("cropped edges: {}" .format(len(self.cropped_edge)))
        if (len(self.cropped_edge) > 600):
            self.seen_car = True
            cv_crop = self.cv_image[top_left[1]:bottom_left[1], top_left[0]:top_right[1]]
            cv2.imwrite('cropped_car{}_{}.png' .format(self.p_counter, len(self.cropped_edge)), cv_crop)
            print("cropped saved!")
        # tl = cv2.circle(self.cv_image, top_left, 15, (0, 0, 255), -1) #red
        # tr = cv2.circle(tl, top_right, 15, (255, 0, 255), -1) #purple
        # bl = cv2.circle(tr, bottom_left, 15, (0, 255, 0), -1) #green
        # br = cv2.circle(bl, bottom_right, 15, (255, 255, 0), -1) #teal
        if (len(self.cropped_edge) < 100 and self.seen_car is True):
            self.seen_car = False
            self.p_counter += 1
        cv2.imshow("b", cropped)
        cv2.waitKey(3)

rospy.init_node('navigation', anonymous=True)
rate = rospy.Rate(10)
robot = NavigationController()
forward_time = 1.75
turn_time = 1.125
test_time = 3.0
left_intersection = False
robot.stop()
count = 0
while not rospy.is_shutdown():
    # if(count % 2 == 0):
    #     count = 0
    # else:
    #     count+=1
    # robot.pos_finder()
    if(left_intersection is False):
        rospy.sleep(rospy.Duration(test_time))
        robot.forward()
        rospy.sleep(rospy.Duration(forward_time))
        robot.turn()
        rospy.sleep((rospy.Duration(turn_time)))
        left_intersection = True
        robot.stop()
    else:
        # robot.img_test()
        robot.edge_detector()
        if(len(robot.edge_val) > 18):
            if(robot.flag):
                robot.stop()
                rospy.sleep((rospy.Duration(2)))
                robot.ped_finder()
                print(robot.ped)
            if(robot.ped):
                robot.stop()
            if(robot.ped is False):
                robot.path_follower()
                robot.flag = False
        else:
            robot.flag = True
            # if (len(robot.edge_val) == 5):
            #     robot.stop()
            #     print("edge=5")
            #     rospy.sleep((rospy.Duration(1)))
            robot.path_follower()
            robot.img_test()
