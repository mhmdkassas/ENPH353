#!/usr/bin/env python
# from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
from os import listdir
from os import walk
import random

def hsv_filter(img, color):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_color_dict = {
        "white": ([0, 0, 168], [172, 111, 255]),
        "red": ([161, 155, 84], [179, 255, 255]),
        "green": ([25, 52, 72], [102, 255, 255]),
        "blue": ([110, 50, 50], [130, 255, 255]),
        "gray": ([0, 0, 90], [0, 0, 150])
        }
    limits = hsv_color_dict[color]
    lower = np.asarray(limits[0])
    upper = np.asarray(limits[1])
    mask = cv2.inRange(hsv, lower, upper)
    return mask

src_path = "/home/rehan/353_ws/src/robot_controller/src/scripts/sim_photos/OPQR"

files = []
# src = os.listdir(src_path)
# for r, d, f in os.walk('./'):
for (dirpath, dirnames, filenames) in walk(src_path):
    files.extend(filenames)
    break
# for file in src:
    # if '.png' in file and 'cropped' in file: 
        # files.append(file)

print(filenames)
files.sort()
imgs = []
img_refined = []
test_counter = 1
for f in filenames:
    img = cv2.imread(src_path + "/{}" .format(f))
    # cv2.imshow('bap',img)
    # cv2.waitKey(0)
    dims = img.shape
    h, w = dims[0], dims[1]
    # print("height {}" .format(h))
    # print("wdith {}" .format(w))
    blue = hsv_filter(img, 'blue')
    retVal, binary = cv2.threshold(blue, 64, 255, cv2.THRESH_BINARY)

    # cv2.imshow('ahh', binary)
    # cv2.waitKey(0)
    # edges = cv2.Canny(binary, 100, 200)
    pix_arr = np.asarray(binary)
    pix_arr = pix_arr/255
    prev = 0
    count = 0
    edge_count = 0
    count = 0
    l = []
    for i in range(1, w):
        if(pix_arr[int(float(h)/2.0)][i] != pix_arr[int(float(h)/2.0)][i-1]):
            count += 1
            if(pix_arr[int(float(h)/2.0)][i] == 0):
                l.append(i)
            else:
                l.append(i - 1)

    # print(l)
    print(test_counter)
    print(filenames[test_counter - 1])
    test_counter += 1
    # print(count)
    if(count == 3):
        cropped = img[:, l[1]:l[2]]
    if(count == 2):
        cropped = img[:, l[0]: l[1]]
    if(count == 1):
        cropped = img[:, l[0]:]
    # cv2.imshow("crp", cropped)
    # cv2.imshow("bin", binary)
    cropped = cropped[int(float(h)*0.5): h, :]
    # cv2.imshow('ugh', cropped)
    # cv2.waitKey(0)
    lower_gray = np.uint8([[[128, 128, 128]]])
    upper_gray = np.uint8([[[192, 192, 192]]])
    l_h = cv2.cvtColor(lower_gray,cv2.COLOR_BGR2HSV)
    u_h = cv2.cvtColor(upper_gray,cv2.COLOR_BGR2HSV)
    # print(l_h)
    # print(u_h)
    mask = hsv_filter(cropped, 'blue')
    arr = np.asarray(mask)/255
    area = len(arr[0])*len(arr)
    mag = float(np.sum(arr))/len(arr)
    if(mag < 10):
        img_refined.append(cropped)

img_cropped = []
for im in img_refined:
    mask = hsv_filter(im, "blue")
    edges = cv2.Canny(mask, 100, 200)
    arr_edges = np.asarray(edges)/255
    # print(255 in arr_edges)
    i = 0
    upper_lim = 0
    lower_lim = 0
    while i < mask.shape[0]:
        if(1 in arr_edges[i]):
            lower_lim = i
            i = mask.shape[0]
        i+=1
    i = mask.shape[0] - 1
    while i > 0:
        if(1 in arr_edges[i]):
            upper_lim = i
            i = 0
        i-=1
    # print(lower_lim)
#     # print(upper_lim)
#     tolerance = 2
#     final_crop = im[lower_lim - tolerance:upper_lim+tolerance, :]
#     # print("height: {}, width: {}" .format(final_crop.shape[0], final_crop.shape[1]))
#     img_cropped.append(final_crop)
#     cv2.imshow("a", im[lower_lim - tolerance:upper_lim+tolerance, :])
#     cv2.waitKey(0)


# resized_lps = []
# for plate in img_cropped:
#     new_dim = (200, 100) # width, height
#     resize = cv2.resize(plate, new_dim)
#     resized_lps.append(resize)
#     cv2.imshow('plate', resize)
#     cv2.waitKey(0)

# cv2.imshow('p1', resized_lps[0])
# cv2.waitKey(0)

# test_crop1 = resized_lps[0][:, 15:50]
# test_crop2 = resized_lps[0][:, 50:85]
# test_crop3 = resized_lps[0][:, 115:150]
# test_crop4 = resized_lps[0][:, 150:185]
# cv2.imshow('q', test_crop1)
# cv2.waitKey(0)
path = os.path.dirname(os.path.realpath(__file__)) + "/"

# letter_lp = []
# for rlp in resized_lps:
#     test_crop1 = rlp[:, 15:50]
#     cv2.imwrite(os.path.join(path + "sim_photos/OPQR/O/", "O{}.png".format(random.randint(0,99))), test_crop1)
#     test_crop2 = rlp[:, 50:85]
#     cv2.imwrite(os.path.join(path + "sim_photos/OPQR/P/", "P{}.png".format(random.randint(0,99))), test_crop2)
#     test_crop3 = rlp[:, 115:150]
#     cv2.imwrite(os.path.join(path + "sim_photos/OPQR/Q/", "Q{}.png".format(random.randint(0,99))), test_crop3)
#     test_crop4 = rlp[:, 150:185]
#     cv2.imwrite(os.path.join(path + "sim_photos/OPQR/R/", "R{}.png".format(random.randint(0,99))), test_crop4)
#     crop_array = [test_crop1, test_crop2, test_crop3, test_crop4]
#     for i in crop_array:
#         letter_lp.append(i)
#         # cv2.imwrite(os.path.join(path + "sim_photos/AB01/", "letter{}{}.png".format(1, random.randint(0,99))), i)

# filtered_letters = []
# for let in letter_lp:
    # ret, binary = cv2.threshold(let, 100, 150, cv2.THRESH_BINARY)
    # edgy = cv2.Canny(binary, 100, 200)
    blued = hsv_filter(blur, "blue")
    # cv2.imshow('bluh', edgy)
    # cv2.waitKey(0)
 