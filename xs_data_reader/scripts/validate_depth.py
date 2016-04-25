#!/usr/bin/python
import rospy
import os
import sys
import cv2
import numpy as np

from plane_extraction import estimate_plane_mask

class ValidateDepth():
    def __init__(self, data_path):
        self._data_path = data_path+"/"
        self._depth_images = []
        self.parseFolder()
        height, width = cv2.imread(self._depth_images[0],2).shape
        self._mask_top_left = [0,0]
        self._mask_bottom_right = [height,width]


    def parseFolder(self):
        print 'Looking for depth images in ', self._data_path
        file_found = True
        file_index = 0
        while file_found:
            string_index = str(file_index).zfill(4)
            file_name = self._data_path + "depth_1_" + string_index + '.png';
            if os.path.isfile(file_name) :
                # look for label images
                # self._file_label_dict[file_name] = []
                # label_index = 0
                # string_label_index = str(label_index).zfill(4)
                # label_file = self._data_path + "rgb_1_" + string_index + '_label_';

                self._depth_images.append(file_name)
                file_index+=1
            else :
                file_found = False
        print 'Found ',len(self._depth_images),' depth images'
        return


if __name__ == '__main__':
    if len(sys.argv) == 2:
        path = sys.argv[1]
    else :
        print "Correct usage: python validate_depth.py <data_path>"
        sys.exit()

    cv2.namedWindow('input', cv2.WINDOW_OPENGL)
    cv2.namedWindow('mask', cv2.WINDOW_OPENGL)
    validation = ValidateDepth(path)
    for file_name in validation._depth_images:
        depth_image = cv2.imread(file_name,2)
        depth_image*=10
        cropped = np.float32(depth_image[validation._mask_top_left[0]:validation._mask_bottom_right[0], validation._mask_top_left[1]:validation._mask_bottom_right[1]])/1000
        # flowmask = 255*np.uint8(depth_image[validation._mask_top_left[0]:validation._mask_bottom_right[0], validation._mask_top_left[1]:validation._mask_bottom_right[1]])
        flowmask = 255*np.ones((validation._mask_bottom_right[0], validation._mask_bottom_right[1]), np.uint8)
        plane = 255*estimate_plane_mask(cropped, flowmask)
        cv2.imshow('input',depth_image)
        cv2.imshow('mask',plane)
        cv2.waitKey(0)
