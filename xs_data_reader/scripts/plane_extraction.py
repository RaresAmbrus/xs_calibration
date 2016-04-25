#!/usr/bin/python
import rospy
import os
import sys
import numpy as np
import cv2

def estimate_plane_mask(D, flowmask):
    threshold = 0.1

    # D = np.float32(depth[top_left[0]:bottom_right[0], top_left[1]:bottom_right[1]])
    # flowmask = 255*np.uint8((depth[240:, 100:540] > 20000)*(depth[240:, 100:540] < 45000))

    height, width = D.shape

    nx, ny = (3, 2)
    x = np.arange(0, width)
    y = np.arange(0, height)
    X, Y = np.float32(np.meshgrid(x, y))

    p = np.zeros((4, 1), np.float32)
    count = 0
    for i in range(0, 500):
        xc = np.random.randint(0, width, (3, 1))
        yc = np.random.randint(0, height, (3, 1))
        dc = np.zeros((3, 1), np.float32)
        for m in range(0, 3):
            if D[yc[m], xc[m]] == 0 or flowmask[yc[m], xc[m]] == 0:
                break
            dc[m] = D[yc[m], xc[m]]
        else:
            pc = np.hstack((np.float32(xc), np.float32(yc), dc))
            pcand = np.zeros((4, 1), np.float32)
            pcand[0:3] = np.matrix(np.cross(pc[0,:]-pc[2,:], pc[1,:]-pc[2,:])).T
            pcand[0:3] = pcand[0:3]/np.linalg.norm(pcand[0:3])
            pcand[3] = -np.dot(pc[2,:], pcand[0:3])
            if pcand[3] < 0:
                pcand = -pcand
            P = pcand[0]*X + pcand[1]*Y + pcand[2]*D + pcand[3]
            countcand = cv2.countNonZero(cv2.bitwise_and(np.uint8(np.fabs(P) < threshold), flowmask))
            if countcand > count:
                count = countcand
                p = pcand

    print 'Max count ', count
    P = p[0]*X + p[1]*Y + p[2]*D + p[3]
    submask = np.uint8(cv2.bitwise_and(np.uint8(np.abs(P) < threshold), flowmask))
    kernel = np.ones((5,5),np.uint8)
    submask = cv2.erode(submask,kernel,iterations = 1)
    mask = cv2.GC_BGD*np.ones(D.shape, np.uint8)
    mask = cv2.GC_FGD*submask + cv2.GC_PR_BGD*(1 - submask)

    return submask
