#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena

This file implements a ROS node that subscribes to topics for RGB images,
pointclouds, and camera calibration info, and uses the functions you
implemented to publish a segmented pointcloud to the topic /segmented_points.

Once you are confident in your implementation in image_segmentation.py and
pointcloud_segmentation.py, run this file to begin publishing a segmented
pointcloud.
"""

from __future__ import print_function
from collections import deque

import rospy
import message_filters
import ros_numpy
import tf

from rigid_transform_3d import rigid_transform_3D

from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import numpy as np
import cv2

from cv_bridge import CvBridge

from image_segmentation import segment_image
from pointcloud_segmentation import segment_pointcloud

def isolate_object_of_interest(points, image, cam_matrix, trans, rot):
    segmented_image = segment_image(image)
    points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
    return points

def get_camera_matrix(camera_info_msg):
    # TODO: Return the camera intrinsic matrix as a 3x3 numpy array
    # by retreiving information from the CameraInfo ROS message.
    # Hint: numpy.reshape may be useful here.
    K = np.array(camera_info_msg.K).reshape(3,3)
    return K

def get_transform(init_points, init_img, ending_points, ending_img):
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    POINTS_TOPIC = '/camera/depth/color/points'
    intrinsic_matrix = get_camera_matrix(CAM_INFO_TOPIC)
    trans, rot = self.listener.lookupTransform('/camera_color_optical_frame',
    '/camera_depth_optical_frame', rospy.Time(0))
    isolated_initial_points = isolate_object_of_interest(init_points, init_img,
    	intrinsic_matrix, np.array(trans), np.array(rot))
    isolated_ending_points = isolate_object_of_interest(ending_points, ending_img,
    	intrinsic_matrix, np.array(trans), np.array(rot))
    return rigid_transform_3D(isolated_initial_points, isolated_ending_points)


