#!/usr/bin/env python




import math, os, sys, time
import rospy
import ros_numpy
import numpy as np
import scipy.misc

# ROS Image message
import message_filters
import imutils
from sensor_msgs.msg import PointCloud2
from ar_track_alvar_msgs.msg import AlvarMarkers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d as a3
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from scipy.spatial import ConvexHull

from sklearn.neighbors.kde import KernelDensity
import time
import PIL
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import icp
# from model_generator import Creation, Strategy



received_points = False
received_marker = False

_EPS = np.finfo(float).eps * 4.0

pointclouds = []
marker_rotations = []
marker_translations = []

BP_Y = 0.381
BP_X = 0.3175

def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


def pointcloud_vis(xyz, bbox, thresh=0.02):
    # x, y, z = xyz[::5].T
    # mask = z < thresh
    # fig = plt.figure()
    # ax = Axes3D(fig)

    # x1, y1, x2, y2 = bbox
    # # zs = np.linspace(0.047, 0.0376*12, 12)
    # # print(zs)
    # # for zi in zs:
    # #     a = [x1, x2, x2, x1]
    # #     b = [y1, y1, y2, y2]
    # #     c = [zi, zi, zi, zi]

    # #     vtx = [zip(a, b, c)]
    # #     ax.add_collection3d(Poly3DCollection(vtx))
    # ax.scatter(x, y, z, c = z)
    # ax.set_xlim(x1, x2)
    # ax.set_ylim(y1, y2)
    # ax.set_zlim(0, 0.4)
    # plt.show()
    # return mask

    x, y, z = xyz[::5].T 
    x1, y1, x2, y2 = bbox 
    xs = np.linspace(x1, x2, num=11) 
    ys = np.linspace(y1, y2, num=13) 

    for xi in xs: 
    	for yi in ys: 
    		plt.plot([xi, xi], [y1, y2]) 
    		plt.plot([x1, x2], [yi, yi]) 

    plt.scatter(x, y, c=z) 
    plt.colorbar() 
    plt.show() 


def pointcloud_callback(pointcloud):
    global received_points
    if received_points is False:
        try:
            print("Reached the pointcloud callback!")
            pc_arr = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud)
            xyz_array = ros_numpy.point_cloud2.get_xyz_points(pc_arr)
	    pointclouds.append(xyz_array)
            np.save("model3/pc_arrs/model_3pc", xyz_array)
            received_points = True
        except CvBridgeError as e:
            print(e)
    else:
      pass

def ar_marker_callback(ar_pose):
    global received_marker
    if received_marker is False:
        markers = ar_pose.markers
        assert len(markers) == 1
        marker = markers[0]
        pose = marker.pose.pose
        trans = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        R = quaternion_matrix(q)[:3, :3]
        marker_rotations.append(R)
        marker_translations.append(trans)
        received_marker = True
    else:
        pass

def transform(pc, R, t):
    return np.dot(R.T, ((pc - t).T)).T

def find_baseplate(pointcloud):
    mins = np.array([np.amin(pointcloud[:, i]) for i in range(3)])
    maxs = np.array([np.amax(pointcloud[:, i]) for i in range(3)])

    tr_x = np.linspace(mins[0], maxs[0], num=30)
    tr_y = np.linspace(mins[1], maxs[1], num=30)

    xy = pointcloud[:, :2]
    most_points = float('-inf')
    best_xy = None

    for x in tr_x:
        for y in tr_y:
            bbox_mins = np.array([x, y])
            bbox_maxs = bbox_mins + np.array([BP_X, BP_Y])
            bbox_mask = np.all((xy < bbox_maxs) & (xy > bbox_mins), axis=1)
            points_in = len(pointcloud[bbox_mask])
            if points_in > most_points:
                most_points = points_in
                best_xy = (x, y)
    x, y = best_xy
    return (x, y, x + BP_X, y + BP_Y)

def voxelize(pc, bbox, num_clouds=1):
    y_split = 12
    x_split = 10
    x1, y1, x2, y2 = bbox
    z1 = 0.047

    dx = (x2 - x1) / x_split
    dy = (y2 - y1) / y_split
    dz = 0.0376

    x_indices = np.floor((pc[:, 0] - x1) / dx).astype(np.int32)
    y_indices = np.floor((pc[:, 1] - y1) / dy).astype(np.int32)
    z_indices = np.floor((pc[:, 2] - z1) / dz).astype(np.int32)

    grid = np.zeros((10, 12, 8)).astype(np.int32)
    vox_sizes = []
    for i in range(10):
        for j in range(12):
            for k in range(8):
                part_mask = (x_indices == i) & (y_indices == j) & (z_indices == k)
                num_points = len(pc[part_mask])
                #print(num_points)
                if num_points > 0:
                    vox_sizes.append(num_points)
    print(sorted(vox_sizes))
    #print("CLOUD NUMBER = ", num_clouds)
    init_thresh = 90 * num_clouds
    vox_sizes = sorted(vox_sizes)[::-1]
    cutoff_ind = len([v for v in vox_sizes if v > init_thresh])

    new_ind = int(4 * round(float(cutoff_ind)/4))
    new_thresh = (vox_sizes[new_ind] + vox_sizes[new_ind - 1]) / 2
    print("init ", init_thresh)
    print("new ", new_thresh)

    print(cutoff_ind)
    print(len([v for v in vox_sizes if v > new_thresh]))

    for i in range(10):
        for j in range(12):
            for k in range(8):
                part_mask = (x_indices == i) & (y_indices == j) & (z_indices == k)
                num_points = len(pc[part_mask])
                # #print(num_points)
                # if num_points > 0:
                #     vox_sizes.append(num_points)
                if num_points > new_thresh:
                    #grid[i, j] = k + 1
                    grid[i, j, k] = 1

    temp_grid = np.zeros((10, 12)).astype(np.int32)
    for k in range(8):
        for i in range(10):
            for j in range(12):
                if grid[i, j, k] == 1:
                    temp_grid[i, j] = k + 1
        plt.imshow(temp_grid)
        plt.show()
                

    #grid = clean_up(grid)
    print(sorted(vox_sizes))
    #plt.imshow(grid)
    #plt.show()
    return grid

def sortNum(elem):
    return elem[2]


def voxelizeByLayer(pc, bbox, num_clouds=1):

    #SHIT NOT TO TOUCH
    y_split = 12
    x_split = 10
    x1, y1, x2, y2 = bbox
    z1 = 0.047

    dx = (x2 - x1) / x_split
    dy = (y2 - y1) / y_split
    dz = 0.0376

    x_indices = np.floor((pc[:, 0] - x1) / dx).astype(np.int32)
    y_indices = np.floor((pc[:, 1] - y1) / dy).astype(np.int32)
    z_indices = np.floor((pc[:, 2] - z1) / dz).astype(np.int32)

    print(pc.shape)
    final_grid = np.zeros((10, 12, 8)).astype(np.int32)
    init_thresh = 90 * num_clouds


    for k in range(1):
        voxels = []

        for i in range(10):
            for j in range(12):
                part_mask = (x_indices == i) & (y_indices == j) & (z_indices == k)
                num_points = len(pc[part_mask])
                if num_points > 0:
                    voxels.append([i, j, num_points])

        print("ADDED POINTS")
        print(voxels)

        grid = np.zeros((10, 12))

        voxels.sort(key=sortNum, reverse=True)
        #print(voxels)

        print("SORTED POINTS")
        print(voxels)

        cutoff_ind = len([v for v in voxels if v[2] > init_thresh])
        print("CUTOFF INDEX")
        print(cutoff_ind)

        if cutoff_ind % 4 != 0:
            new_ind = int(4 * round(float(cutoff_ind)/4))
            new_thresh = (voxels[new_ind][2] + voxels[new_ind - 1][2]) / 2
        else:
            new_thresh = init_thresh


        print("NEW THRESHOLD")
        print(new_thresh)

        voxels = np.array(voxels)
        #print(voxels)

        if len(voxels) > 0:
            thresh_mask = voxels[:, 2] > new_thresh
            print("THRESH MASK")
            print(thresh_mask)            

            ij = voxels[thresh_mask][:2]
            print("IJ")
            print(ij)

            orig_size = grid.shape
            grid_lin = grid.reshape(-1)
            lin_ij = ij[:, 1] + ij[:, 0] * 12
            print("LINEAR IJ")
            print(lin_ij) 

            grid_lin[lin_ij] = 1
            grid = grid.reshape(orig_size)
            print("LAYER")
            print(grid)

            final_grid[:, :, k] = grid[:, :]

    """
    temp_grid = np.zeros((10, 12)).astype(np.int32)
    for k in range(8):
        for i in range(10):
            for j in range(12):
                if final_grid[i, j, k] == 1:
                    temp_grid[i, j] += 1
        plt.imshow(temp_grid)
        plt.show()
    """

def voxelizePyramid(pc, bbox, num_clouds=1):
    y_split = 12
    x_split = 10
    x1, y1, x2, y2 = bbox
    z1 = 0.047

    dx = (x2 - x1) / x_split
    dy = (y2 - y1) / y_split
    dz = 0.0376

    x_indices = np.floor((pc[:, 0] - x1) / dx).astype(np.int32)
    y_indices = np.floor((pc[:, 1] - y1) / dy).astype(np.int32)
    z_indices = np.floor((pc[:, 2] - z1) / dz).astype(np.int32)

    print(pc.shape)

    grid = np.zeros((10, 12)).astype(np.int32)
    vox_sizes = []
    rad_sizes = []
    for i in range(10):
        for j in range(12):
            for k in range(8):
                part_mask = (x_indices == i) & (y_indices == j) & (z_indices == k)
                in_part = pc[part_mask]
                num_points = len(in_part)
                if num_points > 100 * num_clouds:
                    vox_sizes.append(num_points)
                    grid[i, j] = k + 1

    print(sorted(vox_sizes))
    plt.imshow(grid)
    plt.show()
    return grid

def voxelize_kde(pc, bbox, num_clouds=1):
    from scipy.stats import norm
    from sklearn.neighbors import KernelDensity
    
    y_split = 12
    x_split = 10
    (x1, y1), (x2, y2) = np.min(pc[:, :2], axis=0), np.max(pc[:, :2], axis=0)
    x1, y1, x2, y2 = bbox
    z1 = 0.047

    dx = (x2 - x1) / x_split
    dy = (y2 - y1) / y_split
    dz = 0.0376

    x_indices = np.floor((pc[:, 0] - x1) / dx).astype(np.int32)
    y_indices = np.floor((pc[:, 1] - y1) / dy).astype(np.int32)
    z_indices = np.floor((pc[:, 2] - z1) / dz).astype(np.int32)

    grid = np.zeros((10, 12)).astype(np.float32)
    vox_sizes = []
    rad_sizes = []
    for i in range(10):
        for j in range(12):
            part_mask = (x_indices == i) & (y_indices == j) & (z_indices >= 0)
            zs = pc[:, 2][part_mask]
            num_points = len(zs)
            if num_points > 90 * num_clouds:
                vox_sizes.append(num_points)
                #print(zs[:, np.newaxis].shape)
                kde = KernelDensity(kernel='gaussian', bandwidth=0.005).fit(zs[:, np.newaxis])
                samples = np.linspace(z1, 0.4, 1000)
                log_dens = kde.score_samples(samples[:, np.newaxis])
                dens = np.exp(log_dens).reshape(-1)
                peak = np.argmax(dens)
                layer = int((samples[peak] - z1) / dz) + 1
                grid[i, j] = layer #samples[peak]
                #plt.plot(samples, np.exp(log_dens))
                #plt.show()
                #grid[i, j] = 0
    plt.imshow(grid)
    plt.colorbar()
    plt.show()
    return grid


def compare_neighbors(arr):

    comp_arr = np.full(arr.shape, False, dtype=bool) #initialize
    arr_height = arr.shape[0]
    arr_width = arr.shape[1]

    for i in range(arr_height): #Row
        for j in range(arr_width): #column

            center = arr[i,j]

            #Check edges
            if i == 0: #left side
                left = arr[i,j]
            else:
                left = arr[i-1, j]

            if i == arr_height - 1: #right side
                right = arr[i,j]
            else:
                right = arr[i+1,j]

            if j == 0: #up
                up = arr[i,j]
            else:
                up = arr[i, j-1]

            if j == arr_width - 1: #down
                down = arr[i,j]
            else:
                down = arr[i, j+1]

            exposed = 0
            sides = [left, right, up, down]
            for side in sides:
            	if center != side:
            		exposed += 1

            if exposed > 2:
            	comp_arr[i, j] = 0
            else:
            	comp_arr[i, j] = 1

    #plt.imshow(comp_arr)
    #plt.show()
    return comp_arr


def voxelizeZ(pc, bbox, num_clouds=1):
    y_split = 12
    x_split = 10
    x1, y1, x2, y2 = bbox
    z1 = 0.047

    dx = (x2 - x1) / x_split
    dy = (y2 - y1) / y_split
    dz = 0.0376

    x_indices = np.floor((pc[:, 0] - x1) / dx).astype(np.int32)
    y_indices = np.floor((pc[:, 1] - y1) / dy).astype(np.int32)
    z_indices = np.floor((pc[:, 2] - z1) / dz).astype(np.int32)

    print(pc.shape)

    grid = np.zeros((10, 12)).astype(np.int32)
    vox_sizes = []
    for i in range(10):
        for j in range(12):
            part_mask = (x_indices == i) & (y_indices == j)
            num_points = len(pc[part_mask])
            #print(num_points)
            if num_points > 0:
                vox_sizes.append(num_points)

    print("CLOUD NUMBER = ", num_clouds)
    init_thresh = 120 * num_clouds
    vox_sizes = sorted(vox_sizes)[::-1]
    cutoff_ind = len([v for v in vox_sizes if v > init_thresh])

    new_ind = int(4 * round(float(cutoff_ind)/4))
    new_thresh = (vox_sizes[new_ind] + vox_sizes[new_ind - 1]) / 2

    print(cutoff_ind)
    print(len([v for v in vox_sizes if v > new_thresh]))

    for i in range(10):
        for j in range(12):
            part_mask = (x_indices == i) & (y_indices == j)
            # #print(num_points)
            # if num_points > 0:
            #     vox_sizes.append(num_points)
            if num_points > new_thresh:
                grid[i, j] = 5 #k + 1

    #grid = clean_up(grid)
    print(sorted(vox_sizes))
    plt.imshow(grid)
    plt.show()
    return grid    

def clean_up_by_layer(grid):
    max_height = np.amax(grid)
    result = np.zeros((10, 12)).astype(np.float32)
    for k in range(max_height):
        layer = np.zeros((10, 12)).astype(np.float32)
        for i in range(10):
            for j in range(12):
                if grid[i, j] > k:
                    layer[i, j] = 1

        mask = compare_neighbors(layer)
        for i in range(10):
            for j in range(12):
                layer[i, j] = layer[i, j] * mask[i, j]

        for i in range(10):
            for j in range(12):
                result[i, j] = result[i, j] + layer[i, j]

    plt.imshow(result)
    plt.show()
    return result



def clean_up_layer_old(layer, delete_here):
    for i in range(9):
        for j in range(11):
            if layer[i, j] > 0:
                if np.all(layer[i:i+2, j:j+2] > 0):
                    layer[i:i+2, j:j+2] = 0
                    delete_here[i:i+2, j:j+2] = False

def clean_up_old(grid):
    delete_here = np.ones((10, 12)).astype(bool)
    num_layers = np.amax(grid)
    
    for l in range(num_layers):
        current_layer = np.zeros((10, 12)).astype(int)
        current_layer[grid > l] = 1
        clean_up_layer(current_layer, delete_here)

    baseplate_mask = (grid == 0)
    grid[baseplate_mask | delete_here] = 0
    return grid

def get_one_pointcloud():
    global received_points
    global received_marker
    t0 = time.time()
    
    rospy.init_node('image_listener')
    
    pointcloud_topic = "/camera/depth/color/points"
    depth_topic = "/camera/depth/image_rect_raw"
    image_topic = "/camera/color/image_raw"
    # ar_topic = '/ar_pose_marker'
    
    # while not rospy.core.is_shutdown() and not received_marker:
        
    #     rospy.Subscriber(ar_topic, AlvarMarkers, ar_marker_callback)

    # rospy.rostime.wallsleep(0.05)

    # print("Got Marker!")

    while not rospy.core.is_shutdown() and not received_points:
        rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)


    t1 = time.time()

    print "Received Pointcloud and Transform in", t1 - t0, "seconds" 


def main():

    global received_points
    global received_marker
    #this saves an xyz_array to mode13/pc_arrs/model_3pc from a pc_array
    get_one_pointcloud()
    num_clouds = 1

    while True:
        inp = raw_input("Would you like to capture another view? (y/n)")
        if inp == 'n':
            break
        received_marker = False
        received_points = False
        get_one_pointcloud()
        num_clouds += 1


    #if there are multiple captures pointcloud_callback appends another xyz array onto the model_3pc npy array the three following lines assert that they are correctly appended    
    # assert len(pointclouds) == num_clouds
    # assert len(marker_rotations) == num_clouds
    # assert len(marker_translations) == num_clouds
    
    # pc = [transform(p, r, t) for p, r, t in zip(pointclouds, marker_rotations,
    #                                                         marker_translations)]

    # length = min([len(x) for x in pc])
    # pc = [y[:length] for y in pc]

    # new_points = icp.icp_all(pc)


    # t2 = time.time()
    # no_floor = new_points[new_points[:, 2] > 0.01]
    # bbox = find_baseplate(no_floor)
    # t3 = time.time()

    # print "Found Baseplate in", t3 - t2, "seconds"

    # mi1, mi2, ma1, ma2 = bbox
    # mins, maxs = np.array([mi1, mi2]), np.array([ma1, ma2])
    # bbox_mask = np.all((no_floor[:, :2] > mins) & (no_floor[:, :2] < maxs), axis=1)
    # filtered = no_floor[bbox_mask]
    # pointcloud_vis(filtered, bbox)
    # grid = voxelize_kde(filtered, bbox, num_clouds)
    # result = clean_up_by_layer(grid).astype(np.int32)

    # creation = Creation(result)
    # strategy = Strategy(creation).compute_strategy()

    # np.save('strategy', strategy)

    # print(strategy)

    # plt.imshow(result)
    # plt.colorbar()
    # plt.show()

    #voxelize(filtered, bbox, num_clouds)

if __name__ == '__main__':
    main()

