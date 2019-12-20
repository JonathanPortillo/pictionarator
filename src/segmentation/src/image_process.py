import numpy as np
from skimage import data, util
from skimage.measure import label
from rigid_transform_3D import rigid_transform_3D

# segment pointcloud
# need segmented image first, also need camera dimensions
# use pointcloud_segmentation.segment_pointcloud

# use RGB to match objects
def match_objects(cluster1, cluster2):
    # takes in the clustered objects in starting and final image
    # returns a list of tuples of matching objects
    return

# calculate rotation from start to end
def rotation(img1, img2):
    label_img1 = label(img1, connectivity=img1.ndim)
    properties = regionprops(label_img1)
    orientation1 = properties.orientation

    label_img2 = label(img2, connectivity=img2.ndim)
    properties = regionprops(label_img2)
    orientation2 = properties.orientation

    return orientation2 - orientation1


# calculate Euclidean transform between two point clouds
def transform(A,B):
    return rigid_transform_3D(A,B)

# calculate grasp orientation
def grasp_orientation(img):
    label_img = label(img, connectivity=img.ndim)
    properties = regionprops(label_img)
    orientation = properties.orientation
    # assuming the grasper's resting orientation/origin is the x-axis
    grasp_angle = orientation - (np.pi / 2)
    return grasp_angle

# we need grasp depth

# find position of object relative to AR tag
def object_position(object_centroid, focal_dim, frame_dim):
    # params: 
    #   centroids: 2D coordinates (pixels) from the RGB image, 
    #   focal_dim: dimensions of image (pixels)
    #   frame_dim: dimensions of image (METERS)
    # returns: vector from one centroid to the other in METERS
    # assumptions: the AR tag is in the top-left corner since the origin for the image is the top-left
    TAG_LENGTH = 0.165

    # find coordinates of tag centroid assuming top left corner AND it's a square
    scale = focal_dim[0] / frame_dim[0] # ratio of pixel to irl length
    tag_pixel_length = TAG_LENGTH * scale
    tag_centroid = [tag_pixel_length, tag_pixel_length]

    # distance from tag to object
    pixel_vector = object_centroid - tag_centroid
    irl_vector = pixel_vector / scale

    return irl_vector


def main():
    # I guess this puts it all together
    return
