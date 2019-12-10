import numpy as np
from skimage import data, util
from skimage.measure import label
from rigid_transform_3D import rigid_transform_3D

# segment pointcloud
# need segmented image first, also need camera dimensions

# use RGB to match objects


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

def main():
    # I guess this puts it all together
    return
