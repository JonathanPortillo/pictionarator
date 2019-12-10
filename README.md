# Pictionarator
eecs106a FInal Project

## Sources
1.) Creates a point cloud of an environment could be used to create a pointcloud of the environment we need to recreate: https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i

2.) Point set registration allows for matching to pointclouds : could be used as feedback to match the objects in the environment we want to recreate with our original image:
https://en.wikipedia.org/wiki/Point_set_registration 

http://www.open3d.org/docs/release/tutorial/Advanced/global_registration.html

http://wiki.ros.org/pointcloud_registration

3.) This finds the transformation between two point clouds. We can use clustering to segment each object then find the transfornmation of each object:
https://github.com/nghiaho12/rigid_transform_3D/blob/master/rigid_transform_3D.py
