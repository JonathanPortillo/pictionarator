#!/usr/bin/env python
"""
Path Planning Script for Pictionarator
Author: Samarpita Patra
"""
import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
    from baxter_interface import Gripper
    from baxter_interface import CHECK_VERSION
else:
    from intera_interface import Limb
    from baxter_interface import Gripper

import rospy
import numpy as np
import tf2_ros
import traceback
from ar_track_alvar_msgs.msg import AlvarMarkers
#import os

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
import tf

from path_planner import PathPlanner
from controller import Controller

from distance_between import Distance_Between
import imutils
import cv2
import cv_bridge
from sensor_msgs.msg import (
    Image,
)

ar_x = 0
ar_y = 0
ar_z = 0
arTransformStamp = None
ar_rot = None


#function to be run in main
#gets AR tag transform from left hand camera to base
#transform used to get grasping coordinates into base frame coordinates for the robot

def send_image():
    """
    Send the image located at the specified path to the head
    display on Baxter.
    @param path: path to the image file to load and send
    """
    # img = cv2.imread('/home/cc/ee106a/fa19/class/ee106a-afn/ros_workspaces/project/src/planning/src/images/display1.jpg')
    img = cv2.imread('/home/cc/ee106a/fa19/class/ee106a-afn/ros_workspaces/project/src/planning/src/images/display2.jpg')
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)

def tag_transform():
    global arTransformStamp

    source_topic = '/ar_marker_11'
    target_topic = '/base'

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    trans = None
    rot = None
    print("hdshsd")

    while not rospy.is_shutdown() and trans is None:
        try:
            # arTransformStamp = TransformStamped()
            # arTransformStamp.header.frame_id = "base"
            trans, rot = listener.lookupTransform(target_topic, source_topic, rospy.Time(0))
            # trans = listener.lookupTransform('/base', '/left_hand_camera', rospy.Time(0))
            # arTransformStamp.translation, arTransformStamp.rotation = y
            # arTransformStamp.translation = trans

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    arTransformStamp = TransformStamped()
    arTransformStamp.header.frame_id = 'base'

    arTransformStamp.transform.translation.x = trans[0]
    arTransformStamp.transform.translation.y = trans[1]
    arTransformStamp.transform.translation.z = trans[2]

    arTransformStamp.transform.rotation.x = rot[0]
    arTransformStamp.transform.rotation.y = rot[1]
    arTransformStamp.transform.rotation.z = rot[2]
    arTransformStamp.transform.rotation.w = rot[3]

#print("Translation got")

def getOrientationConstraints():
    return []

#need to somehow get PoseStamped message from CV stuff
def getGoal(x, y, z):
    #bx, by, bz = transformPoint(np.array([x, y, z]))
    # bx, by, bz = transformPoint(np.array([x, y, z]))

    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    #x, y, and z position
    goal_1.pose.position.x = x
    goal_1.pose.position.y = y
    goal_1.pose.position.z = z

    #Orientation as a quaternion
    goal_1.pose.orientation.x = 0.0
    goal_1.pose.orientation.y = -1.0
    goal_1.pose.orientation.z = 0.0
    goal_1.pose.orientation.w = 0.0

    return goal_1

def callback(message):
    global ar_x, ar_y, ar_z, haveMarker
    #Print the contents of the message to the console
    if message.markers is not []:
    ar_x = message.markers[0].pose.pose.position.x
    ar_y = message.markers[0].pose.pose.position.y
    ar_z = message.markers[0].pose.pose.position.z

    # print(ar_x, ar_y, ar_z)
    ar_x += ar_trans[0]
    ar_y += ar_trans[1]
    ar_z += ar_trans[2]

    haveMarker = True
    # print(ar_x, ar_y, ar_z)


    #print(rospy.get_name() + ": I heard %s" % message.markers[0].pose.pose.position.x)

def listener():
    global haveMarker

    #Run thils program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.


    #Create a new instance of the rospy.Subscriber object which we can
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    while not rospy.core.is_shutdown() and not haveMarker:
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

    print("Got AR marker!")

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    #rospy.spin()

def dropoff(x, y, z, point):

    # while not rospy.is_shutdown():
    #     #goal: a list of geometry_msgs/PoseStamped messages
    #     #movement specified might need 2 pose stamped messages
    #     #one that moves to the top of the object, and one that
    #     #specifies moving vertically down
    # ar_x = arTransformStamp.transform.translation.x
    # ar_y = arTransformStamp.transform.translation.y
    # ar_z = arTransformStamp.transform.translation.z

    while not rospy.is_shutdown():
      try:
          goal = getGoal(x + point[1], y - point[0], -0.1)
          #goal = getGoal(ar_x + points[0][0], -ar_y + points[0][1] , 0)
          #goal = getGoal(0.67, -0.14, 0.42)


          plan = planner.plan_to_pose(goal, [])

          raw_input("Press <Enter> to move the right arm to pose: ")

          if not planner.execute_plan(plan):
              raise Exception("Execution failed")
      except Exception as e:
          print e
          traceback.print_exc()
      else:
          # grip_right.close()
          # grip_right.open()
          break

    while not rospy.is_shutdown():
        try:
            goal = getGoal(ar_x + point[1], ar_y - point[0], -0.17)
            #goal = getGoal(ar_x + points[0][0], -ar_y + points[0][1] , 0)
            #goal = getGoal(0.67, -0.14, 0.42)


            plan = planner.plan_to_pose(goal, [])

            raw_input("Press <Enter> to move the right arm to pose: ")

            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
            traceback.print_exc()
        else:
            # grip_right.close()
            grip_right.open()
            break


def pickup (x, y, z) :

    # while not rospy.is_shutdown():
    # try:
    #    pickup = getGoal(x, y, z)
    #    orientation_constraints = getOrientationConstraints()
    #    plan = planner.plan_to_pose(pickup, orientation_constraints)
    #
    #    raw_input("Press <Enter> to move the right arm to pick up object: ")
    #
    #    if not planner.execute_plan(plan):
    #        raise Exception("Execution failed")
    #
    # except Exception as e:
    #    print e
    #    traceback.print_exc()
    # else:
    #    grip_right.close()

    while not rospy.is_shutdown():
        try:
            goal = getGoal(x -0.054, y - 0.21, 0)
            #goal = getGoal(ar_x + points[0][0], -ar_y + points[0][1] , 0)
            #goal = getGoal(0.67, -0.14, 0.42)

            plan = planner.plan_to_pose(goal, [])

            raw_input("Press <Enter> to move the right arm to pose: ")

            if not planner.execute_plan(plan):
              raise Exception("Execution failed")
      except Exception as e:
          print e
          traceback.print_exc()
      else:
          # grip_right.close()
          grip_right.open()
          break

  while not rospy.is_shutdown():
      try:
          goal = getGoal(x -0.054, y - 0.21, -0.18)
          #goal = getGoal(ar_x + points[0][0], -ar_y + points[0][1] , 0)
          #goal = getGoal(0.67, -0.14, 0.42)

          plan = planner.plan_to_pose(goal, [])

          raw_input("Press <Enter> to move the right arm to pose: ")

          if not planner.execute_plan(plan):
              raise Exception("Execution failed")
      except Exception as e:
          print e
          traceback.print_exc()
      else:
          grip_right.close()
          # grip_right.open()
          break


#for main: idea is to getGoals from CV
#then calculate angles for the path (IK)
#then use path planning and then controller to execute the path
# after create an obstacle in the space of the placed object
def main():
    """
    Main Script
    """
    grip_right = Gripper('right')

    # Make sure that you've looked at and understand path_planner.py before starting
    imagePathFile = '/home/cc/ee106a/fa19/class/ee106a-afn/ros_workspaces/project/src/planning/src/images/our17.jpg'
    planner = PathPlanner("right_arm")
    # add_block_obstacle(planner, [0.963, -0.15, -0.1], 'table', [10, 10, 0])
    # size = np.array([0.963, -0.15, -0.1])
    # pose = PoseStamped()
    # pose.header.frame_id = 'table'
    # pose.pose.position.x = 10
    # pose.pose.position.y = 10
    # pose.pose.position.z = 0

    # planner.add_box_obstacle(size, 'table', pose)

    #gripper calibration
    grip_right.calibrate()
    grip_right.open()

    # img_npy = 'distance_coordinates.npy'
    # points = np.load(img_npy, "r")
    # distBetween = Distance_Between('/home/cc/ee106a/fa19/class/ee106a-afn/ros_workspaces/project/src/planning/src/images/our16.jpg', 0.02426)
    #for second test
    distBetween = Distance_Between(imagePathFile, 0.02426)
    points = distBetween.getTransformInfo()

    # return None

    # print(points)

    # qx = p[2]
    # qy = p[3]
    # qz = p[4]
    # qw = p[5]

    #one for table
    # size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
    # name: unique name of the obstacle (used for adding and removing)
    # pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
    # pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
    # planner.add_box_obstacle()

    # controller = Controller(Kp, Ki, Kd, Kw, Limb('right')
    ar_x = arTransformStamp.transform.translation.x
    ar_y = arTransformStamp.transform.translation.y
    ar_z = arTransformStamp.transform.translation.z

    print(points[0][0], points[0][1])

    while not rospy.is_shutdown():
        try:
            for point in points:
                #pose 1
                pickup(ar_x, ar_y, ar_z)
                dropoff(ar_x, ar_y, ar_z, p)
        except Exception as e:
            print e
            traceback.print_exc()


if __name__ == '__main__':
    rospy.init_node('planning', anonymous = True)
    send_image()
    tag_transform()
    main()
