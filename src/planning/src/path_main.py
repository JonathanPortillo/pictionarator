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
else:
    from intera_interface import Limb
    from baxter_interface import Gripper

import rospy
import numpy as np
import traceback
#import os

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from controller import Controller



#need to somehow get PoseStamped message from CV stuff
def getGoal():
    #naive goal, replace



#moveit_msgs/OrientationConstraint message
def getOrientationConstraint():
    #naive orientation constraint, replace

#for main: idea is to getGoals from CV
#then calculate angles for the path (IK)
#then use path planning and then controller to execute the path
# after create an obstacle in the space of the placed object
def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    if ROBOT == "sawyer":
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    else:
        Kp = 0.45 * np.array([0.8, 2.5, 1.7, 2.2, 2.4, 3, 4])
        Kd = 0.015 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])


    ##
    ## Add the obstacle to the planning scene here
    ##

    #one for table
    # size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
    # name: unique name of the obstacle (used for adding and removing)
    # pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
    planner.add_box_obstacle()

    controller = Controller(Kp, Ki, Kd, Kw, Limb('right'))


    while not rospy.is_shutdown():
        try:
            #goal: a list of geometry_msgs/PoseStamped messages
            #movement specified might need 2 pose stamped messages
            #one that moves to the top of the object, and one that
            #specifies moving vertically down
            horiz_goal, vert_goal = getGoal()

            #moveit_msgs/OrientationConstraint message
            orientation_constraints = getOrientationConstraint()


            plan = planner.plan_to_pose(goal, orientation_constraints)

            raw_input("Press <Enter> to move the right arm to pose: ")

            if not controller.execute_path(plan):
                raise Exception("Execution failed")

            #should update next run with obstacles


        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break


    # sample from path_test.py from Lab 7
    # while not rospy.is_shutdown():
    #
    #     while not rospy.is_shutdown():
    #         try:
    #             if ROBOT == "baxter":
    #                 x, y, z = 0.47, -0.85, 0.07
    #             else:
    #                 x, y, z = 0.8, 0.05, -0.23
    #             goal_1 = PoseStamped()
    #             goal_1.header.frame_id = "base"
    #
    #             #x, y, and z position
    #             goal_1.pose.position.x = x
    #             goal_1.pose.position.y = y
    #             goal_1.pose.position.z = z
    #
    #             #Orientation as a quaternion
    #             goal_1.pose.orientation.x = 0.0
    #             goal_1.pose.orientation.y = -1.0
    #             goal_1.pose.orientation.z = 0.0
    #             goal_1.pose.orientation.w = 0.0
    #
    #             # Might have to edit this . . .
    #             plan = planner.plan_to_pose(goal_1, [])
    #
    #             raw_input("Press <Enter> to move the right arm to goal pose 1: ")
    #             # if not planner.execute_plan(plan):
    #             if not controller.execute_path(plan):
    #                 raise Exception("Execution failed")
    #         except Exception as e:
    #             print e
    #             traceback.print_exc()
    #         else:
    #             break



if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
