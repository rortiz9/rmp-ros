#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
import tf
import math
import numpy as np
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import  geometry_msgs.msg

oldStates = []

sigma = 1

def CollisionController(rw, s, sdot):
    w = np.max(0, rw - s) ** 2
    u = 1-np.exp(- sdot ** 2 / (2 * sigma ** 2)) if sdot < 0 else 0
    du = np.exp(- sdot ** 2 / (2 * sigma ** 2)) * sdot / (sigma ** 2) if sdot < 0 else 0
    m = w * u + 0.5 * w * sdot * du
    return m


def AttractionController(rw, s, sdot):
    return - 10 * CollisionController(rw, s, sdot)

def MotionPolicy(s, avoid, goal):
    sdot = (s - oldStates) # might have to actually figure out velocity

    mp = 0
    for state in avoid:
        mp += CollisionController(state, s, sdot)
    mp += AttractionController(goal, s, sdot)
    oldStates = s

    return mp


# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
    rospy.init_node("RMP")
    
    listener = tf.TransformListener()

    frames = ["/shoulder_pan_link", "/shoulder_lift_link", "/upperarm_roll_link",
              "/elbow_flex_link", "/forearm_roll_link", "/wrist_flex_link", "/wrist_roll_link",
              "/gripper_link"]

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pos = []
        for frame in frames:
            try:
                t = listener.getLatestCommonTime(frame, "/map")
                pos.append(listener.lookupTransform(frame, "/map", t))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        print pos
	rate.sleep()
