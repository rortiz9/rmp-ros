#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
import tf
import math
import numpy as np
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

sigma = 0.01

def CollisionController(rw, s, sdot):
    w = (rw - s)# np.max(0, rw - s) ** 2
    w[w < 0] = 0
    w = w ** 2
    u = 1-np.exp(- sdot ** 2 / (2 * sigma ** 2))
    u[sdot > 0] = 0
    du = np.exp(- sdot ** 2 / (2 * sigma ** 2)) * sdot / (sigma ** 2)
    du[sdot > 0] = 0
    m = w * u + 0.5 * w * sdot * du
    return m


def AttractionController(rw, s, sdot):
    result = np.zeros(s.shape)
    good = - 10 * CollisionController(rw, s, sdot)
    result[-1,:] = good[-1,:]
    return result

def MotionPolicy(olds, s, avoid, goal):
    sdot = (s - olds) # might have to actually figure out velocity

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

    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    
    listener = tf.TransformListener()

    frames = ["/shoulder_pan_link", "/shoulder_lift_link", "/upperarm_roll_link",
              "/elbow_flex_link", "/forearm_roll_link", "/wrist_flex_link", "/wrist_roll_link",
              "/gripper_link"]

    obstacle = [[3.68883, 3.145310, 0.942161]]
    goal = [3.8, 3.15, 0.803]

    oldStates = np.zeros((8, 3))
    time = rospy.get_time()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pos = []
        ori = []
        for frame in frames:
            try:
                t = listener.getLatestCommonTime(frame, "/map")
                trans, rot = listener.lookupTransform(frame, "/map", t)
                pos.append(trans)
                ori.append(rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                break
        if len(pos) == 8:
            pos = np.array(pos)
	    acc = MotionPolicy(oldStates, pos, obstacle, goal)
            oldStates = pos

            acc = np.multiply(acc, ((rospy.get_time() - time) ** 2) / 2.0)
            time = rospy.get_time()

            poses = np.add(pos, acc)
            pose = poses[-2]
            rot = ori[-2]
            gripper_pose = Pose(Point(pose[0], pose[1], pose[2]),
                                Quaternion(rot[0], rot[1], rot[2], rot[3]))

            gripper_pose_stamped = PoseStamped()
            gripper_pose_stamped.header.frame_id = "/map"
            gripper_pose_stamped.header.stamp = t
            gripper_pose_stamped.pose = gripper_pose

            move_group.moveToPose(gripper_pose_stamped, frames[-2][1:])
        rate.sleep()
    move_group.get_move_action().cancel_all_goals()
