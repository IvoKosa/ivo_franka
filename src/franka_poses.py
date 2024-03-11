#!/usr/bin/env python3

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np

from franka_move import MoveGroupPyInterface

def rectification(x, y, z, w):
    r = R.from_quat([x, y, z, w])
    ry = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
    rz = R.from_rotvec(np.pi * np.array([0, 0, 1]))
    Rtot = r*ry*rz
    return Rtot.as_quat()

def getTFPose():
    views_size=32
    vsVector =[]

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    target_pose = geometry_msgs.msg.TransformStamped()

    for i in range(0, views_size):
        frame = 'tf_d'+str(i)
        transformStamped = tfBuffer.lookup_transform('panda_link0', frame, rospy.Time(0), rospy.Duration(10.0))
        target_pose = geometry_msgs.msg.Pose()
        
        target_pose.position.x = transformStamped.transform.translation.x
        target_pose.position.y = transformStamped.transform.translation.y
        target_pose.position.z = transformStamped.transform.translation.z
        target_pose.orientation = transformStamped.transform.rotation
        vsVector.append(target_pose)
    return vsVector  

def coord_targets(vsTargets):
    return 0

def main():

    rospy.init_node("poses_getter", anonymous=True)

    vsTargets = getTFPose()

    print(vsTargets)

    move = MoveGroupPyInterface()
    move.addCollisionObjects()

    move.go_to_pose_goal(0.35745, 0.028111, 0.532,  -0.00066075, 0.71554, -0.0004089, 0.698572)

    cartesian_plan, fraction = move.plan_cartesian_path()
    
    move.execute_plan(cartesian_plan)

if __name__ == '__main__':
    main()