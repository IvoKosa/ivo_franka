#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np

from franka_move import MoveGroupPyInterface

def rectification(x, y, z, w):
    # Rectification to align camera x axis with targets z axis

    r = R.from_quat([x, y, z, w])
    ry = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
    rz = R.from_rotvec(np.pi * np.array([0, 0, 1]))
    Rtot = r*ry*rz
    return Rtot.as_quat()

def getTFPose():
    views_size=3
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

def move_to_coords(vsTargets, move):
    
    for i in range(len(vsTargets)):
        rec = rectification(vsTargets[i].orientation.x, vsTargets[i].orientation.y, vsTargets[i].orientation.z, vsTargets[i].orientation.w)
        vsTargets[i].orientation.x = rec[0]
        vsTargets[i].orientation.y = rec[1]
        vsTargets[i].orientation.z = rec[2]
        vsTargets[i].orientation.w = rec[3]

        print("------- Moving pos", str(i), " -------")

        move.go_to_pose_goal(vsTargets[i].position.x, vsTargets[i].position.y, vsTargets[i].position.z, vsTargets[i].orientation.x, vsTargets[i].orientation.y,vsTargets[i].orientation.z, vsTargets[i].orientation.w)

        print("-------- Sleeping --------")

        rospy.sleep(5)

def main():

    rospy.init_node("poses_getter", anonymous=True)

    vsTargets = getTFPose()

    print(vsTargets)

    move = MoveGroupPyInterface()
    move.addCollisionObjects()

    move_to_coords(vsTargets, move)

    print("---------------------- Finished Planning ----------------------")

if __name__ == '__main__':
    main()