#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import moveit_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import trajectory_msgs.msg
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
from moveit_commander.conversions import pose_to_list
from scipy.spatial.transform import Rotation as R
import numpy as np

import open3d as o3d
from sklearn.cluster import KMeans, DBSCAN, OPTICS
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
import copy
import subprocess

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

import time

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

bridge = CvBridge()
global bestGPlist 
bestGPlist = [] 
global bestSClist 
bestSClist  = [] 

def covGraphic(covPercentageNoAdj , covPercentageAdj):
   x = ["VsNoAdj", "VsAdj"]
   y = [covPercentageNoAdj , covPercentageAdj]
   plt.barh(x, y)
   plt.title("Views Coverage: Center Adjustment vs No Center Adjustment")
   plt.xlim([0,100])  
   for index, value in enumerate(y):
      plt.text(value, index,str(value))
   plt.savefig("/home/josepatino/ros/noetic/system/src/franka_vp/src/StatisticsGraphics/squares.png") 
   plt.show()
   

def RANSAC(pcd, dt,rn,ni):
    plane_model, inliers = pcd.segment_plane(distance_threshold=dt, ransac_n=rn,num_iterations=ni)
    [a, b, c, d] = plane_model
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return (inlier_cloud, outlier_cloud)

def removeOutBBX(pcd, center, size, transformation):
    center = center/1000
    size = size/1000
    center[2] = center[2] - (center[2] - size[2]/2)
    center2 = center
    center2 = np.vstack([center2[np.newaxis].T,[1]])
    upBounds = center + size/2
    lowBounds = center - size/2
    transformation[:3,3] = transformation[:3,3]/1000
    bounds = np.array([ [[upBounds[0]],[0],[0],[1]] , [[lowBounds[0]],[0],[0],[1]] , [[0],[upBounds[1]],[0],[1]] , [[0],[lowBounds[1]],[0],[1]] , [[0],[0],[upBounds[2]],[1]] , [[0],[0],[lowBounds[2]],[1]] ])
    T0_w = np.linalg.inv(transformation)
    boundTransformed = np.matmul(T0_w, bounds)
    finalPoints = []
    for i in boundTransformed:
       finalPoints.append( [ i[0][0], i[1][0], i[2][0]] )
       #finalPoints.append( [ i[1][0], -i[0][0], i[2][0]] )
    centerTr = np.matmul(T0_w, center2) 
    centerTr = np.array([centerTr[0][0],centerTr[1][0],centerTr[2][0]])
    #centerTr = np.array([centerTr[1][0],-centerTr[0][0],centerTr[2][0]])
    finalPoints = np.array(finalPoints)
    ux = (finalPoints[0] - finalPoints[1])[np.newaxis].T
    ux = ux/np.linalg.norm(ux)
    #print(np.linalg.norm(ux))
    uy = (finalPoints[2] - finalPoints[3])[np.newaxis].T
    uy = uy/np.linalg.norm(uy)
    #print(np.linalg.norm(uy))
    uz  = (finalPoints[4] - finalPoints[5])[np.newaxis].T
    uz = uz/np.linalg.norm(uz)
    #print(np.linalg.norm(uz))
    R = np.hstack( [np.hstack([ux,uy]),uz] )
    #print("R: ",R)
    bbx=o3d.geometry.OrientedBoundingBox(centerTr, R, size+0.00003)
    indexs = bbx.get_point_indices_within_bounding_box(pcd.points)
    #print(indexs)
    pcd = pcd.select_by_index(indexs)
    o3d.visualization.draw_geometries([pcd,bbx])#,zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])

    return pcd

def outlierRemoval(pcd,securityFactor, modo):
    points = np.asarray(pcd.points)
    media = np.mean(np.asarray(points), axis=0)

    #print( "Media: ", media )
    q75, q25 = np.percentile( points, [75 ,25], axis=0)
    iqr = q75 - q25
    #print( "q75: ", q75)
    #print( "q25: ", q25)
    upBound = q75 + securityFactor*iqr
    lowBound = q25 - securityFactor*iqr
    #print( "upBound: ", upBound)
    #print( "lowBound: ", lowBound)
    
    if modo=="z":
     pcl = o3d.geometry.PointCloud()
     bounds = np.array( [[0,0,upBound[2]], [0,0,lowBound[2]] ] )
     pcl.points = o3d.utility.Vector3dVector( bounds )
     boundColors = np.array( [[0,0,1], [1,0,0] ] )
     pcl.colors = o3d.utility.Vector3dVector( boundColors )
     #o3d.visualization.draw_geometries([pcd, pcl])
    
     index = np.argwhere(np.asarray(pcd.points)[:,2]<upBound[2])
     #print("Index: ",index)
     final = pcd.select_by_index(index)
     return final
    
    if modo=="total":
     pcl = o3d.geometry.PointCloud()
     bounds = np.array( [[0,0,upBound[2]], [0,0,lowBound[2]] ] )
     pcl.points = o3d.utility.Vector3dVector( bounds )
     boundColors = np.array( [[0,0,1], [1,0,0] ] )
     pcl.colors = o3d.utility.Vector3dVector( boundColors )
     #o3d.visualization.draw_geometries([pcd, pcl])
    
     index = np.argwhere((np.asarray(pcd.points)[:,2]>lowBound[2]) * (np.asarray(pcd.points)[:,2]<upBound[2]) * (np.asarray(pcd.points)[:,0]>lowBound[0]) * (np.asarray(pcd.points)[:,0]<upBound[0]))
     #print("Index: ",index)
     final = pcd.select_by_index(index)
     return final
    #o3d.visualization.draw_geometries([final])
    


def color_callback(msg):
    # print("Received color image!")
    global color_img
    color_img = bridge.imgmsg_to_cv2(msg, "rgb8")
    #print("saving color image")
    #print(color_img)

def depth_callback(depth_msg):
    global depthimg
    bridge = CvBridge()
    depth_img = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    cv_image_array = np.array(depth_img, dtype=np.dtype("f8"))
    cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    # cv_image_resized = cv2.resize(cv_image_norm, interpolation=cv2.INTER_CUBIC)
    depthimg = depth_img

def vs2O3D(transformation): 
    transformation[:3,3] = transformation[:3,3]/1000
    tx= transformation[0,3]
    ty= transformation[1,3]
    tz= transformation[2,3]

    transformation[0,3] = ty
    transformation[1,3] = -tx
    transformation[2,3] = tz

    # print("points:", np.matmul( transformation, np.asarray(pcd2.points) ))


    #transformation[:3,3] = transformation[:3,3]/1000
    r = R.from_matrix(transformation[:3,:3])
    #print("Matriz = ", r.as_matrix())
    #print("RotVec = ", r.as_rotvec())

    rotvec = r.as_rotvec()
    #rotvec[0] = -rotvec[0]

    x = rotvec[0]
    y = rotvec[1]
    z = rotvec[2]

    #Se realizan estas transformaciones porque el sistema de coordenadas de open 3D difiere del sistema de coordenadas de la hemiesfera
    rotvec[0] = y 
    rotvec[1] = -x 
    rotvec[2] = z 

    #rotvec[1] = -rotvec[1]
    #print("RotVecM = ", rotvec)
    rM = R.from_rotvec(rotvec).as_matrix()
    #print("MatrizM = ", rM)
    #print(transformation)
    transformation[:3,:3] = rM
    return transformation

def getVSTF(i):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    Q0_i = tfBuffer.lookup_transform('tf_d0', 'camera_depth_optical_frame', rospy.Time(0), rospy.Duration(10.0))
    x_pos = Q0_i.transform.translation.x
    y_pos = Q0_i.transform.translation.y
    z_pos = Q0_i.transform.translation.z  
    orientation = Q0_i.transform.rotation
    #print("Orientatiooooooooon========= ",orientation)
     
    R0_i = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
    t0_i = np.array([[x_pos],[y_pos],[z_pos]])/1000
    T0_i = np.hstack([R0_i, t0_i])
    T0_i = np.vstack([T0_i, np.array([[0,0,0,1]])])
    #print('HTM==== ',T0_i)
    #if i !=0:
    # T0_i = vs2O3D(T0_i)
    return T0_i
    
def camera_info_callback(msg, i):
    global K
    global aligned_depth_image
    cam_info = msg.K
    K = np.array(
        [
            [cam_info[0], 0.0, cam_info[2]],
            [0.0, cam_info[4], cam_info[5]],
            [0.0, 0.0, 0.0],
        ]
    )
    data_dict = {
            "rgb": np.array(color_img),
            #"depth_raw": self.depth_array / 1000.0,
            "depth": np.array(depthimg) / 1000.0,
            "label": np.zeros((720, 1280), dtype=np.uint8),
            "K": K,
        }
    #np.save("/home/josepatino/ros/noetic/system/src/franka_vp/src/RawScene" + "/raw_capture.npy", data_dict)
    di = o3d.geometry.Image( np.array(depthimg)/1000 )
    
    ci = o3d.geometry.Image( ((np.array(color_img)[:, ::-1])) )
    #print(di)
    #print("Colors")
    #print(np.array(ci))
    rospy.sleep(5)
    
    intrinsic = o3d.camera.PinholeCameraIntrinsic(msg.width, msg.height, cam_info[0], cam_info[4], cam_info[2], cam_info[5])
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(ci, di, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    '''
    R = pcd.get_rotation_matrix_from_xyz((0,np.pi , np.pi))
    pcd = pcd.rotate(R)
    '''
    ic, oc = RANSAC(pcd,0.000002,200,1000)#0.000005,200,1000
    #print( "numero de puntos: ", len(np.asarray(oc.points)) )
    
    if len(np.asarray(oc.points))>66200:
     ic2, oc2 = RANSAC(oc,0.000005,50,1000)#0.0000008,50,1000
     #print( "IC2: ", len(np.asarray(ic2.points)) )
     #o3d.visualization.draw_geometries([ic2,oc2])
     if len(np.asarray(ic2.points))>23500:
      ic = ic2
      oc = oc2
    #print( "IC: ", len(np.asarray(ic.points)) )
    
    #o3d.visualization.draw_geometries([oc]) // descomentar si hay algun problema con el point cloud final
    final = outlierRemoval(oc,10,"z")
    #print("hola11")
    final.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.0001, max_nn=30))
    #print("hola12")
    final.orient_normals_towards_camera_location()
    '''
    final.orient_normals_consistent_tangent_plane(100)
    print("hola13")
    final.normals= o3d.utility.Vector3dVector( -np.asarray(final.normals) )
    '''
    final = final.voxel_down_sample(voxel_size=0.000005) #0.00001
    #print("Raw PCD")
    
    
    #o3d.visualization.draw_geometries([final],point_show_normal=True,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0], zoom=0.89999999999999996)
    
    
    return final
    
    
def draw_registration_result(source, target, transformation,Tw_0):
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.00005)
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    
    #transformation = vs2O3D(transformation)
    source_temp.transform(transformation)
    #o3d.visualization.draw_geometries([source_temp,origin]) #amarillo
    #o3d.visualization.draw_geometries([target_temp,origin]) #celeste
    #o3d.visualization.draw_geometries([source_temp, target_temp], front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0], zoom=0.89999999999999996)
    #o3d.visualization.draw_geometries([source_temp, target_temp],front = [ 0.0, 0.0, 1.0 ],lookat= [ -0.011450698783788519, 0.061363026798163252, 0.014929970929408806 ],up = [ 0.0, 1.0, 0.0 ],zoom= 0.7)
    #,front = [ 0.0, 0.0, 1.0 ],lookat= [ 0,0,0 ],up = [ 0.0, 1.0, 0.0 ],zoom= 0.7
    pcdFin = source_temp + target_temp
    #print("Registered PCD")
    #o3d.visualization.draw_geometries([pcdFin, origin],front = [0, 0, -1], lookat = [9.8028275533568133e-06,5.3913320185515508e-05,0.00018610356798528431], up = [0, -1, 0], zoom =0.7)#0.005
    threshold = 0.02/1000
    evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, transformation)
    #print(evaluation)
    #tfBuffer1 = tf2_ros.Buffer()
    #listener1 = tf2_ros.TransformListener(tfBuffer1)
    #Tw_0 = pose2HTM( (tfBuffer1.lookup_transform('panda_link0', "tf_d0", rospy.Time(0), rospy.Duration(10.0))).transform )
    Tw_0cop =copy.deepcopy(Tw_0)
    ocz = (object_size[2]/2)#(0.725+(object_size[2]/2)) - ((0.725+(object_size[2]/2))-(object_size[2]/2)) #object center in z
    pcdFin = removeOutBBX(pcdFin, np.array([object_position[0], object_position[1], ocz]), np.array([object_size[0],object_size[1],object_size[2]]), Tw_0cop)
    return pcdFin  #source y target fusionados    
    #print("hola")
    #o3d.visualization.draw_geometries([pcd])
    
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


def rectification(x, y, z, w):
  #print("-----------rect0-----------")
  #print([x, y, z, w])
  r = R.from_quat([x, y, z, w])
  #print(r.as_matrix())
  ry = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
  #print(ry.as_matrix())
  rz = R.from_rotvec(np.pi * np.array([0, 0, 1]))
  #print(rz.as_matrix())
  Rtot = r*ry*rz
  #print(Rtot.as_matrix())
  #print("-----------rect1-----------")
  return Rtot.as_quat()

def getTFPose():
  views_size=32
  vsVector =[]
  
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
  
  target_pose = geometry_msgs.msg.TransformStamped()
  
  for i in range(0, views_size):
   frame = 'tf_d'+str(i)
   #print(frame)
   transformStamped = tfBuffer.lookup_transform('panda_link0', frame, rospy.Time(0), rospy.Duration(10.0))
   target_pose = geometry_msgs.msg.Pose()
   
   target_pose.position.x = transformStamped.transform.translation.x;
   target_pose.position.y = transformStamped.transform.translation.y;
   target_pose.position.z = transformStamped.transform.translation.z;  
   target_pose.orientation = transformStamped.transform.rotation;
   vsVector.append(target_pose)
  
  #print(vsVector)
  return vsVector  

def pos2HTM(pose):# Pose 2 HTM
   p = np.array([ [pose.position.x], [pose.position.y], [pose.position.z] ])
   quat = pose.orientation
   rotM = R.from_quat(np.array([quat.x, quat.y, quat.z, quat.w])).as_matrix()
   HTM = np.hstack([rotM, p])
   HTM = np.vstack([HTM, np.array([[0, 0, 0, 1]])])
   return HTM

def pose2HTM(pose): #transform stamped to HTM
   p = np.array([ [pose.translation.x], [pose.translation.y], [pose.translation.z] ])
   quat = pose.rotation
   rotM = R.from_quat(np.array([quat.x, quat.y, quat.z, quat.w])).as_matrix()
   HTM = np.hstack([rotM, p])
   HTM = np.vstack([HTM, np.array([[0, 0, 0, 1]])])
   return HTM

def pcd2RGBD(pcd, intrinsic): # To convert pcd 2 RGBD when no more view poses are available

 (h, w) = (intrinsic.height,intrinsic.width)

 #intrinsic = o3d.camera.PinholeCameraIntrinsic(h,w, 924.2759399414062, 924.2759399414062, 640, 360)

 fx = intrinsic.intrinsic_matrix[0][0]
 fy = intrinsic.intrinsic_matrix[1][1]
 cx = intrinsic.intrinsic_matrix[0][2]
 cy = intrinsic.intrinsic_matrix[1][2]


 depthoutputimg = np.zeros((h,w), np.uint16) # (h,w)
 coloroutputimg = np.zeros((h,w,3), np.float32)
 for i in range(len(pcd.points)):
        x = pcd.points[i][0]
        y = pcd.points[i][1]
        z = pcd.points[i][2]

        d = round(z*1000000)
        u = round(x*fx/z+cx)
        v = round(y*fy/z+cy)
        #print(u)
        #print(v)
        depthoutputimg[v][u] = d
        coloroutputimg[v, u, :] = np.asarray(pcd.colors)[i] 
 return [depthoutputimg, coloroutputimg]

def graspHTM2msg(n2Fgrasp):
   rrotate = np.vstack( ( np.hstack( ( R.from_euler('z', 90, degrees=True).as_matrix(), np.zeros([3,1]) ) ) , np.array([[0,0,0,1]]) ) )

   graspTestRotation = R.from_matrix(n2Fgrasp[:3,:3]).as_quat()
   graspTestRotation = R.from_matrix(np.matmul(n2Fgrasp[:3,:3],rrotate[:3,:3])).as_quat()
   
   t2 = geometry_msgs.msg.TransformStamped()
   t2.header.frame_id = "world"
   t2.header.stamp = rospy.get_rostime()
   t2.child_frame_id = "graspTest"
   t2.transform.translation.x = n2Fgrasp[0,3]
   t2.transform.translation.y = n2Fgrasp[1,3]
   t2.transform.translation.z = n2Fgrasp[2,3]
   t2.transform.rotation.x = graspTestRotation[0]
   t2.transform.rotation.y = graspTestRotation[1]
   t2.transform.rotation.z = graspTestRotation[2]
   t2.transform.rotation.w = graspTestRotation[3]
   return t2

def getBestTTGrasp(grasp,scores):
   pred = grasp 
   scores = scores 

   near2FarGraspsIndex = np.argsort(np.sum(pred[:,:2,3]**2, axis=1))
   near2FarGrasps = pred[near2FarGraspsIndex]
   near2FarGrasps = near2FarGrasps[:20]

   
   topIndex = np.argsort(near2FarGrasps[:,2,2]+1)

   return [near2FarGrasps, topIndex]
   '''
   print(len(topIndex[0]))
   if len(topIndex[0])>0:
     pred = pred[topIndex]
     #print(topIndex)
     scores = scores[topIndex]
     #print(scores)
     topScIndex = np.argmax(scores)
     pred = pred[topScIndex]
     #rospy.sleep(5)
     bestSClist.append(scores[topScIndex])
     return pred
   '''
   '''
   else:
      topScIndex = np.argmax(scores)
      pred =  pred[topScIndex]
      bestSClist.append(scores[topScIndex])
      #ospy.sleep(5)
      return pred
   '''

   
      



class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        
       

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        #move_group.set_planner_id("RRTMod")

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        #move_group.set_end_effector_link("panda_leftfinger")
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL
        #move_group.set_planning_time(0) 	
        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        '''
        cylpose = geometry_msgs.msg.PoseStamped()
        cylpose.pose.position.x = 0.65
        cylpose.pose.position.y = 0
        cylpose.pose.position.z = 0.769067
        cylpose.pose.orientation.w = 1.0
        self.scene.add_box("unit_cylinder", cylpose,size = (0.04,0.04,0.1))
        '''

    def go_to_pose_goal(self, x, y, z,  ox, oy, oz, ow):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:

        pose_goal = geometry_msgs.msg.Pose()
        #Quaternion = get_quaternion_from_euler(-2.48, -0.1, -0.6775759)
        # pose_goal.orientation.x = Quaternion[0]
        # pose_goal.orientation.y = Quaternion[1]
        # pose_goal.orientation.z = Quaternion[2]
        # pose_goal.orientation.w = Quaternion[3]
        pose_goal.orientation.x = ox #0.878599
        pose_goal.orientation.y = oy #0.352386
        pose_goal.orientation.z = oz #0.156799
        pose_goal.orientation.w = ow #-0.281607
        pose_goal.position.x = x #0.03
        pose_goal.position.y = y #0.1
        pose_goal.position.z = z #1.4
        
        move_group.set_pose_reference_frame("panda_link0")
        move_group.set_pose_target(pose_goal,"camera_link")

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        plan = move_group.plan()
        print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("Plan: ",plan[0])
        move_group.set_goal_position_tolerance(0.00001)
        success = move_group.go(wait=True)
        print("Success: ",success)
        print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
	
        
        
        
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose(end_effector_link = "camera_link").pose
        return [plan[0], success]#all_close(pose_goal, current_pose, 0.01)

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL
    def openGripper(self, posture):
        posture.joint_names = ["panda_finger_joint1","panda_finger_joint2"]       
        pts = trajectory_msgs.msg.JointTrajectoryPoint()
        pts.positions = [0.04,0.04]
        
        pts.time_from_start = rospy.Duration(10)
        posture.points = [pts]
    def closeGripper(self, posture):
        posture.joint_names = ["panda_finger_joint1","panda_finger_joint2"]       
        ptsc = trajectory_msgs.msg.JointTrajectoryPoint()
        ptsc.positions = [0.0005,0.0005]
        ptsc.velocities = [0.00001, 0.00001]
        #ptsc.accelerations = [0, 0]
        ptsc.time_from_start = rospy.Duration(10)
        posture.points = [ptsc]
        
    def pick(self, position,ox=None,oy= None,oz=None,ow=None): 
        #position= [x,y,z]
        #orientation = [ox,oy,oz,ow]
        grasp = moveit_msgs.msg.Grasp()
        grasp.grasp_pose.header.frame_id = "world"
        #q_orig = quaternion_from_euler(0, 0, 0)
        # Rotate the previous pose by 180* about X
        #q_rot = quaternion_from_euler(3.14159, 0, 0)
        #(ox,oy,oz,ow) = quaternion_multiply(q_rot, q_orig)
        grasp.grasp_pose.pose.orientation.x = ox
        grasp.grasp_pose.pose.orientation.y = oy
        grasp.grasp_pose.pose.orientation.z = oz
        grasp.grasp_pose.pose.orientation.w = ow
    
        grasp.grasp_pose.pose.position.x = position[0]
        grasp.grasp_pose.pose.position.y = position[1]
        grasp.grasp_pose.pose.position.z = position[2]

        grasp.pre_grasp_approach.direction.header.frame_id = "world"
        #/* Direction is set as negative z axis */
        grasp.pre_grasp_approach.direction.vector.z = -0.5 #-0.5
        grasp.pre_grasp_approach.min_distance = 0.04
        grasp.pre_grasp_approach.desired_distance = 0.225
        
        grasp.post_grasp_retreat.direction.header.frame_id = "world"
        #/* Direction is set as positive z axis */
        grasp.post_grasp_retreat.direction.vector.z = -1 #-0.5
        grasp.post_grasp_retreat.min_distance = 0.001
        grasp.post_grasp_retreat.desired_distance = 0.002
        
        self.openGripper(grasp.pre_grasp_posture)
        self.closeGripper(grasp.grasp_posture)
        
        self.move_group.set_support_surface_name("table")
        self.move_group.set_goal_joint_tolerance(0.05)
        #Call pick to pick up the object using the grasps given
        self.move_group.set_end_effector_link("panda_hand_tcp") #panda_leftfinger
        signal = self.move_group.pick("unit_cylinder", [grasp])
        #print(signal)
        return signal
        
    def attach_box(self, attach_srv):
        
        #self.box_name = "unit_cylinder"
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        
        req = AttachRequest()
        req.model_name_1 = "panda"
        req.link_name_1 = "panda_leftfinger"
        req.model_name_2 = "unit_cylinder"
        req.link_name_2 = "link_0"
        
        attach_srv.call(req)
        
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, req.model_name_2, touch_links=touch_links)

        
    def place(self, position):
    	place_location = moveit_msgs.msg.PlaceLocation()
    	place_location.place_pose.header.frame_id = "world"
    	
    	q_orig = quaternion_from_euler(0, 0, 0)
    	# Rotate the previous pose by 180* about X
    	q_rot = quaternion_from_euler(0, 0, 0)
    	(ox,oy,oz,ow) = quaternion_multiply(q_rot, q_orig)
    	place_location.place_pose.pose.orientation.x = ox
    	place_location.place_pose.pose.orientation.y = oy
    	place_location.place_pose.pose.orientation.z = oz
    	place_location.place_pose.pose.orientation.w = ow
    	
    	place_location.place_pose.pose.position.x = position[0]
    	place_location.place_pose.pose.position.y = position[1]
    	place_location.place_pose.pose.position.z = position[2]
    	
    	place_location.pre_place_approach.direction.header.frame_id = "world"
    	#/* Direction is set as negative z axis */
    	place_location.pre_place_approach.direction.vector.z = -0.5
    	place_location.pre_place_approach.min_distance = 0.04
    	place_location.pre_place_approach.desired_distance = 0.225
    	
    	place_location.post_place_retreat.direction.header.frame_id = "world"
    	#/* Direction is set as positive z axis */
    	place_location.post_place_retreat.direction.vector.z = -1
    	place_location.post_place_retreat.min_distance = 0.001
    	place_location.post_place_retreat.desired_distance = 0.002
    	
    	self.openGripper(place_location.post_place_posture)
    	
    	self.move_group.set_support_surface_name("table")
    	self.move_group.set_goal_joint_tolerance(0.05)
    	#Call pick to pick up the object using the grasps given
    	self.move_group.place("unit_cylinder", [place_location])
    
    def detach_object(self, detach_srv):
        scene = self.scene
        eef_link = self.eef_link
        
        req = AttachRequest()
        req.model_name_1 = "panda"
        req.link_name_1 = "panda_leftfinger"
        req.model_name_2 = "unit_cylinder"
        req.link_name_2 = "link_0"
        
        detach_srv.call(req)        
        
        scene.remove_attached_object(eef_link, name=req.model_name_2)
	
        
    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False     
    
    def retract(self, dist, scale=1):
        move_group = self.move_group
        move_group.set_pose_reference_frame("world")
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z +=  dist  # First move up (z)
        #wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.0001, 0.0,avoid_collisions = False 	) 
        return plan, fraction 
        
       
    def plan_cartesian_path(self,gx,gy, scale=1):
        
        move_group = self.move_group
        move_group.set_pose_reference_frame("world")

        
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z +=  0.05  # First move up (z)
        #wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x =  gx  # Second move forward/backwards in (x)
        wpose.position.y = gy  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -=  0.05  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.0001, 0.0,avoid_collisions = False 	) 		  # waypoints to follow  # eef_step
         
        return plan, fraction    
    
    def execute_plan(self, plan):
        
        move_group = self.move_group

        
        move_group.execute(plan, wait=True)

        

    def addCollisionObjects(self):
        scene = self.scene
        
        table = geometry_msgs.msg.PoseStamped()
        table.header.frame_id = self.planning_frame 
        table.pose.position.x =  0
        table.pose.position.y = 0
        table.pose.position.z = 0.365
        table.pose.orientation.z = 0
        scene.add_box("table", table,size = (0.8,1.58,0.72))
        
        wall = geometry_msgs.msg.PoseStamped()
        wall.header.frame_id = self.planning_frame 
        wall.pose.position.x =  -0.54
        wall.pose.position.y = 0
        wall.pose.position.z = 1.25
        wall.pose.orientation.z = 0
        scene.add_box("wall", wall,size = (0.1,2.5,2.5))
        
        cylpose = geometry_msgs.msg.PoseStamped() #0.210321, -0.210319
        cylpose.header.frame_id = self.planning_frame 
        cylpose.pose.position.x = object_position[0]
        cylpose.pose.position.y = object_position[1]
        cylpose.pose.position.z = 0.725+object_size[2]/2
        #-0.93604324, -0.00451694, -0.02485613,  0.35097694
        #cylpose.pose.orientation.x = 0#-0.93604324
        #cylpose.pose.orientation.y = 0#-0.00451694
        #cylpose.pose.orientation.z = 0#-0.02485613
        #cylpose.pose.orientation.w = 1#0.35097694
        scene.add_box("unit_cylinder", cylpose,size = (object_size[0],object_size[1],object_size[2]))
        
        print("objetos", scene.get_known_object_names())
        
        
def main():
    global object_size
    object_size = [0.08,0.08,0.1]
    global object_position
    object_position = [0.649997, 0.000003]
    tutorial = MoveGroupPythonInterfaceTutorial()
    
    tutorial.addCollisionObjects()
    
    
    vsUpdatePub = rospy.Publisher('/moveVS', std_msgs.msg.Float32MultiArray, queue_size=10)
    vsUpdateMsg = std_msgs.msg.Float32MultiArray()
    dimension = std_msgs.msg.MultiArrayDimension()
    dimension.label = "moveVS"
    dimension.stride = 1
    dimension.size = 5 #actualizar(1 o 0), x,y,z, rotacionejez
    vsUpdateMsg.layout.dim = [dimension]
    vsUpdateMsg.data = [0, 0, 0, 0, 0]
 
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    attach_srv.wait_for_service()
    
    #tutorial.detach_object(detach_srv)
    
    update=0
    
    vsTargets = getTFPose()
    #exito = tutorial.go_to_pose_goal(0.03, 0.1, 1.4,0.878599, 0.352386,0.156799, -0.281607)
    #print("Exito: ", exito)
    #tutorial.go_to_pose_goal(0.02, 0.2, 1.3,0.878599, 0.352386,0.156799, -0.281607)
    
    br = tf2_ros.TransformBroadcaster()
    
    '''
    
    
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.frame_id = "world"
    t2.child_frame_id = "lowBoundX"
    t2.transform.translation.x = 0.649997 - 0.15/2
    t2.transform.translation.y = 0
    t2.transform.translation.z = 0
    t2.transform.rotation.w = 1
    
    
    '''
    vsMask= np.zeros(32)
    regPCD = o3d.geometry.PointCloud()
    
    rospy.sleep(10.)
    gazObjStates = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=20)
    objNames = np.array(gazObjStates.name)
    cylIndex = np.argwhere( objNames=="unit_cylinder" )
    cylPose = np.array(gazObjStates.pose)[cylIndex]
    initObjPose = pos2HTM(cylPose[0][0])
    mug = geometry_msgs.msg.TransformStamped()
    mug.header.frame_id = "world"
    
    mug.child_frame_id = "Mug"
    mug.header.stamp = rospy.get_rostime()
    mug.transform.translation.x = cylPose[0][0].position.x
    mug.transform.translation.y = cylPose[0][0].position.y
    mug.transform.translation.z = cylPose[0][0].position.z
    mug.transform.rotation.x = cylPose[0][0].orientation.x
    mug.transform.rotation.y = cylPose[0][0].orientation.y
    mug.transform.rotation.z = cylPose[0][0].orientation.z
    mug.transform.rotation.w = cylPose[0][0].orientation.w
    np.save("Tw_m.npy", initObjPose)
    print(initObjPose)
    initial_angle = np.arctan2(initObjPose[1,0], initObjPose[0,0]) 
    br.sendTransform(mug)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    Q0_i = tfBuffer.lookup_transform('world', 'tf_d0', rospy.Time(0), rospy.Duration(10.0))
    Tw_0=np.linalg.inv(pose2HTM(Q0_i.transform))
    
    #Tw_f0 = pose2HTM(Q0_i.transform)
    #Tw_f0[]
    np.save("Tw_0.npy", pose2HTM(Q0_i.transform))
    #o3d.io.write_point_cloud('PCD'+str(i)+".pcd", pose2HTM(pose)) 
    

    initObjPose[2,3]=initObjPose[2,3]#-(0.725)
    lCentroz =  0.09 - initObjPose[2,3] 
    #Td_m = np.matmul(transformation,np.matmul(Tc_w,initObjPose))
    Td_m = np.matmul(Tw_0,initObjPose)
    np.save("Td_bowl.npy", Td_m)

    
    Q0_5 = tfBuffer.lookup_transform('tf_d0', 'tf_d13', rospy.Time(0), rospy.Duration(10.0))
    T5_0=np.linalg.inv(pose2HTM(Q0_5.transform))
    np.save("T13_0.npy", T5_0)

    Q0_14 = tfBuffer.lookup_transform('tf_d0', 'tf_d22', rospy.Time(0), rospy.Duration(10.0))
    T14_0=np.linalg.inv(pose2HTM(Q0_14.transform))
    np.save("T22_0.npy", T14_0)



    #time.sleep(10)
    
    #trial
    #trial
    '''
    pick_pos = [0.2395916392976865, 0.0899494680479472,0.876208186521]#[graspFinal[0,3],graspFinal[1,3],graspFinal[2,3]]#0.904
    tutorial.pick(pick_pos, -0.021042333558650338,0.9979761122538647,-0.0594484716979036, -0.00817182855006355)#graspTestRotation[0],graspTestRotation[1],graspTestRotation[2],graspTestRotation[3])
    #print([graspFinal[0,3],graspFinal[1,3],graspFinal[2,3],graspTestRotation[0],graspTestRotation[1],graspTestRotation[2],graspTestRotation[3]])
    tutorial.attach_box(attach_srv)
    d = 0.38#-0.33
    direction = np.array([initObjPose[0,3],initObjPose[1,3]])
    norm = np.linalg.norm(direction)
    xyPlace = np.array([0.2395916392976865, 0.0899494680479472]) + d*(direction/norm)
    place_pos = [xyPlace[0],xyPlace[1],0.8]#0.8#graspFinal[2,3]]#0.909 muy arriba con 0.909, manana intentar con 0.5 (entre 0.5 y 0.909)
    print(place_pos)
    cartesian_plan, fraction = tutorial.plan_cartesian_path(place_pos[0],place_pos[1])
    tutorial.execute_plan(cartesian_plan)
    #tutorial.place(place_pos)
    rospy.sleep(10)
    
    tutorial.detach_object(detach_srv)
    rospy.sleep(20)
    tutorial.go_to_pose_goal( 0.35745, 0.028111, 0.532,  -0.00066075, 0.71554, -0.0004089, 0.698572)
    '''
    '''
    pick_pos = [0.4883865335922,0.35754365851,0.93152570216961]#0.904
    tutorial.pick(pick_pos, 0.496877926,0.8659648757,-0.00495684517, 0.056503003547)
    #print([graspFinal[0,3],graspFinal[1,3],graspFinal[2,3],graspTestRotation[0],graspTestRotation[1],graspTestRotation[2],graspTestRotation[3]])
    tutorial.attach_box(attach_srv)
    d = -0.33#-0.33
    direction = np.array([initObjPose[0,3],initObjPose[1,3]])
    norm = np.linalg.norm(direction)
    xyPlace = np.array([0.4883865335922,0.35754365851]) + d*(direction/norm)
    place_pos = [xyPlace[0],xyPlace[1],0.8]#0.8#graspFinal[2,3]]#0.909 muy arriba con 0.909, manana intentar con 0.5 (entre 0.5 y 0.909)
    print(place_pos)
    rospy.sleep(5)
    tutorial.place(place_pos)
    rospy.sleep(10)
    tutorial.detach_object(detach_srv)
    rospy.sleep(20)
    tutorial.go_to_pose_goal( 0.35745, 0.028111, 0.532,  -0.00066075, 0.71554, -0.0004089, 0.698572)
    '''
    #'''
    #trial
    tfBuffer1 = tf2_ros.Buffer()
    listener1 = tf2_ros.TransformListener(tfBuffer1)
    Tw_0 = pose2HTM( (tfBuffer1.lookup_transform('panda_link0', "tf_d0", rospy.Time(0), rospy.Duration(10.0))).transform )
    #print("W0000000000:",Tw_0)
    #rospy.sleep(5)
    Qw_c = 0
    for i in range(len(vsTargets)):
     #br.sendTransform(bestGrasp)
      
     #br.sendTransform([t,t2,t3,t4,t5,t6])
     vsUpdatePub.publish(vsUpdateMsg)
     if (update==1):
       rospy.sleep(2.)
       vsTargets = getTFpose()
       update = 0
     
     print(vsTargets[i])
     rec = rectification(vsTargets[i].orientation.x, vsTargets[i].orientation.y, vsTargets[i].orientation.z, vsTargets[i].orientation.w) # rectification to align camera x axis with targets z axis
     print(rec)
     
     vsTargets[i].orientation.x = rec[0]
     vsTargets[i].orientation.y = rec[1]
     vsTargets[i].orientation.z = rec[2]
     vsTargets[i].orientation.w = rec[3]
     print(vsTargets[i])
     print("------------")
     
     if vsTargets[i].position.x>0.15:
      exito = tutorial.go_to_pose_goal(vsTargets[i].position.x, vsTargets[i].position.y, vsTargets[i].position.z, vsTargets[i].orientation.x, vsTargets[i].orientation.y,vsTargets[i].orientation.z, vsTargets[i].orientation.w)
     else:
      exito =[False, False]  
      
     if exito[0] == True and exito[1] == True and vsTargets[i].position.x>0.15:
      #Seguir con el proceso
      rospy.sleep(5) 
      print("Exito: ", exito)
      color_topic = "/camera/color/image_raw"
      colorImage = rospy.wait_for_message(color_topic, Image, timeout=20)
      color_callback(colorImage)
      depth_topic = "/camera/aligned_depth_to_color/image_raw"
      depthImage = rospy.wait_for_message(depth_topic, Image, timeout=20)
      depth_callback(depthImage)
      cam_topic = "/camera/aligned_depth_to_color/camera_info"
      camInfo = rospy.wait_for_message(cam_topic, CameraInfo, timeout=20)
      currPCD = camera_info_callback(camInfo,i)
      print("hola1")

      '''
      bestGrasp=GPD()    
      print("bestGPlist:", bestGPlist)
      #rospy.sleep(5) 
      br.sendTransform(bestGrasp)
      '''

      #T0_i = getVSTF(i)
      if i == 0:
        transformation = getVSTF(i)
        regPCD = draw_registration_result(currPCD, regPCD, transformation,Tw_0)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        Q0_i = tfBuffer.lookup_transform('world', 'camera_depth_optical_frame', rospy.Time(0), rospy.Duration(10.0))
        Qw_c = Q0_i
        Tc_w=np.linalg.inv(pose2HTM(Q0_i.transform))
        x_pos = Q0_i.transform.translation.x
        y_pos = Q0_i.transform.translation.y
        z_pos = Q0_i.transform.translation.z  
        orientation = Q0_i.transform.rotation
        print("Orientatiooooooooon========= ",orientation)
        T0_c= pose2HTM((tfBuffer.lookup_transform('tf_d0', 'camera_depth_optical_frame', rospy.Time(0), rospy.Duration(10.0))).transform)
        
        gazObjStates = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=20)
        objNames = np.array(gazObjStates.name)
        cylIndex = np.argwhere( objNames=="unit_cylinder" )
        cylPose = np.array(gazObjStates.pose)[cylIndex]
        Tw_obj = pos2HTM(cylPose[0][0])

        print('HTM==== ',pose2HTM(Q0_i.transform))
        #T0_i = vs2O3D(T0_i)
        np.save("Tw_d.npy", pose2HTM(Q0_i.transform))
        #o3d.io.write_point_cloud('PCD'+str(i)+".pcd", pose2HTM(pose)) 
        mug.header.stamp = rospy.get_rostime()
        #br.sendTransform(mug)
        #tfBuffer = tf2_ros.Buffer()
        #listener = tf2_ros.TransformListener(tfBuffer)
        #Q0_i = tfBuffer1.lookup_transform('world', 'Mug', rospy.Time(0), rospy.Duration(10.0))
        #Q0_i.transform.translation.z -=0.725
        
        #if initObjPose[2,3]<0.71:
        #Tw_obj = initObjPose#[2,3]#=initObjPose[2,3]-0.09#-(0.725/2)
        print(Tw_obj[2,3])
        #Tw_obj[2,3] =Tw_obj[2,3]-0.19
        print(Tw_obj[2,3])
        Td_l = np.matmul(T0_c,np.matmul(Tc_w,Tw_obj))
        np.save("Td_l.npy", Td_l)
      else:
        transformation = getVSTF(i)
        print("hola2")
        threshold = 0.02/100000
        #reg_p2p = o3d.pipelines.registration.registration_icp(currPCD, regPCD, threshold, transformation, o3d.pipelines.registration.TransformationEstimationPointToPoint(),  
        #o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        #print("hola3")
        #print("W0000000001:",Tw_0)
        #rospy.sleep(5)
        regPCD = draw_registration_result(currPCD, regPCD, transformation,Tw_0)
        print("hola4")
      vsMask[i]=1
      # Get a tf from the initial frame to the current frame
      
      #np.save("T0_"+str(i), T0_i)
      #o3d.io.write_point_cloud('PCD'+str(i)+".pcd", currPCD) 
     elif exito[0] == True and exito[1] == False:
      #Control failed / Retry 
      print("Retry")
     elif exito[0] == False and exito[1] == False:
      #Activate viewspace mask to be considered when the hemisphere is moved 
      print("Activar/Desactivar digitos de la Mascara")
      #vsMask[i]=0
      
    print("Mask: ", vsMask)
    print(np.argwhere(vsMask == 0))
    numViewsOne = np.count_nonzero(vsMask == 1)
    o3d.io.write_point_cloud("/home/josepatino/ros/noetic/system/src/franka_vp/src/PCDregNoFinal/cil.pcd",regPCD)

    #from here we begin to process pcd 2 grasp
    
    #to here

    
    #graspFinal = getBestTTGrasp(np.array(bestGPlist),np.array(bestSClist))  
    #print("bestGPlist:", graspFinal)
    #rospy.sleep(5) 
    
   
    
    #rospy.sleep(5)
    '''
    graspTestRotation = R.from_matrix(graspFinal[:3,:3]).as_quat()
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.frame_id = "world"
    t2.header.stamp = rospy.get_rostime()
    t2.child_frame_id = "graspTest"
    t2.transform.translation.x = graspFinal[0,3]
    t2.transform.translation.y = graspFinal[1,3]
    t2.transform.translation.z = graspFinal[2,3]
    t2.transform.rotation.x = graspTestRotation[0]
    t2.transform.rotation.y = graspTestRotation[1]
    t2.transform.rotation.z = graspTestRotation[2]
    t2.transform.rotation.w = graspTestRotation[3]
    br.sendTransform(t2)
    '''


    tutorial.go_to_pose_goal( 0.35745, 0.028111, 0.532,  -0.00066075, 0.71554, -0.0004089, 0.698572)
    
 


    #################################################################################################################################################
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.00001) 
    #regPCD = regPCD.voxel_down_sample(voxel_size=0.00001) 
    o3d.io.write_point_cloud("/home/josepatino/ros/noetic/system/src/franka_vp/src/pcdFinal"+".pcd", regPCD)
    regPCD = outlierRemoval(regPCD,1,"total")
    o3d.visualization.draw_geometries([regPCD,origin],point_show_normal=True,zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])
    
    #Poisson Algorithm
    print("Poisson Algorithm")
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(regPCD, depth=8)
    o3d.visualization.draw_geometries([mesh,origin],zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])
    mesh.compute_triangle_normals()
    o3d.io.write_triangle_mesh("/home/josepatino/ros/noetic/system/src/franka_vp/src/Comparison/Poisson.stl", mesh)
    


if __name__ == "__main__":
    main()
