#!/usr/bin/env python3

# ROS Imports
import rospy
import tf2_ros
import geometry_msgs.msg
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, CameraInfo

# General Imports
import numpy as np
import cv2
import open3d as o3d
import copy

# Own Imports
from franka_move import MoveGroupPyInterface

class ReconstructionSystem:

    def __init__(self, vis=False):

        # Bridge to convert ROS image message to cv2 format
        self.bridge = CvBridge()
        self.visualisations = vis

    # ------------------------- Callback functions -------------------------

    def color_callback(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def depth_callback(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, "32FC1")

    # ------------------------- Getting Poses -------------------------

    # Rectification to align camera x axis with targets z axis
    def rectification(self, x, y, z, w): 

        r = R.from_quat([x, y, z, w])
        ry = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
        rz = R.from_rotvec(np.pi * np.array([0, 0, 1]))
        Rtot = r*ry*rz
        return Rtot.as_quat()

    def getTFPose(self, view_poses):

        # View Poses: a list of integers, representing 32 possible poses

        vsVector =[]

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        target_pose = geometry_msgs.msg.TransformStamped()

        for i in view_poses:
            frame = 'tf_d'+str(i)
            transformStamped = tfBuffer.lookup_transform('panda_link0', frame, rospy.Time(0), rospy.Duration(10.0))
            target_pose = geometry_msgs.msg.Pose()
            
            target_pose.position.x = transformStamped.transform.translation.x
            target_pose.position.y = transformStamped.transform.translation.y
            target_pose.position.z = transformStamped.transform.translation.z
            target_pose.orientation = transformStamped.transform.rotation
            vsVector.append(target_pose)

            print("Saving: ", str(i))
        return vsVector  

    def move_to_coords(self, vsTargets, move):

        rec = self.rectification(vsTargets.orientation.x, vsTargets.orientation.y, vsTargets.orientation.z, vsTargets.orientation.w)
        vsTargets.orientation.x = rec[0]
        vsTargets.orientation.y = rec[1]
        vsTargets.orientation.z = rec[2]
        vsTargets.orientation.w = rec[3]

        move.go_to_pose_goal(vsTargets.position.x, vsTargets.position.y, vsTargets.position.z, vsTargets.orientation.x, vsTargets.orientation.y,vsTargets.orientation.z, vsTargets.orientation.w)

        print("-------- Sleeping --------")

        rospy.sleep(0.5)

    # ------------------------- 3D Stuff -------------------------

    def outlierRemoval(self, pcd, securityFactor, mode):
        points = np.asarray(pcd.points)
        media = np.mean(np.asarray(points), axis=0)

        q75, q25 = np.percentile( points, [75 ,25], axis=0)
        iqr = q75 - q25
        upBound = q75 + securityFactor*iqr
        lowBound = q25 - securityFactor*iqr
        
        if mode=="z":
            pcl = o3d.geometry.PointCloud()
            bounds = np.array( [[0,0,upBound[2]], [0,0,lowBound[2]] ] )
            pcl.points = o3d.utility.Vector3dVector( bounds )
            boundColors = np.array( [[0,0,1], [1,0,0] ] )
            pcl.colors = o3d.utility.Vector3dVector( boundColors )
            index = np.argwhere(np.asarray(pcd.points)[:,2]<upBound[2])
            final = pcd.select_by_index(index)
            return final
        
        if mode=="total":
            pcl = o3d.geometry.PointCloud()
            bounds = np.array( [[0,0,upBound[2]], [0,0,lowBound[2]] ] )
            pcl.points = o3d.utility.Vector3dVector( bounds )
            boundColors = np.array( [[0,0,1], [1,0,0] ] )
            pcl.colors = o3d.utility.Vector3dVector( boundColors )
            index = np.argwhere((np.asarray(pcd.points)[:,2]>lowBound[2]) * (np.asarray(pcd.points)[:,2]<upBound[2]) * (np.asarray(pcd.points)[:,0]>lowBound[0]) * (np.asarray(pcd.points)[:,0]<upBound[0]))
            final = pcd.select_by_index(index)
            return final

    def RANSAC(self, pcd, dt, rn, ni):
        plane_model, inliers = pcd.segment_plane(distance_threshold=dt, ransac_n=rn,num_iterations=ni)
        [a, b, c, d] = plane_model
        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        return (inlier_cloud, outlier_cloud)

    def removeOutBBX(self, pcd, center, size, transformation):
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

        centerTr = np.matmul(T0_w, center2) 
        centerTr = np.array([centerTr[0][0],centerTr[1][0],centerTr[2][0]])
        finalPoints = np.array(finalPoints)

        ux = (finalPoints[0] - finalPoints[1])[np.newaxis].T
        ux = ux/np.linalg.norm(ux)
        uy = (finalPoints[2] - finalPoints[3])[np.newaxis].T
        uy = uy/np.linalg.norm(uy)
        uz  = (finalPoints[4] - finalPoints[5])[np.newaxis].T
        uz = uz/np.linalg.norm(uz)

        R = np.hstack( [np.hstack([ux,uy]),uz] )
        bbx=o3d.geometry.OrientedBoundingBox(centerTr, R, size+0.00003)
        indexs = bbx.get_point_indices_within_bounding_box(pcd.points)
        pcd = pcd.select_by_index(indexs)

        if self.visualisations:
            o3d.visualization.draw_geometries([pcd,bbx])#,zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])

        return pcd

    def pose2HTM(self, pose): #transform stamped to HTM
        p = np.array([ [pose.translation.x], [pose.translation.y], [pose.translation.z] ])
        quat = pose.rotation
        rotM = R.from_quat(np.array([quat.x, quat.y, quat.z, quat.w])).as_matrix()
        HTM = np.hstack([rotM, p])
        HTM = np.vstack([HTM, np.array([[0, 0, 0, 1]])])
        return HTM

    def getVSTF(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        Q0_i = tfBuffer.lookup_transform('tf_d0', 'camera_depth_optical_frame', rospy.Time(0), rospy.Duration(10.0))
        x_pos = Q0_i.transform.translation.x
        y_pos = Q0_i.transform.translation.y
        z_pos = Q0_i.transform.translation.z  
        orientation = Q0_i.transform.rotation
        
        R0_i = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
        t0_i = np.array([[x_pos],[y_pos],[z_pos]])/1000
        T0_i = np.hstack([R0_i, t0_i])
        T0_i = np.vstack([T0_i, np.array([[0,0,0,1]])])

        return T0_i

    def draw_registration_result(self, source, target, transformation, Tw_0):
        object_size = [0.08,0.08,0.1]
        object_position = [0.649997, 0]

        # object_size = [0.1,0.1,0.1]
        # object_position = [0.705265, 0.002895]

        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.transform(transformation)
        pcdFin = source_temp + target_temp
        Tw_0cop =copy.deepcopy(Tw_0)
        ocz = (object_size[2]/2)#(0.725+(object_size[2]/2)) - ((0.725+(object_size[2]/2))-(object_size[2]/2)) #object center in z
        pcdFin = self.removeOutBBX(pcdFin, np.array([object_position[0], object_position[1], ocz]), np.array([object_size[0],object_size[1],object_size[2]]), Tw_0cop)
        return pcdFin    

    def cam_info_callback(self, color_img, depthimg, cam_info_msg):
        cam_info = cam_info_msg.K
        
        di = o3d.geometry.Image( np.array(depthimg)/1000 )
        ci = o3d.geometry.Image( ((np.array(color_img)[:, ::-1])) )

        intrinsic = o3d.camera.PinholeCameraIntrinsic(cam_info_msg.width, cam_info_msg.height, cam_info[0], cam_info[4], cam_info[2], cam_info[5])
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(ci, di, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

        ic, oc = self.RANSAC(pcd,0.000002,200,1000)#0.000005,200,1000

        if len(np.asarray(oc.points))>66200:
            ic2, oc2 = self.RANSAC(oc,0.000005,50,1000)#0.0000008,50,1000
            if len(np.asarray(ic2.points))>23500:
                ic = ic2
                oc = oc2

        final = self.outlierRemoval(oc,10,"z")
        final.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.0001, max_nn=30))
        final.orient_normals_towards_camera_location()
        final = final.voxel_down_sample(voxel_size=0.000005) #0.00001
        return final

    def runner(self, view_poses):
        vsTargets = self.getTFPose(view_poses)

        print(vsTargets)

        move = MoveGroupPyInterface()
        move.addCollisionObjects()

        regPCD = o3d.geometry.PointCloud()

        for i in range(len(vsTargets)):
            print("-------------- Moving pos", str(i), " --------------")

            self.move_to_coords(vsTargets[i], move)

            color_topic = "/camera/color/image_raw"
            depth_topic = "/camera/aligned_depth_to_color/image_raw"
            cam_topic = "/camera/aligned_depth_to_color/camera_info"

            colorImage = rospy.wait_for_message(color_topic, Image, timeout=20)
            depthImage = rospy.wait_for_message(depth_topic, Image, timeout=20)
            camInfo = rospy.wait_for_message(cam_topic, CameraInfo, timeout=20)

            colour_img = self.color_callback(colorImage)
            depth_img = self.depth_callback(depthImage)
            currPCD = self.cam_info_callback(colour_img, depth_img, camInfo)

            colour_name = "/home/ivokosa/Desktop/3D_Mesh/colour_" + str(i) + ".png"
            depth_name = "/home/ivokosa/Desktop/3D_Mesh/depth_" + str(i) + ".png"

            cv2.imwrite(colour_name, colour_img)
            cv2.imwrite(depth_name, depth_img)

            # -------------- -------------- -------------- --------------

            tfBuffer1 = tf2_ros.Buffer()
            listener1 = tf2_ros.TransformListener(tfBuffer1)
            Tw_0 = self.pose2HTM( (tfBuffer1.lookup_transform('panda_link0', "tf_d0", rospy.Time(0), rospy.Duration(10.0))).transform )
            
            transformation = self.getVSTF()
            regPCD = self.draw_registration_result(currPCD, regPCD, transformation, Tw_0)
        
        print("-------------- Finished Loop --------------")

        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.00001) 
        # regPCD = regPCD.voxel_down_sample(voxel_size=0.00001) 
        o3d.io.write_point_cloud("/home/ivokosa/Desktop/3D_Mesh/pcdFinal.ply", regPCD)
        regPCD = self.outlierRemoval(regPCD,1,"total")
        if self.visualisations:
            o3d.visualization.draw_geometries([regPCD,origin],point_show_normal=True,zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])
        
        # Poisson Algorithm
        # print("Poisson Algorithm")
        # mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(regPCD, depth=8)
        # if self.visualisations:
        #     o3d.visualization.draw_geometries([mesh,origin],zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])
        # mesh.compute_triangle_normals()

        return regPCD

        # o3d.io.write_triangle_mesh("/home/ivokosa/Desktop/3D_Mesh/tstmesh.obj", mesh)

# ------------------------- Main -------------------------
        
if __name__ == '__main__':

    rospy.init_node("Reconstruction_System", anonymous=True)

    view_nums = [7, 9, 21] 

    reconstruct = ReconstructionSystem()
    r = reconstruct.runner(view_nums)