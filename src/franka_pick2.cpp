
//STD STREAM
#include <iostream>
// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_model/joint_model_group.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

//Standard
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"


//Pose
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

// Gazebo Attacher
#ifdef GAZEBO_LINK_ATTACHER
#include <gazebo_ros_link_attacher/Attach.h>
#endif

// Services
#ifdef GAZEBO_LINK_ATTACHER
    gazebo_ros_link_attacher::Attach attach_srv_;
    ros::ServiceClient attach_client_, detach_client_;
#endif

//Include View Space Class
#include "View_Space.hpp"
//include vector
#include <vector>
//include cmath
#include <cmath>

#include <string>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
string exito;


void attach1(moveit::planning_interface::MoveGroupInterface& move_group_atc)
{

  // Now, let's attach the collision object to the robot.
  move_group_atc.attachObject("target_box", "panda_leftfinger",
                                  {"panda_rightfinger", "panda_leftfinger",
                                   "panda_hand"
                                  "link"});
#ifdef GAZEBO_LINK_ATTACHER
  attach_srv_.request.model_name_1 = "target_box";
  attach_srv_.request.link_name_1 = "link";

  if (attach_client_.call(attach_srv_))
    ROS_INFO("Successfully attached");
  else
    ROS_WARN("NOT attached");
#endif
}


void detach1(moveit::planning_interface::MoveGroupInterface& move_group_atc)
{
  // Now, let's detach the collision object from the robot.
  ROS_INFO_NAMED("tutorial", "Detach the object to the robot");
  move_group_atc.detachObject("target_box");
  
#ifdef GAZEBO_LINK_ATTACHER
  if (detach_client_.call(attach_srv_))
      ROS_INFO("Successfully detached");
  else
      ROS_WARN("NOT detached");
#endif
}



void openGripper(trajectory_msgs::JointTrajectory &posture)
{

    /* Add both finger joints of panda robot. */
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(2);

}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{

    /* Add both finger joints of panda robot. */

    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(2);

    

}

void rotationConstraint(moveit::planning_interface::MoveGroupInterface &move_group)
{
   moveit_msgs::OrientationConstraint ocm;
   ocm.link_name = "panda_link7";
   ocm.header.frame_id = "panda_link0";
   
   tf2::Quaternion quat;
   quat.setRPY(0, M_PI, 0);
   ocm.orientation = tf2::toMsg(quat);
   //ocm.orientation.w = 1;
   ocm.absolute_x_axis_tolerance = 0.1;
   ocm.absolute_y_axis_tolerance = 0.1;
   ocm.absolute_z_axis_tolerance = 0.1;
   ocm.weight = 1.0;
   
   moveit_msgs::Constraints test_constraints;
   test_constraints.orientation_constraints.push_back(ocm);
   move_group.setPathConstraints(test_constraints);
}

std::vector< geometry_msgs::Pose > getTFpose(){
  int views_size=32;
  // debe retornar: const std::vector< geometry_msgs::Pose >
  std::vector< geometry_msgs::Pose > vsVector; //View SPace Vector
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  vsVector.clear();
  for (int i = 0; i < views_size; i++) {
    
    transformStamped = tfBuffer.lookupTransform("panda_link0", "tf_d"+std::to_string(i),ros::Time(0),ros::Duration(5.0));
    geometry_msgs::Pose target_pose;
    
    //target_pose.header.stamp = ros::Time(0);
    //target_pose.header.frame_id = "tf_d"+std::to_string(i);
    target_pose.position.x = transformStamped.transform.translation.x;
    target_pose.position.y = transformStamped.transform.translation.y;
    target_pose.position.z = transformStamped.transform.translation.z;  
    target_pose.orientation = transformStamped.transform.rotation;
    
    
    
    vsVector.push_back(target_pose);
    
  }
  //std::cout<< vsVector.size()<<std::endl;
  
  return vsVector;

}

void sendTF(View_Space vs, tf2_ros::TransformBroadcaster tfb){
  vector<geometry_msgs::TransformStamped> tS = vs.views2tStamped();
  tfb.sendTransform(tS);
  
}

void getStatus(const moveit_msgs::MoveGroupActionFeedback msg)
{   
    exito = msg.feedback.state;
    //tf_frame = msg.header.frame_id;
}





bool control_failed(geometry_msgs::Pose vsTargets,geometry_msgs::TransformStamped ee_link0){
  float x_error= abs(vsTargets.position.x - ee_link0.transform.translation.x) ;
  float y_error= abs(vsTargets.position.y - ee_link0.transform.translation.y);
  float z_error= abs(vsTargets.position.z - ee_link0.transform.translation.z) ;
  float xor_error= abs( abs(vsTargets.orientation.x) - abs(ee_link0.transform.rotation.x) ) ;
  float yor_error= abs( abs(vsTargets.orientation.y) - abs(ee_link0.transform.rotation.y) ) ;
  float zor_error= abs( abs(vsTargets.orientation.z) - abs(ee_link0.transform.rotation.z) ) ;
  float wor_error= abs( abs(vsTargets.orientation.w) - abs(ee_link0.transform.rotation.w) ) ;
  
  std::cout<< "x_error"<< vsTargets.position.x <<" - "<< ee_link0.transform.translation.x<< endl ;
  std::cout<< "y_error"<< vsTargets.position.y <<" - "<< ee_link0.transform.translation.y << endl;
  std::cout<< "z_error"<< vsTargets.position.z <<" - "<< ee_link0.transform.translation.z << endl;
  std::cout<< "xor_error"<< vsTargets.orientation.x <<" - "<< ee_link0.transform.rotation.x << endl;
  std::cout<< "yor_error"<< vsTargets.orientation.y <<" - "<< ee_link0.transform.rotation.y << endl;
  std::cout<< "zor_error"<< vsTargets.orientation.z <<" - "<< ee_link0.transform.rotation.z << endl;
  std::cout<< "wor_error"<< vsTargets.orientation.w <<" - "<< ee_link0.transform.rotation.w << endl;
  
  float err_tres=0.01;
  bool fail=0;
  if ((x_error > err_tres) or (y_error > err_tres) or (z_error > err_tres) or (xor_error > err_tres) or (yor_error > err_tres) or (zor_error > err_tres) or (wor_error > err_tres)){
   fail=1;
  }
  else{
   fail=0;
  }
  return fail;
 
}

/*
std::vector<float> normalize_quat(float x1,float y1,float z1,float w1){
  
  
  return normalized_quat;
}
*/

std::vector<float> normalize(float x1,float y1,float z1,float w1){
  float norma = sqrt( pow(x1,2) + pow(y1,2) + pow(z1,2) + pow(w1,2) );
  std::vector<float> quat = {x1/norma, y1/norma, z1/norma, w1/norma};
  return quat;
}

std::vector<float> quat_mult(float x1,float y1,float z1,float w1,float x2,float y2,float z2,float w2){
  float x_res = w1*x2 + x1*w2 + y1*z2 - z1*y2;
  float y_res = w1*y2 - x1*z2 + y1*w2 + z1*x2;
  float z_res = w1*z2 + x1*y2 - y1*x2 + z1*x2;
  float w_res = w1*w2 - x1*x2 - y1*y2 - z1*z2;
  std::vector<float> quat = {x_res, y_res, z_res, w_res};
  return quat;
}


Eigen::Quaterniond rectification(float x1,float y1,float z1,float w1){
   /*
   float x2 =0 ,y2 = 0 ,z2 = 1,w2 = M_PI;
   float x3 =0 ,y3 = 1 ,z3 = 0,w3 = M_PI/2;
   std::vector<float> x2_norm = normalize(x2,y2,z2,w2);
   std::vector<float> x3_norm = normalize(x3,y3,z3,w3);
   std::vector<float> quat = quat_mult( x1,y1,z1, w1, x3_norm[0],x3_norm[1],x3_norm[2],x3_norm[3] );
   quat = quat_mult( quat[0],quat[1],quat[2], quat[3], x2_norm[0],x2_norm[1],x2_norm[2],x2_norm[3] );
   return quat;
   */
   Eigen::Quaterniond q;
   q.x() = x1;
   q.y() = y1;
   q.z() = z1;
   q.w() = w1;
   Eigen::Matrix3d R = q.toRotationMatrix();
   R = R*Eigen::AngleAxis<double>( (M_PI/2) , Eigen::Vector3d::UnitY())*Eigen::AngleAxis<double>( (M_PI) , Eigen::Vector3d::UnitZ());
   
   Eigen::Quaterniond qf(R);
   return qf;
   
   
   
}


int move(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose target)
{

    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    //std::vector<moveit_msgs::Grasp> pre_grasps;
    //pre_grasps.resize(1);
    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener(tfBuffer);
    
    
    //geometry_msgs::TransformStamped transformStamped;
    //transformStamped = tfBuffer.lookupTransform("panda_link0", "tf_d",ros::Time(0),ros::Duration(3.0));

    
    
    //geometry_msgs::Pose target_pose1;
    //tf2::Quaternion orientation;
    //orientation.setRPY(M_PI, M_PI/2 , M_PI/2);
    
    //target_pose1.orientation.w = -1;
    /*
    target_pose1.position.x = 0.1;
    target_pose1.position.y = 0.1;
    target_pose1.position.z = 0.5;
    target_pose1.orientation = tf2::toMsg(orientation);
    */
    /*
    target_pose1.position.x = transformStamped.transform.translation.x;
    target_pose1.position.y = transformStamped.transform.translation.y;
    target_pose1.position.z = transformStamped.transform.translation.z;  
    target_pose1.orientation = transformStamped.transform.rotation;
    
    std::cout<< target_pose1.position.x<<std::endl;
    std::cout<< target_pose1.position.y<<std::endl;
    std::cout<< target_pose1.position.z<<std::endl;
    */
    
    /*
    std::vector< float > rec = rectification(target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w);
    target.orientation.x = rec[0];
    target.orientation.y = rec[1];
    target.orientation.z = rec[2];
    target.orientation.w = rec[3];
    */
    
    
    //std::vector< geometry_msgs::Pose > vsTargets = getTFpose();
    
    move_group.setPoseReferenceFrame("panda_link0"); 
    //move_group.setEndEffector("panda_EE");
    
    
    move_group.setPoseTarget(target, "camera_link");
    
    
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode success = move_group.plan(my_plan);//move_group.plan(my_plan);//(move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    int ex = success.val;
    cout<<"Exito: "<<ex<<endl;
    
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    
    /*
    const moveit::core::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup("panda_arm");
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;

    moveit::core::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);

    move_group.setPoseTarget(target_pose1);
    
    move_group.setPlanningTime(10.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
    */

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    /*
    pre_grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, M_PI, 0);
    pre_grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    pre_grasps[0].grasp_pose.pose.position.x = 0.5;
    pre_grasps[0].grasp_pose.pose.position.y = 0.1;
    pre_grasps[0].grasp_pose.pose.position.z = 0.8;
    */
    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    //pre_grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    //pre_grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    //pre_grasps[0].pre_grasp_approach.min_distance = 0.04;
    //pre_grasps[0].pre_grasp_approach.desired_distance = 0.05;


    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    //openGripper(pre_grasps[0].pre_grasp_posture);



    // Set support surface as table.
    move_group.setSupportSurfaceName("table");
    move_group.setGoalJointTolerance(0.1);
    move_group.move();
    //move_group.execute(trajectory);
    
    // Call pick to pick up the object using the grasps given
    //move_group.pick("target_box", pre_grasps);
    // END_SUB_TUTORIAL
    return ex;
}

void pick2(moveit::planning_interface::MoveGroupInterface &move_group2)
{

    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);


    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasps[0].grasp_pose.header.frame_id = "world";
    tf2::Quaternion orientation;
    orientation.setRPY(tau / 2, 0, tau / 8);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    //grasps[0].grasp_pose.pose.position.x = 0.05;
    grasps[0].grasp_pose.pose.position.x = 0.01;
    grasps[0].grasp_pose.pose.position.y = 0.615;
    grasps[0].grasp_pose.pose.position.z = 0.88;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
    /* Direction is set as negative z axis */
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.04;
    grasps[0].pre_grasp_approach.desired_distance = 0.225;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.001;
    grasps[0].post_grasp_retreat.desired_distance = 0.002;

    closedGripper(grasps[0].grasp_posture);

    
    // Set support surface as table1.
    move_group2.setSupportSurfaceName("table");
    move_group2.setGoalJointTolerance(1);
    // Call pick to pick up the object using the grasps given
    move_group2.pick("target_box", grasps);
    // END_SUB_TUTORIAL
}





void place(moveit::planning_interface::MoveGroupInterface &group)
{

    // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
    // a single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "world";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, -tau/8); // A quarter turn
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* For place location, we set the value to the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0.2;
    place_location[0].place_pose.pose.position.y = -0.5;
    place_location[0].place_pose.pose.position.z = 0.825;

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "world";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.04;
    place_location[0].pre_place_approach.desired_distance = 0.10;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "world";
    /* Direction is set as positive z axis */
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.001;
    place_location[0].post_place_retreat.desired_distance = 0.002;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table");
    // Call place to place the object using the place locations given.
    group.place("target_box", place_location);

}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    // Add the table where the coke will originally be kept.
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "world";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.8;
    collision_objects[0].primitives[0].dimensions[1] = 1.58;
    collision_objects[0].primitives[0].dimensions[2] = 0.72;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.365;
    collision_objects[0].primitive_poses[0].orientation.z = 0;

    collision_objects[0].operation = collision_objects[0].ADD;


    collision_objects[1].id = "wall";
    collision_objects[1].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.1;
    collision_objects[1].primitives[0].dimensions[1] = 2.5;
    collision_objects[1].primitives[0].dimensions[2] = 2.5;

    /* Define the pose of the object. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = -0.54;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 1.25;
    collision_objects[1].primitive_poses[0].orientation.z = 0;

    collision_objects[1].operation = collision_objects[1].ADD;

    collision_objects[2].id = "cabinet";
    collision_objects[2].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.5;
    collision_objects[2].primitives[0].dimensions[1] = 0.5;
    collision_objects[2].primitives[0].dimensions[2] = 2.5;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = -0.24;
    collision_objects[2].primitive_poses[0].position.y = 1.23;
    collision_objects[2].primitive_poses[0].position.z = 1.25;
    collision_objects[2].primitive_poses[0].orientation.z = 0;

    collision_objects[2].operation = collision_objects[2].ADD;


      // Define the object that we will be manipulating
    collision_objects[3].id = "target_box";
    collision_objects[3].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.05;
    collision_objects[3].primitives[0].dimensions[1] = 0.07;
    collision_objects[3].primitives[0].dimensions[2] = 0.07;

    /* Define the pose of the object. */
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.052468;
    collision_objects[3].primitive_poses[0].position.y = 0.618629;
    collision_objects[3].primitive_poses[0].position.z = 0.76525;
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;
    collision_objects[3].operation = collision_objects[3].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    /*
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0 , -M_PI/2);
    
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "pose_d";
    
    transformStamped.transform.translation.x = 0.28;
    transformStamped.transform.translation.y = 0.2;
    transformStamped.transform.translation.z = 1.2;
    transformStamped.transform.rotation= tf2::toMsg(orientation);
    
    tfb.sendTransform(transformStamped);
    */
    
    /*
    Eigen::Vector3d c(0.35, 0, 0.43);// Valid spheres for all views: {[c(0.4,0,0.45), r=0.1], [c(0.33,0,0.43), r=0.15], [c(0.15,-0.15,0.45), r=0.15],[c(0.13,-0.13,0.45), r=0.15],[c(0.45, 0, 0.3), r=0.08 excelente], }
    double r=0.08; 
    View_Space view_space(c,r);
    tf2_ros::TransformBroadcaster tfb;
    sendTF(view_space,  tfb);
    */
    
//=======================================================================
// Services
//=======================================================================
#ifdef GAZEBO_LINK_ATTACHER
    attach_srv_.request.model_name_2 = "panda";
    attach_srv_.request.link_name_2 = "panda_leftfinger";
    // attach_srv_.request.model_name_2 = "panda_arm";
    // attach_srv_.request.link_name_2 = "panda_link7";

    attach_client_ = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_"
                                                                       "attacher_"
                                                                       "node/"
                                                                       "attach");

    detach_client_ = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_"
                                                                       "attacher_"
                                                                       "node/"
                                                                       "detach");
#endif

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher frame_pub = n.advertise<std_msgs::Int32>("frame", 1000);

    ros::WallDuration(3.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(60.0);

    addCollisionObjects(planning_scene_interface);
    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();
    
    //to move to home position
    //rotationConstraint(group);
    
    ros::Subscriber sub = n.subscribe("/move_group/feedback", 1000, getStatus);
    
    bool update=0;
    std_msgs::String msg;
    std_msgs::Int32 msg2;
    std::vector< geometry_msgs::Pose > vsTargets = getTFpose();
    tf2_ros::Buffer tfBuff;
    tf2_ros::TransformListener tfListen(tfBuff);
    geometry_msgs::TransformStamped ee_link0;
    for (int i = 0; i < vsTargets.size(); i++){
      
      if (update==1){
       ros::WallDuration(2.0).sleep();
       vsTargets = getTFpose();
       update = 0;
      }
      
      
      Eigen::Quaterniond rec = rectification(vsTargets[i].orientation.x, vsTargets[i].orientation.y, vsTargets[i].orientation.z, vsTargets[i].orientation.w);
      
      vsTargets[i].orientation.x = rec.x();
      vsTargets[i].orientation.y = rec.y();
      vsTargets[i].orientation.z = rec.z();
      vsTargets[i].orientation.w = rec.w();
      //cout<< vsTargets.header.fr <<endl;
      int err_code = move(group, vsTargets[i]); 
      //cout<< "Exito: "<<exito<<endl;
      ee_link0 = tfBuff.lookupTransform("panda_link0", "camera_link",ros::Time(0),ros::Duration(5.0));
        
      bool controlFail = control_failed(vsTargets[i],  ee_link0);
      
      string estado= "CONTINUE";
      if(err_code == -6){
        
        // publish move hemisphere
        estado= "Move Hemisphere";
        cout<< estado << endl;
        update = 1;
        //cout<< frame <<endl;
        //vsTargets = getTFpose();
        
        
      }
      else if (controlFail == 1 and err_code !=-6){
       cout<< "Retry"<< endl;
       //crear un while para reintentar el objetivo
       while (controlFail == 1){
         err_code = move(group, vsTargets[i]); 
         ee_link0 = tfBuff.lookupTransform("panda_link0", "camera_link",ros::Time(0),ros::Duration(5.0));    
         controlFail = control_failed(vsTargets[i],  ee_link0);
         cout<<"controlFail: "<< controlFail<<endl;
         
       }
      }
      else if (controlFail == 0 and err_code !=-6){
       cout<< "Continue with next frame"<< endl;
       //do nothing
      }
      msg.data = estado;
      chatter_pub.publish(msg);
      int fr = i+1;
      msg2.data = fr;
      frame_pub.publish(msg2);
      
      ros::spinOnce();
      
      
      
      ros::WallDuration(1.0).sleep();
    }
    
    

    
    ros::waitForShutdown();

    return 0;
}
