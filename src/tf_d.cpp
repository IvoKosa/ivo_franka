#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include "View_Space.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include "moveit_msgs/MoveGroupActionFeedback.h"
#include <string>
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalStatus.h"

using namespace std;

string estado;
string tf_frame;
vector<string> texto;

/*
void getStatus(const actionlib_msgs::GoalStatusArray msg)
{   
    exito = msg.data;
    //tf_frame = msg.header.frame_id;
    
}
*/

void getState(const std_msgs::String msg)
{   
    estado = msg.data;
}

int main(int argc, char** argv){

/*
  Eigen::Vector3d c(0.35, 0, 0.43);// Valid spheres for all views: {[c(0.4,0,0.45), r=0.1], [c(0.33,0,0.43), r=0.15], [c(0.15,-0.15,0.45), r=0.15],[c(0.13,-0.13,0.45), r=0.15],[c(0.45, 0, 0.3), r=0.08 excelente], }
  double r=0.08; 
  View_Space view_space(c,r);
  //cout<< "n_views: "<<view_space.views.size()<<endl;
  
  cout<< "---------------------------------"<<endl;
  Eigen::Vector3d ct(0.1, 0.1, 0.43);
  double ra=0.08;
  view_space.update(ct,ra);
  
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  
  
  
  ros::Rate rate(10.0);
  while (node.ok()){
    vector<geometry_msgs::TransformStamped> tS = view_space.views2tStamped();
    tfb.sendTransform(tS);
    rate.sleep();
    //printf("sending\n");
  }
  */
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle n("~");

    //ros::Publisher chatter_pub = n.advertise<radar_msgs::RadarDetection>("srr2_detections", 1000);
    //ros::Subscriber sub = n.subscribe("/move_group/status", 1000, getStatus);
    ros::Subscriber sub2 = n.subscribe("/chatter", 1000, getState);
    ros::Rate loop_rate(10);
    
    // Cylinder ivo-franka-full
    Eigen::Vector3d c(0.65, 0, 0.09);
    // Cylinder Closer
    // Eigen::Vector3d c(0.43, 0, 0.09);
    
    double r = 0.35; 

    std::vector<double> pos_list;
    double sum = 0;

    n.getParam("pos_list", pos_list);

    for(unsigned i=0; i < pos_list.size(); i++) {
      sum += pos_list[i];
    }

    // if (n.getParam("pos_list", pos_list)) {
    //   cout << "HELLO::::" << endl;
    // }

    // int rad_param;

    // n.getParam("pos_param", pos_param);
    // n.getParam("rad_param", rad_param)

    // ROS_INFO("Got parameter : %d", pos_param);
    // cout << rad_param << endl

    View_Space view_space(c,r);
    tf2_ros::TransformBroadcaster tfb;
    
    string estado_pre= "CONTINUE";
    estado = "CONTINUE";
    //int count = 0;
    while (ros::ok())
    {
            //chatter_pub.publish(pub_data);
            
            ros::spinOnce();
            if (estado== "CONTINUE" or estado_pre == estado){
            cout<<"State:"<<estado<<endl;
            //ros::spinOnce();
            vector<geometry_msgs::TransformStamped> tS = view_space.views2tStamped();
            tfb.sendTransform(tS);
            
            }
            
            else if (estado== "Move Hemisphere" and estado_pre!="Move Hemisphere"){
            //ros::spinOnce();
             cout<<"replace"<<endl;
             cout<<"State:"<<estado<<endl;
             Eigen::Vector3d c(0.65, 0, 0.09);
             view_space.update(c,r);
             vector<geometry_msgs::TransformStamped> tS = view_space.views2tStamped();
             tfb.sendTransform(tS);
             //estado = "CONTINUE";
            }
            estado_pre = estado;
            
            loop_rate.sleep();
            //++count;
    }
    return 0;

};
