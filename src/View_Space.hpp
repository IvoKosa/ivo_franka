
#include <vector>
#include <Eigen/Core>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream> 
#include <cmath>
#include <complex> 

using namespace std;

class View
{
  public:
	Eigen::Vector3d init_pos;
	//Eigen::Matrix4d pose;	
	Eigen::Transform<double, 3, 0, 0> t;
	
	
  View(Eigen::Vector3d _init_pos, Eigen::Vector3d center, Eigen::Matrix4d t_1) { //init_pos is just a point in the hemisphere (probably in the order read in the .txt file
		init_pos = _init_pos;
		//pose = Eigen::Matrix4d::Identity(4, 4); // JUst to initialize, the rotation is an identity matrix
		get_next_camera_pos(center, t_1);
	}

  //void get_next_camera_pos(Eigen::Matrix4d now_camera_pose_world, Eigen::Vector3d object_center_world) 
  void get_next_camera_pos(Eigen::Vector3d object_center_world, Eigen::Matrix4d t_1)
  {
    
    //Define a object center pose
    Eigen::Matrix4d Tworld2center=  Eigen::Matrix4d::Identity(4, 4);
    Tworld2center(0,3)=object_center_world(0);
    Tworld2center(1,3)=object_center_world(1);
    Tworld2center(2,3)=object_center_world(2);
    
    
    Eigen::Vector3d object(object_center_world(0), object_center_world(1), object_center_world(2));
    Eigen::Vector3d view(init_pos(0), init_pos(1), init_pos(2));
		
    Eigen::Vector3d Z;	 Z = object - view;	 Z = Z.normalized();
    Eigen::Vector3d X;	 X = Z.cross(view);	 X = X.normalized();
    Eigen::Vector3d Y;	 Y = Z.cross(X);	 Y = Y.normalized();
    
    Eigen::Matrix4d T(4, 4);
    T(0, 0) = 1; T(0, 1) = 0; T(0, 2) = 0; T(0, 3) = view(0);
    T(1, 0) = 0; T(1, 1) = 1; T(1, 2) = 0; T(1, 3) = view(1);
    T(2, 0) = 0; T(2, 1) = 0; T(2, 2) = 1; T(2, 3) = view(2);
    T(3, 0) = 0; T(3, 1) = 0; T(3, 2) = 0; T(3, 3) = 1;
    
    Eigen::Matrix4d R(4, 4);
    R(0, 0) = X(0); R(0, 1) = Y(0); R(0, 2) = Z(0); R(0, 3) = 0;
    R(1, 0) = X(1); R(1, 1) = Y(1); R(1, 2) = Z(1); R(1, 3) = 0;
    R(2, 0) = X(2); R(2, 1) = Y(2); R(2, 2) = Z(2); R(2, 3) = 0;
    R(3, 0) = 0;	R(3, 1) = 0;	R(3, 2) = 0;	R(3, 3) = 1;
    //pose = (T*R).eval();
    Eigen::Transform<double, 3, 0, 0> h(T*R);
    t=h;
    //cout << pose.matrix() << endl;
    //cout << t(0,3) << endl;
    //Eigen::Matrix4d t_1=Eigen::Matrix4d::Identity(4, 4);
    
    double phi_min= minimize_Rzangle( t(0,0), t(1,0), t(2,0), t(0,1), t(1,1), t(2,1), t_1(0,0), t_1(1,0), t_1(2,0) );
    //double phi_min= minimize_Rzangle( t(0,0), 0, 0, t(0,1), 0, 0, t_1(0,0), 0, 0 );
    
    //t= t *  Eigen::AngleAxis<double>( phi_min + (M_PI/2), Eigen::Vector3d::UnitZ());
    t= t *  Eigen::AngleAxis<double>( phi_min + (M_PI/2) , Eigen::Vector3d::UnitZ());
    
    double angle = acos( t(1,0) );
    
    if (angle > (M_PI/2) ){
      t= t * Eigen::AngleAxis<double>( (M_PI), Eigen::Vector3d::UnitZ());
    }
    
    // Tcenter2i= inverse(Tworld2center)*(Tworld2i)
    // Define Tworld2newcenter, the center rotated a cecrtain angle
    // t = (Tworld2newcenter)*(Tcenter2i)
    Eigen::Transform<double, 3, 0, 0> Tcenter2i( Tworld2center.inverse()*( t.matrix() ) );
    Eigen::Transform<double, 3, 0, 0> Tworld2newcenter;
    Tworld2newcenter=  Tworld2center ;
    double circleRot=atan2(object_center_world(0), object_center_world(1));
    //Tworld2newcenter= Tworld2newcenter * Eigen::AngleAxis<double>( M_PI-circleRot, Eigen::Vector3d::UnitZ()); //-(M_PI/2)
    Tworld2newcenter= Tworld2newcenter * Eigen::AngleAxis<double>( -circleRot, Eigen::Vector3d::UnitZ()); //-(M_PI/2)
    t = Tworld2newcenter.matrix() *Tcenter2i.matrix();
    
    
    
    
    //cout << "Prev_Pose: "<< endl << t_1.matrix() << endl;
    //cout << "angle_y: " << endl << angle << endl;
    //cout << "After_Pose: " << endl << t.matrix() << endl;
    
    //cout << phi_min << endl;
  }
  
  double minimize_Rzangle( double ra11, double ra21, double ra31, double ra12, double ra22, double ra32, double rb11, double rb21, double rb31){
    
    
    double real_num= - rb21*ra11*ra12*ra22  - rb31*ra11*ra12*ra32  + rb11*ra11*pow(ra22,2)  + rb11*ra11*pow(ra32,2) + rb21*pow(ra12,2)*ra21 +rb31*pow(ra12,2)*ra31 - rb11*ra12*ra21*ra22  - rb11*ra12*ra31*ra32 -rb31*ra21*ra22*ra32 + rb21*ra21*pow(ra32,2) + rb31*pow(ra22,2)*ra31 - rb21*ra22*ra31*ra32;
    
    double imag_num= rb21*pow(ra11,2)*ra22 + rb31*pow(ra11,2)*ra32 - rb21*ra11*ra12*ra21- rb31*ra11*ra12*ra31 - rb11*ra11*ra21*ra22 - rb11*ra11*ra31*ra32 + rb11*ra12*pow(ra21,2) + rb11*ra12*pow(ra31,2) + rb31*pow(ra21,2)*ra32  - rb31*ra21*ra22*ra31 - rb21*ra21*ra31*ra32 + rb21*ra22*pow(ra31,2);
    
    double imag_den= - rb21*pow(ra11,2)*ra22 - rb31*pow(ra11,2)*ra32 + rb21*ra11*ra12*ra21 + rb31*ra11*ra12*ra31 + rb11*ra11*ra21*ra22 + rb11*ra11*ra31*ra32 - rb11*ra12*pow(ra21,2) - rb11*ra12*pow(ra31,2) - rb31*pow(ra21,2)*ra32 + rb31*ra21*ra22*ra31 + rb21*ra21*ra31*ra32 - rb21*ra22*pow(ra31,2);
    
    double real_den = - rb21*ra11*ra12*ra22  - rb31*ra11*ra12*ra32 + rb11*ra11*pow(ra22,2)  + rb11*ra11*pow(ra32,2) + rb21*pow(ra12,2)*ra21 + rb31*pow(ra12,2)*ra31  - rb11*ra12*ra21*ra22 - rb11*ra12*ra31*ra32 - rb31*ra21*ra22*ra32 + rb21*ra21*pow(ra32,2) + rb31*pow(ra22,2)*ra31 - rb21*ra22*ra31*ra32; 
    
    /*
    double real_num = sqrt( - pow(ra11,2)*pow(rb21,2) + 2*ra11*ra21*rb11*rb21 - pow(ra12,2)*pow(rb21,2) + 2*ra12*ra22*rb11*rb21 - pow(ra21,2)*pow(rb11,2) - pow(ra22,2)*pow(rb11,2) );
    
    double imag_num = 0;
    
    
    double real_den = ra11*rb21 - ra21*rb11 ;
    
    double imag_den = - ra12*rb21 + ra22*rb11;
    */
    
    complex<double> numerador(real_num , imag_num);
    
    complex<double> denominador(real_den , imag_den);
    
    complex<double> sustraendo_1 = log( ( numerador )/( denominador ) ) ;
    complex<double> sustraendo_2(0.0 , 1.0);
    
    complex<double> producto = sustraendo_1 * sustraendo_2;
    
    complex<double> sustraendo( real(producto)/2, imag(producto)/2 );
    //complex<double> sustraendo( real(producto), imag(producto) );
    
    complex<double> Pi(M_PI,0);
    
    complex<double> min_angle = Pi - ( sustraendo );
    
    cout << "Extreme_Angle: " << real(min_angle) << "," << imag(min_angle) << endl;
    
    
    return real(min_angle);
     
  
  }
    
  
  vector<double> getRPY(){
   Eigen::Matrix3d rotationMatrix = t.rotation();
   double pitch= atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(1,0)*rotationMatrix(1,0) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  );
   double roll= atan2( rotationMatrix(2,1),rotationMatrix(2,2) );
   double yaw= atan2( rotationMatrix(1,0),rotationMatrix(0,0) );
   
   vector<double> rpy = {roll, pitch, yaw};
   
    
   return rpy;
  }
  
  

};


class View_Space
{
  public:
    							
    Eigen::Vector3d object_center_world;
    double radio;
    int num_views;
    vector<vector<double>> pt_sphere;
    vector<View> views;	
    

  void get_view_space() { //By now, everything in world coordinates (maybe later in the view space)
  	//Define a variable for the precious frame and the next frame, so that the angle would be minimize
  	// With respect to the previous frame and not wrt the world origin
  	Eigen::Matrix4d t_1=Eigen::Matrix4d::Identity(4, 4);
  	for (int i = 0; i < pt_sphere.size(); i++) {
  	   //cout <<"[" <<pt_sphere[i][0] << ","<< pt_sphere[i][1] << ","<< pt_sphere[i][2] << " ]"<< endl;
  	   View view( Eigen::Vector3d(pt_sphere[i][0], pt_sphere[i][1], pt_sphere[i][2]), object_center_world, t_1 );
  	   views.push_back(view);
  	   //t_1= view.t.matrix();
  	   //cout<<view.t.translation()<<endl;
  	}
    
      
    
  }

//Constructor
  View_Space(Eigen::Vector3d center, double r) {
    
    
    
    object_center_world=center;
    radio=r;
    num_views=32;
    
    
    ifstream fin_sphere("/home/ivokosa/ros/noetic/system/src/ivo_franka/src/tot_sort.txt");
    pt_sphere.resize(num_views);
    
    //double pts_subscaled;
    
    
    if (fin_sphere.is_open()){
      for (int i = 0; i < num_views; i++) {
	 pt_sphere[i].resize(3);
	 for (int j = 0; j < 3; j++) {
	  //fin_sphere>> pt_sphere[i][j];
	  fin_sphere>> pt_sphere[i][j];
	  //cout << pt_sphere[i][j] << endl;
			}
	  double px = pt_sphere[i][0];
	  double py = pt_sphere[i][1];
	  double pz = pt_sphere[i][2];
	  pt_sphere[i][0]= radio*(  px/ sqrt(pow(px,2)+ pow(py,2)+ pow(pz,2)) ) + object_center_world[0] ;
	  pt_sphere[i][1]= radio*(  py/ sqrt(pow(px,2)+ pow(py,2)+ pow(pz,2)) ) + object_center_world[1] ;
	  pt_sphere[i][2]= radio*(  pz/ sqrt(pow(px,2)+ pow(py,2)+ pow(pz,2)) ) + object_center_world[2] ;
	  
	  //cout <<"[" <<pt_sphere[i][0] <<" ," << pt_sphere[i][1] <<" ," << pt_sphere[i][2] <<" ]"<< endl;
		}
	  
	}
	
     
    else cout << "Unable to open file view_space.txt"<<endl; 
    //cout << pt_sphere << endl;
    
    get_view_space();	
		
  }
//Constructor end

//Function to return the views vector as a TransformStamped vector
  vector<geometry_msgs::TransformStamped> views2tStamped(){
    
    vector<geometry_msgs::TransformStamped> tfStamped_vector;
    
    for (int i = 0; i < views.size(); i++) {
      geometry_msgs::TransformStamped transformStamped;
      vector<double> v_rpy = views[i].getRPY();
      Eigen::Vector3d v_xyz = views[i].t.translation();    
      
      geometry_msgs::TransformStamped tS;
      
      tS.header.frame_id = "panda_link0";
      tS.header.stamp = ros::Time::now();
      tS.child_frame_id = "tf_d"+to_string(i);
      //Position
      tS.transform.translation.x = v_xyz[0];
      tS.transform.translation.y = v_xyz[1];
      tS.transform.translation.z = v_xyz[2];
      
      //Orientation
      //RPY(x,y,z) Rz*Ry*Rx
      tf2::Quaternion q;
      //q.setRPY(M_PI, M_PI/2, M_PI/2);
      q.setRPY(v_rpy[0], v_rpy[1], v_rpy[2]);
      tS.transform.rotation.x = q.x();
      tS.transform.rotation.y = q.y();
      tS.transform.rotation.z = q.z();
      tS.transform.rotation.w = q.w();
      //Add the TransformStamped to its corresponding vector
      tfStamped_vector.push_back(tS);
      
    
    } 
  //cout <<"Num_tS: " << tfStamped_vector.size() << endl;
  return tfStamped_vector;
}

  //In the case that the robot can not reach a view, the view space can be updated (assuming that the object was taken to another zone of the table plane)
  void update(Eigen::Vector3d center, double r){
    pt_sphere.clear();
    views.clear();
  
    object_center_world=center;
    radio=r;
    num_views=32;
    
    
    ifstream fin_sphere("/home/ivokosa/ros/noetic/system/src/ivo_franka/src/tot_sort.txt");
    pt_sphere.resize(num_views);
    
    //double pts_subscaled;
    
    
    if (fin_sphere.is_open()){
      for (int i = 0; i < num_views; i++) {
	 pt_sphere[i].resize(3);
	 for (int j = 0; j < 3; j++) {
	  //fin_sphere>> pt_sphere[i][j];
	  fin_sphere>> pt_sphere[i][j];
	  //cout << pt_sphere[i][j] << endl;
			}
	  double px = pt_sphere[i][0];
	  double py = pt_sphere[i][1];
	  double pz = pt_sphere[i][2];
	  pt_sphere[i][0]= radio*(  px/ sqrt(pow(px,2)+ pow(py,2)+ pow(pz,2)) ) + object_center_world[0] ;
	  pt_sphere[i][1]= radio*(  py/ sqrt(pow(px,2)+ pow(py,2)+ pow(pz,2)) ) + object_center_world[1] ;
	  pt_sphere[i][2]= radio*(  pz/ sqrt(pow(px,2)+ pow(py,2)+ pow(pz,2)) ) + object_center_world[2] ;
	  
	  //cout <<"[" <<pt_sphere[i][0] <<" ," << pt_sphere[i][1] <<" ," << pt_sphere[i][2] <<" ]"<< endl;
		}
	  
	}
	
     
    else cout << "Unable to open file view_space.txt"<<endl; 
    //cout << pt_sphere << endl;
    
    get_view_space();
    cout << views.size() << endl;
      
    
    
  
  }









};



