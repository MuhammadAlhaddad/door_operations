#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
//
using namespace std;

std::string global_frame;
std::string base_frame;
double x_frame , y_frame , theta_frame;

double qx , qy , qz , qw;


// Initialize config parameters
void initialize_parameters(ros::NodeHandle n)
{
    n.getParam("global_frame" , global_frame);
    n.getParam("base_frame" ,   base_frame);
    n.getParam("x_frame" ,   x_frame);
    n.getParam("y_frame" ,   y_frame);
    n.getParam("theta_frame" ,   theta_frame);
    
}
/*
void config_frame_bk(const std_msgs::Float32MultiArray msg) {
  x_frame = msg.data[0]; 
  y_frame = msg.data[1];
  theta_frame = msg.data[2];
  ROS_INFO("New door frame at %f %f" , x_frame , y_frame);
}*/

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  initialize_parameters(node);
//  ros::Subscriber frame_sub  = node.subscribe("/conf_frame", 10 , config_frame_bk);
  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10);
  while (ros::ok()){
    transform.setOrigin( tf::Vector3(x_frame, y_frame, 0) );
    qz = sin(0.5*theta_frame);
    qw = cos(0.5*theta_frame);

    transform.setRotation( tf::Quaternion(0, 0, qz, qw) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_frame, base_frame));
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};