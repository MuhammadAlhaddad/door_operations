// PATH FOLLOWING AND MODIFIED PATH_TRAJECTORY_CONTROLLER
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "door_operations/Navigate2DAction.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <random>
#include <stdio.h>
#include <time.h>
#include <fstream>
//
using namespace std;

typedef actionlib::SimpleActionServer<door_operations::Navigate2DAction> NavServer;
door_operations::Navigate2DResult result_msg;
door_operations::Navigate2DFeedback feedback_msg;
// declare error parameters
double distance_error = 0 , angle_error = 0;

// declare condition parametrs
double  distance_stop = 0.45;

// declare control parameters
double v_cmd = 0 , w_cmd = 0;
double v_robot = 0;
double w_robot = 0;

int ind_control = 0;
    double max_v_local = 0;

// declare integers k_v is index depends on position and h is index depends on time
int k_v = 0;

double d_stop = 0 , dt=0.1;
bool robot_orientation_corrected = false;

int door_case = 0;

// declare path parameters

bool  path_stored = false ,  robot_reached = false , start_node = false , activate_cmd = false , doing_parking = true  , command_recieved = false;

// declare odometry parametrs
double odom_x, odom_y, odom_z , odom_theta;

double x_parking11[]={-0.8 , -0.3 , 0.5};
double y_parking11[]={1.1 , 1 , 0.8};

double x_parking12[]={-0.4, -0.1, 0.5};
double y_parking12[]={1.3, 0.9 , 0.8};

double x_parking13[]={0 , -0.4 , -0.4, -0.1, 0.5};
double y_parking13[]={2.2, 1.8, 1.3, 0.9 , 0.8};

double y_parking21[]={-0.8 , -0.3 , 0.5};
double x_parking21[]={1.1 , 1 , 0.8};

double y_parking22[]={-0.4, -0.1, 0.5};
double x_parking22[]={1.3, 0.9 , 0.8};

double y_parking23[]={0 , -0.4 , -0.4, -0.1, 0.5};
double x_parking23[]={2.2, 1.8, 1.3, 0.9 , 0.8};

double x_passing11[]={-0.1, 0.4, 0.43, 0.43};
double y_passing11[]={1.5, 1.2, 0.5, -1};

double x_passing12[]={0.4, 0.4, 0.43, 0.43};
double y_passing12[]={1.5, 1.2, 0.5, -1};

double x_passing13[]={0.8, 0.4, 0.43, 0.43};
double y_passing13[]={1.5, 1.2, 0.5, -1};

double y_passing21[]={-0.1, 0.4, 0.43, 0.43};
double x_passing21[]={1.5, 1.2, 0.5, -1};

double y_passing22[]={0.4, 0.4, 0.43, 0.43};
double x_passing22[]={1.5, 1.2, 0.5, -1};

double y_passing23[]={0.8, 0.4, 0.43, 0.43};
double x_passing23[]={1.5, 1.2, 0.5, -1};

double x_opening1[] = {0 , -1.05};
double y_opening1[] = {0.95 , 0.9};

double x_opening2[] = {0 , -1.05};
double y_opening2[] = {0.95 , 0.9};

double x[20] , y[20];

uint32_t size_path = 5;

int direction = 1;
double compensate_angle = 0;
int asked_motion = 0;
// declare trajectory parameters
geometry_msgs::Pose2D qp[100] , err;


// declare message parameters
geometry_msgs::Twist        tw_msg;
geometry_msgs::Point        robot_pt , path_pt;
visualization_msgs::Marker  robot_pt_mk , path_pt_mk;



// declare Publishers and Subscribers
ros::Publisher   cmd_pub;
ros::Publisher   path_pt_mk_pub;
ros::Publisher   robot_pt_mk_pub;


// declare topics
std::string cmd_topic;
std::string global_frame;
std::string base_frame;
std::string door_status;


// declare config parameters
double max_v;
double min_v;
double acc_v;
double max_w;
double acc_w;

// declare functions
double get_yaw(geometry_msgs::Quaternion q);
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);
void stop_mode();
void get_path_errors();
void forward_controller();
void backward_controller();
void initialize_parameters(ros::NodeHandle n);
void initialize_markers();
void control_method();
void publish_marker_odom();
void correct_orientation();
void path_back();
void right_case_paths();
void left_case_paths();
void check_frame_case();
void publish_result(NavServer* nav_server);

std::string string_status , string_goal;



void navCallback(const door_operations::Navigate2DGoal::ConstPtr& goal_point,
                 NavServer* nav_server)
{

  ROS_INFO("Executing order %s", goal_point->goal.c_str());

  string_goal = goal_point->goal;

  if(goal_point->goal=="makeparking")
  {     
      asked_motion = 1;
  }
  else if(goal_point->goal=="opendoor")
  {
      asked_motion = 2;
  }
  else if(goal_point->goal=="gotoroom")
  {
      asked_motion = 3;
  }
  check_frame_case();

  ros::Rate feedback_rate(2);

  double goal_received_time = ros::Time::now().toSec();
  ROS_INFO("Goal Received");

  publish_result(nav_server);
  
  feedback_rate.sleep(); 

}

void publish_result(NavServer* nav_server)
{
    ros::Rate feedback_rate(5);

    while(!robot_reached)
    {       
        ROS_INFO("Robot moving");
        feedback_msg.x = odom_x;
        feedback_msg.y = odom_y;
        nav_server->publishFeedback(feedback_msg);
        feedback_rate.sleep();
    }
    result_msg.status = string_status;
    nav_server->setSucceeded(result_msg);
}

void check_frame_case() {
  if (door_status=="right")
  {
      right_case_paths();
      ROS_INFO("Case: Door opening by right hand");
  }
  else if (door_status=="left")
  {
      ROS_INFO("Case: Door opening by left hand");
      left_case_paths();
  }
  else
  {
      ROS_WARN("Error in choosing door status! Choose 'right' if the handle is located at left side or 'left' otherwise");
  }
}

void right_case_paths()
{
    double angle_position_robot = atan2(odom_y,odom_x)*180/3.1415;
    ROS_INFO("Robot is at orientation %f in respect to origin", angle_position_robot);
    if(asked_motion==1)
    {
      if (angle_position_robot >104)
      {
        size_path = sizeof x_parking11 / sizeof x_parking11[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_parking11[i];
            y[i] = y_parking11[i];
        }
      }
      else if (angle_position_robot <=104 && angle_position_robot >90)
      {
        size_path = sizeof x_parking12 / sizeof x_parking12[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_parking12[i];
            y[i] = y_parking12[i];
        }
      }
      else if (angle_position_robot <=90 && angle_position_robot >=0)
      {
        size_path = sizeof x_parking13 / sizeof x_parking13[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_parking13[i];
            y[i] = y_parking13[i];
        }
      }
      compensate_angle = 0;
      direction = 1;
    }
    else if (asked_motion==2)
    {
      size_path = sizeof x_opening1 / sizeof x_opening1[0];
      for (int i=0; i<size_path; i++)
      {
          x[i] = x_opening1[i];
          y[i] = y_opening1[i];
      }
      compensate_angle = 3.14;
      direction = -1;
    }
    else if (asked_motion==3)
    {
      if (angle_position_robot> 95)
      {
        size_path = sizeof x_passing11 / sizeof x_passing11[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_passing11[i];
            y[i] = y_passing11[i];
        }
      }
      else if (angle_position_robot<=95 && angle_position_robot>=63)
      {
        size_path = sizeof x_passing12 / sizeof x_passing12[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_passing12[i];
            y[i] = y_passing12[i];
        }
      }
      else if (angle_position_robot<63 && angle_position_robot>=0)
      {
        size_path = sizeof x_passing13 / sizeof x_passing13[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_passing13[i];
            y[i] = y_passing13[i];
        }
      }
      compensate_angle = 0;
      direction = 1;
    }
    path_back();
}

void left_case_paths()
{
    double angle_position_robot = atan2(odom_y,odom_x)*180/3.1415;
    ROS_INFO("Robot is at orientation %f in respect to origin", angle_position_robot);
    if(asked_motion==1)
    {
      if (angle_position_robot < -19)
      {
        size_path = sizeof x_parking21 / sizeof x_parking21[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_parking21[i];
            y[i] = y_parking21[i];
        }
      }
      else if (angle_position_robot >=-19 && angle_position_robot <0)
      {
        size_path = sizeof x_parking22 / sizeof x_parking22[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_parking22[i];
            y[i] = y_parking22[i];
        }
      }
      else if (angle_position_robot >=0)
      {
        size_path = sizeof x_parking23 / sizeof x_parking23[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_parking23[i];
            y[i] = y_parking23[i];
        }
      }
      compensate_angle = 0;
      direction = 1;
    }
    else if (asked_motion==2)
    {
      size_path = sizeof x_opening2 / sizeof x_opening2[0];
      for (int i=0; i<size_path; i++)
      {
          x[i] = x_opening2[i];
          y[i] = y_opening2[i];
      }
      compensate_angle = 3.14;
      direction = -1;
    }
    else if (asked_motion==3)
    {
      if (angle_position_robot < -19)
      {
        size_path = sizeof x_passing21 / sizeof x_passing21[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_passing21[i];
            y[i] = y_passing21[i];
        }
      }
      else if (angle_position_robot>=-19 && angle_position_robot<30)
      {
        size_path = sizeof x_passing22 / sizeof x_passing22[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_passing22[i];
            y[i] = y_passing22[i];
        }
      }
      else if (angle_position_robot>=30)
      {
        size_path = sizeof x_passing23 / sizeof x_passing23[0];
        for (int i=0; i<size_path; i++)
        {
            x[i] = x_passing23[i];
            y[i] = y_passing23[i];
        }
      }
      compensate_angle = 0;
      direction = 1;
    }
    path_back();
}

// Callback to get the Path from Planning node

void path_back() 
{
    path_pt_mk.points.clear();
    robot_pt_mk.points.clear();
    activate_cmd = true;
    start_node = true;
    path_pt.x = odom_x;
    path_pt.y = odom_y;
    path_pt_mk.points.push_back(path_pt);
            ROS_INFO("New Path");
            for (int j=0; j<size_path ; j++)
            { 
                qp[j].x = x[j];
                qp[j].y = y[j];
                ROS_INFO("x , y %f %f" , qp[j].x , qp[j].y );
                path_pt.x = qp[j].x;
                path_pt.y = qp[j].y;
                path_pt_mk.points.push_back(path_pt);
            }
            path_pt_mk_pub.publish(path_pt_mk);
        
        ROS_INFO("size path: %i" , size_path );
        path_stored = true;
        robot_reached = false;
        
        get_path_errors();
        stop_mode();
  
}

void forward_controller()
{
    //  DECREASING
    if (k_v >= size_path-1 && distance_error<0.5)// && abs(angle_error)<=0.25 && v_cmd>0.3)  // decreasing
    {
        v_cmd = sat_linear_velocity(0.15,0,acc_v ,distance_error, v_cmd);
        w_cmd = 0;
        ind_control = -1;
    }
    else if (distance_error>0.05)
    {
        // Motion 
        if(abs(angle_error)<0.12)  // case error angle very small
        {
            v_cmd = sat_linear_velocity(max_v,0,acc_v,distance_error , v_cmd);
            w_cmd = angle_error;
            ind_control = 1;
        }
        else if(abs(angle_error)>=0.12 && abs(angle_error)<0.45)  // case error angle small
        {
            v_cmd = sat_linear_velocity(max_v-0.05,0,acc_v,distance_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 2;
        }
        else if(abs(angle_error)>=0.45 && abs(angle_error)<0.8)  // case error angle medium
        {
            ROS_INFO("max_v %f" , max_v);
            v_cmd = sat_linear_velocity(max_v-0.08,0,acc_v,distance_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 3;
        }
        else if(abs(angle_error)>=0.8 && abs(angle_error)<1.25)  // case error angle big
        {
            v_cmd = sat_linear_velocity(max_v-0.1,0,acc_v,distance_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 4;
        }
        else if(abs(angle_error)>=1.25 && abs(angle_error)<1.4)  // case angle error very big
        {
            v_cmd = sat_linear_velocity(max_v-0.15,0,acc_v,distance_error , v_cmd);
            w_cmd = 1*angle_error; 
            ind_control = 5;
        }     
        ROS_INFO("Enter control");
    }
    ROS_INFO("v_cmd %f" , v_cmd);

        

    if(w_cmd>max_w)
    {w_cmd = max_w;}
    if(w_cmd<-max_w)
    {w_cmd = -max_w;}
 /*   if(v_cmd>max_v)
    {v_cmd = max_v;}
    if(v_cmd<min_v)
    {v_cmd = min_v;}*/

    ROS_INFO("kv siz ind errors %i %i %i %f %f %f %f" , k_v , size_path , ind_control, distance_error , angle_error , v_cmd , w_cmd);
     
    tw_msg.linear.x=v_cmd;
    tw_msg.angular.z=w_cmd;
    
}

void backward_controller()
{
    //  DECREASING
    if (k_v >= size_path-1 && distance_error<0.5)// && abs(angle_error)<=0.25 && v_cmd>0.3)  // decreasing
    {
        v_cmd = sat_linear_velocity(0,-0.15,acc_v ,distance_error, v_cmd);
        w_cmd = 0;
        ind_control = -1;
    }
    else if (distance_error>0.05)
    {
        // Motion 
        if(abs(angle_error)<0.12)  // case error angle very small
        {
            v_cmd = sat_linear_velocity(0,min_v,acc_v,distance_error , v_cmd);
            w_cmd = angle_error;
            ind_control = 1;
        }
        else if(abs(angle_error)>=0.12 && abs(angle_error)<0.45)  // case error angle small
        {
            v_cmd = sat_linear_velocity(0,min_v+0.05,acc_v,distance_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 2;
        }
        else if(abs(angle_error)>=0.45 && abs(angle_error)<0.8)  // case error angle medium
        {
            ROS_INFO("max_v %f" , max_v);
            v_cmd = sat_linear_velocity(0,min_v+0.08,acc_v,distance_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 3;
        }
        else if(abs(angle_error)>=0.8 && abs(angle_error)<1.25)  // case error angle big
        {
            v_cmd = sat_linear_velocity(0,min_v+0.1,acc_v,distance_error , v_cmd);
            w_cmd = 1*angle_error;
            ind_control = 4;
        }
        else if(abs(angle_error)>=1.25 && abs(angle_error)<1.4)  // case angle error very big
        {
            v_cmd = sat_linear_velocity(0,min_v+0.12,acc_v,distance_error , v_cmd);
            w_cmd = 1*angle_error; 
            ind_control = 5;
        } 
        else
        {
            v_cmd = 0;
            w_cmd = 0.4*angle_error;
            ind_control = 6;
        }    
        ROS_INFO("Enter control");
    }
    ROS_INFO("min_v %f" , min_v);

    if(w_cmd>max_w)
    {w_cmd = max_w;}
    if(w_cmd<-max_w)
    {w_cmd = -max_w;}
/*    if(v_cmd>max_v)
    {v_cmd = max_v;}
    if(v_cmd<min_v)
    {v_cmd = min_v;}*/

    ROS_INFO("kv siz ind errors %i %i %i %f %f %f %f" , k_v , size_path , ind_control, distance_error , angle_error , v_cmd , w_cmd);
     
    tw_msg.linear.x=v_cmd;
    tw_msg.angular.z=w_cmd;
    
}

// Get the errors between the robot and path
void get_path_errors()
{
    double angle_path2=0;
    if(path_stored)
    {
    //Calculate the erros between the robot and the current segement
        err.x = (qp[k_v].x-odom_x) ; 
        err.y =  (qp[k_v].y-odom_y) ;
        distance_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
        
        double angle_path1=atan2(err.y,err.x);
        err.theta =  angle_path1 - odom_theta - compensate_angle;
        angle_error = atan2(sin(err.theta ),cos(err.theta));


    // accordingly to these errors move to the next segment      
        if (distance_error<distance_stop && k_v <size_path-1)
        {
        k_v++;
        }
    } 
}

void correct_orientation()
{
    if (abs(angle_error)>=0.11)
    {
        w_cmd = angle_error;
        if(w_cmd>max_w)
         {w_cmd = max_w;}
        if(w_cmd<-max_w)
        {w_cmd = -max_w;}


        v_cmd = 0;
        tw_msg.linear.x     =   v_cmd;
        tw_msg.angular.z    =   w_cmd;
        if (activate_cmd)
    {
        cmd_pub.publish(tw_msg);
    }
    }
    else
    {
        robot_orientation_corrected = true;
    }
}

// Stop Mode
void stop_mode()
{
    d_stop = sqrt(pow((qp[size_path-1].x-odom_x), 2) + pow((qp[size_path-1].y-odom_y), 2));
    if (d_stop<=0.15)
    {
       path_stored=false;
       robot_reached=true;
       ROS_INFO("Robot Reached to the goal");
       start_node = false;
       k_v = 0;
       string_status = "is done";
       
    }
    else
    {
        robot_reached = false;
        string_status = "is not completed";
    }
}

void publish_marker_odom()
{
    robot_pt.x=odom_x;
    robot_pt.y=odom_y;
    robot_pt_mk.points.push_back(robot_pt);
    robot_pt_mk_pub.publish(robot_pt_mk);

}

// Saturation Linear Speed Function
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double v_real)
{
    double v_diff = v_ref - v_real;
    // saturations on accelerations
    if(v_diff>=0)
    {
        v_diff = v_real + direction * accel*dt;
    }
    else if(v_diff<0)
    {
        v_diff = v_real - direction * accel*dt;
    }
    ROS_INFO("vdif %f" , v_diff);

    if (v_diff<min)
    {
        return min;
    }
    else if (v_diff>max)
    {
        return max;
    }
    else
    {
        return v_diff;
    }
}

void control_method()
{
    if (path_stored) 
    {

        if(direction==-1)
        {
            backward_controller();
        }
        else
        {
            forward_controller();
        }
        
        publish_marker_odom();
    
        if(robot_reached)
        {
            path_stored = false;
        }
        stop_mode();
       
   //   ROS_INFO ("PATH FOLLOW,  [%i], [%f %f] [%f %f] " , k_v ,distance_error, angle_error ,v_cmd, w_cmd);
      
    }
    else
    {
        v_cmd = 0;
        w_cmd = 0;
        
        tw_msg.linear.x=v_cmd;
        tw_msg.angular.z=w_cmd;
      //  ROS_INFO("Complete path");
      //  ROS_INFO("odometry %f , %f , %f" , odom_x ,odom_y , odom_theta);
    }
    
        
    if (activate_cmd)
    {
        cmd_pub.publish(tw_msg);
    }

    if (robot_reached)
    {
        activate_cmd = false;
    }
    
}

// Initialize config parameters
void initialize_parameters(ros::NodeHandle n)
{
    std::string right = "right";
    std::string left = "left";
    n.getParam("cmd_topic" ,    cmd_topic);
    n.getParam("global_frame" , global_frame);
    n.getParam("base_frame" ,   base_frame);
    n.getParam("max_v", max_v);
    max_v = 0.22;

    n.getParam("min_v", min_v);
    min_v = -0.22;
    n.getParam("acc_v", acc_v);
    acc_v = 0.05;
    n.getParam("max_w", max_w);
    n.getParam("acc_w", acc_w);
    
    n.getParam("door_status", door_status);

    max_w = 0.4;
}

void initialize_markers()
{
    path_pt_mk.header.frame_id = global_frame;
    path_pt_mk.header.stamp = ros::Time::now();
    path_pt_mk.ns = "points_and_lines";
    path_pt_mk.action = 0;
    path_pt_mk.pose.orientation.w = 1.0;
    path_pt_mk.type = 4;
    path_pt_mk.scale.x = 0.1;
    path_pt_mk.scale.y = 0.1;
    path_pt_mk.color.r = 1.0f;
    path_pt_mk.color.a = 1.0;
    path_pt_mk.points.clear();

    robot_pt_mk.header.frame_id = global_frame;
    robot_pt_mk.header.stamp = ros::Time();
    robot_pt_mk.ns = "points_and_lines";
    robot_pt_mk.action = visualization_msgs::Marker::ADD;
    robot_pt_mk.pose.orientation.w = 1.0;
    robot_pt_mk.type = 4;
    robot_pt_mk.scale.x = 0.2;
    robot_pt_mk.scale.y = 0.2;
    robot_pt_mk.color.a = 1.0;
    robot_pt_mk.color.r = 0.0;
    robot_pt_mk.color.g = 0.0;
    robot_pt_mk.color.b = 1.0;
    robot_pt_mk.points.clear();

}


int main(int argc, char **argv) {
 
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;

    initialize_parameters(n);

    initialize_markers();
    

    // Define the publishers and sunscribers
    cmd_pub                 = n.advertise<geometry_msgs::Twist>         (cmd_topic, 1);
    robot_pt_mk_pub         = n.advertise<visualization_msgs::Marker>   ("marker_real", 1);
    path_pt_mk_pub          = n.advertise<visualization_msgs::Marker>   ("path_pt_mk", 1);   


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(10); // ros spins 20 frames per second

    NavServer server(n, "navigate_2d", boost::bind(&navCallback, _1, &server), false);
  server.start();
    

    while (ros::ok()) {

    geometry_msgs::TransformStamped transformStamped;
    try{
        
      transformStamped = tfBuffer.lookupTransform(global_frame, base_frame,
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    odom_x = transformStamped.transform.translation.x;
    odom_y = transformStamped.transform.translation.y;
    odom_z = transformStamped.transform.translation.z;
    odom_theta = get_yaw(transformStamped.transform.rotation);
/*
    odom_x = 2;
    odom_y=2;
    odom_theta = 0;
*/

  /*  if (doing_parking)
    {
        path_back();
        doing_parking = false;
    }*/
    if(path_stored)
    {
        get_path_errors();
    }
    if (robot_orientation_corrected)
            {
                control_method();
            }
            else
            {
                correct_orientation();
            }
    

     
    
    ros::spinOnce();
    loop_rate.sleep();
    }
    
    return 0;
}


double get_yaw(geometry_msgs::Quaternion q){
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
