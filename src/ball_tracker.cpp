/* 
Authors: Pedro Deniz
           Marlene Cobian 
*/

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/WalkingParam.h"

void readyToDemo();
void setModule(const std::string& module_name);
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();

void goInitPose();
void goAction(int page);
void turn2search();

bool isActionRunning();
bool getJointPose();

void callbackBallCenter(const geometry_msgs::Point& msg);
//void callbackJointStates(const sensor_msgs::JointState& msg);
void tracking();
void publishHeadJoint(double pan, double tilt);

double alpha = 0.4;
double present_pitch_;

const double FOV_WIDTH = 35.2 * M_PI / 180;
const double FOV_HEIGHT = 21.6 * M_PI / 180;

geometry_msgs::Point ball_position;
sensor_msgs::JointState head_angle_msg;

double current_ball_pan = 0;
double current_ball_tilt = 0;
double xerror_sum = 0;
double yerror_sum = 0;

double p_gain = 0.105; //0.6
double d_gain = 0.0012; //0.045
double i_gain = 0.00001;

ros::Time prev_time;

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_head_joint_pub;
ros::Publisher write_head_joint_offset_pub;
ros::Publisher write_joint_pub;
ros::Publisher action_pose_pub;

ros::Subscriber read_joint_sub;
ros::Subscriber ball_sub;
ros::Subscriber imu_sub;

ros::ServiceClient set_joint_module_client;
ros::ServiceClient is_running_client;
ros::ServiceClient get_param_client;

int control_module = None;
bool demo_ready = false;


//node main
int main(int argc, char **argv)
{
  
    //init ros
    ros::init(argc, argv, "read_write");
    ros::NodeHandle nh(ros::this_node::getName());

    int robot_id;
    nh.param<int>("robot_id", robot_id, 0);

    //subscribers
    //read_joint_sub = nh.subscribe("/robotis/present_joint_states",1, callbackJointStates);
    ball_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/ball_center", 5, callbackBallCenter);
    //imu_sub = nh.subscribe("/robotis/open_cr/imu", 1, callbackImu);
    
    //publishers
    init_pose_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/base/ini_pose", 0);
    dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/dxl_torque", 0);
    write_head_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/head_control/set_joint_states", 0);
    write_head_joint_offset_pub = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/head_control/set_joint_states_offset", 0);
    
    //services
    set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis_" + std::to_string(robot_id) + "/set_present_ctrl_modules");
    is_running_client = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis_" + std::to_string(robot_id) + "/action/is_running");
    get_param_client = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/get_params");
    
    ros::start();

    //set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    //wait for starting of op3_manager
    std::string manager_name = "/op3_manager";
    while (ros::ok())
    {
        ros::Duration(1.0).sleep();

        if (checkManagerRunning(manager_name) == true)
        {
        break;
        ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
        }
        ROS_WARN("Waiting for op3 manager");
    }

    readyToDemo();
    ros::Duration(5.0).sleep();

    setModule("head_control_module");
    head_angle_msg.name.push_back("head_pan");
    head_angle_msg.position.push_back(0);
    head_angle_msg.name.push_back("head_tilt");
    head_angle_msg.position.push_back(0);

    write_head_joint_pub.publish(head_angle_msg);

    //node loop
    sensor_msgs::JointState write_msg;
    write_msg.header.stamp = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();
        setModule("head_control_module");
        tracking();
    }
  
    return 0;
}

void tracking(){
  double xerror = 0;
  double yerror = 0;

  if (ball_position.x != 999 || ball_position.y != 999) {
    xerror = (320 - ball_position.x) * 70 / 320; //-atan(ball_position.x * tan(FOV_WIDTH));
    yerror = (240 - ball_position.y) * 70 / 240; //-atan(ball_position.y * tan(FOV_HEIGHT));

    std::cout << "xerror: " << xerror << std::endl;
    std::cout << "yerror: " << yerror << std::endl;

    if (xerror >= 10 || xerror <= -10){
      xerror = xerror * M_PI / 180;
      yerror = yerror * M_PI / 180;

      ros::Time curr_time = ros::Time::now();
      ros::Duration dur = curr_time - prev_time;

      double delta_time = dur.nsec * 0.000000001 + dur.sec;
      prev_time = curr_time;

      double xerror_dif = (xerror - current_ball_pan) / delta_time;
      double yerror_dif = (yerror - current_ball_tilt) / delta_time;

      xerror_sum += xerror;
      yerror_sum += yerror;

      double xerror_target = xerror * p_gain + xerror_dif * d_gain + xerror_sum * i_gain;
      double yerror_target = yerror * p_gain + yerror_dif * d_gain + yerror_sum * i_gain;

      publishHeadJoint(xerror_target, yerror_target);

      current_ball_pan = xerror;
      current_ball_tilt = yerror;
    }
  }
}

void publishHeadJoint(double pan, double tilt){

    if (pan >= 1.2217) pan = 1.2217;            //70 deg
    else if (pan <= -1.2217) pan = -1.2217;     //-70 deg
    

    if (tilt >= 0.34906) tilt = 0.34906;        //20 deg
    else if (tilt <= -1.2217) tilt = -1.2217;   //-70 deg

    head_angle_msg.name.push_back("head_pan");
    head_angle_msg.position.push_back(pan);
    head_angle_msg.name.push_back("head_tilt");
    head_angle_msg.position.push_back(tilt);

    write_head_joint_offset_pub.publish(head_angle_msg);
}

// void tracking(){
//   double xerror = 0;
//   double yerror = 0;

//   if (ball_position.x != 999 || ball_position.y != 999) {
//     xerror = (320 - ball_position.x) * 70 / 320; //-atan(ball_position.x * tan(FOV_WIDTH));
//     yerror = (240 - ball_position.y) * 70 / 240; //-atan(ball_position.y * tan(FOV_HEIGHT));

//     std::cout << "xerror: " << xerror << std::endl;
//     std::cout << "yerror: " << yerror << std::endl;

//     if (xerror >= 10 || xerror <= -10){
//       xerror = xerror * M_PI / 180;
  
//       ros::Time curr_time = ros::Time::now();
//       ros::Duration dur = curr_time - prev_time;

//       double delta_time = dur.nsec * 0.000000001 + dur.sec;
//       prev_time = curr_time;

//       double xerror_dif = (xerror - current_ball_pan) / delta_time;
//       xerror_sum += xerror;
//       double xerror_target = xerror * p_gain + xerror_dif * d_gain + xerror_sum * i_gain;
      
//       publishHeadJoint(xerror_target, NULL);

//       current_ball_pan = xerror;
//     }
//     if (yerror >= 10 || yerror <= -10){
//       yerror = yerror * M_PI / 180;

//       ros::Time curr_time = ros::Time::now();
//       ros::Duration dur = curr_time - prev_time;

//       double delta_time = dur.nsec * 0.000000001 + dur.sec;
//       prev_time = curr_time;

//       double yerror_dif = (yerror - current_ball_tilt) / delta_time;
//       yerror_sum += yerror;
//       double yerror_target = yerror * p_gain + yerror_dif * d_gain + yerror_sum * i_gain;

//       publishHeadJoint(NULL, yerror_target);

//       current_ball_tilt = yerror;
//     }
//   }
// }

// void publishHeadJoint(double pan, double tilt){
//   if (pan != NULL) {
//     if (pan >= 1.2217) pan = 1.2217;            //70 deg
//     else if (pan <= -1.2217) pan = -1.2217;     //-70 deg
    
//     head_angle_msg.name.push_back("head_pan");
//     head_angle_msg.position.push_back(pan);
//   }else{
//     std::cout << "pan is null" << std::endl;
//   }
//   if (tilt != NULL) {
//     if (tilt >= 0.34906) tilt = 0.34906;        //20 deg
//     else if (tilt <= -1.2217) tilt = -1.2217;   //-70 deg

//     head_angle_msg.name.push_back("head_tilt");
//     head_angle_msg.position.push_back(tilt);
//   }else{
//     std::cout << "tilt is null" << std::endl;
//   }
//   write_head_joint_offset_pub.publish(head_angle_msg);
// }

void readyToDemo()
{
  ROS_INFO("Start read-write demo");
  torqueOnAll();
  ROS_INFO("Torque on all joints");

  //send message for going init posture
  goInitPose();
  ROS_INFO("Go init pose");

  //wait while ROBOTIS-OP3 goes to the init posture.
  ros::Duration(4.0).sleep();

  setModule("none");
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub.publish(init_msg);
}

void goAction(int page) 
{
  setModule("action_module");
  ROS_INFO("Action pose");

  std_msgs::Int32 action_msg;
  action_msg.data = page;
  action_pose_pub.publish(action_msg);
}

bool checkManagerRunning(std::string& manager_name) 
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }
  ROS_ERROR("Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name) 
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }
  return ;
}

void torqueOnAll() 
{
  std_msgs::String check_msg;
  check_msg.data = "check";
  dxl_torque_pub.publish(check_msg);
}

bool isActionRunning() 
{
  op3_action_module_msgs::IsRunning is_running_srv;

  if (is_running_client.call(is_running_srv) == false) {
    ROS_ERROR("Failed to start action module");
    return true;
  } else {
    if (is_running_srv.response.is_running == true) {
      return true;
    }
  }
  return false;
}

void callbackBallCenter(const geometry_msgs::Point& msg)
{
  ball_position.x = msg.x; // 320) - 1;
  ball_position.y = msg.y; // 240) - 1;
}


