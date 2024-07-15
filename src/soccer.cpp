/* 
Authors: Pedro Deniz
         Marlene Cobian 
*/

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Eigen>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/WalkingParam.h"

//default
void readyToDemo();
void setModule(const std::string& module_name);
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();

//basics
void goInitPose();
void goAction(int page);
void goWalk(std::string& command);
void callbackImu(const sensor_msgs::Imu::ConstPtr& msg);
//void callbackJointStates(const sensor_msgs::JointState& msg);

//services
bool isActionRunning();
bool getWalkingParam();
bool getJointPose();
void waitFollowing();

//walking
void calcFootstep(double target_distance, double target_angle, double delta_time, double& fb_move, double& rl_angle);
void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
bool processFollowing();
void waitFollowing();

//looking
void callbackBallCenter(const geometry_msgs::Point& msg);
bool tracking();
void search();
void turn();
void turn_search();
void publishHeadJoint(double pan, double tilt);

//imu
double alpha = 0.4;
double pitch;
double rpy_orientation;
const double FALL_FORWARD_LIMIT = 55;
const double FALL_BACK_LIMIT = -55;
double present_pitch_;

//kicking
double distance_to_ball = 0.0;
int page;
int state;

//standing up txt
const int rows = 40;
const int cols = 6;
float positions[rows][cols];

//walking
double rest_inc_giro = 0.08726;
const double SPOT_FB_OFFSET = 0.0;
const double SPOT_RL_OFFSET = 0.0;
const double SPOT_ANGLE_OFFSET = 0.0;
double current_x_move_ = 0.005;
double current_r_angle_ = 0.0;
const double IN_PLACE_FB_STEP = -0.003;
const int NOT_FOUND_THRESHOLD = 0.5;
int count_not_found_ = 0;
int count_to_kick_ = 0;
const double CAMERA_HEIGHT = 0.46;
const double FOV_WIDTH = 35.2 * M_PI / 180;
const double FOV_HEIGHT = 21.6 * M_PI / 180;
const double hip_pitch_offset_ = 0.12217305; //7°
const double UNIT_FB_STEP = 0.002;
const double UNIT_RL_TURN = 0.00872665;      //0.5°
const double MAX_FB_STEP = 0.007;
const double MAX_RL_TURN =  0.26179939;      //15°
const double MIN_FB_STEP = 0.003;
const double MIN_RL_TURN = 0.08726646;       //5°
double accum_period_time = 0.0;
double current_period_time = 0.6;
double distance_to_kick = 0.22;  
double delta_time_w;
std::string command;

//looking
geometry_msgs::Point ball_position;
sensor_msgs::JointState head_angle_msg;

double current_ball_pan = 0;
double current_ball_tilt = 0;
double xerror_sum = 0;
double yerror_sum = 0;

double p_gain = 0.105;   //0.6
double d_gain = 0.0012;  //0.045
double i_gain = 0.00001;

ros::Time prev_time_walk;
ros::Time prev_time;
ros::Time start_time(0);
bool end_time = false;
ros::Time now_time(0);
ros::Time start_search(0);
bool end_search = 0;
ros::Time now_search(0);

double x_target = 0;
double y_target = 0;
double t = 0;
double head_direction = 1;

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;
int control_module = None;
bool demo_ready = false;

ros::Publisher init_pose_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_head_joint_pub;
ros::Publisher write_head_joint_offset_pub;
ros::Publisher write_joint_pub;
ros::Publisher action_pose_pub;
ros::Publisher walk_command_pub;
ros::Publisher set_walking_param_pub;

ros::Subscriber read_joint_sub;
ros::Subscriber ball_sub;
ros::Subscriber imu_sub;

ros::ServiceClient set_joint_module_client;
ros::ServiceClient is_running_client;
ros::ServiceClient get_param_client;

op3_walking_module_msgs::WalkingParam current_walking_param;

//node main
int main(int argc, char **argv) {
    //init ros
    ros::init(argc, argv, "soccer");
    ros::NodeHandle nh(ros::this_node::getName());

    int robot_id;
    nh.param<int>("robot_id", robot_id, 0);
    
    std::ifstream myfile ("/home/robotis/blenders_ws/src/soccer_pkg/data/Pararse.txt");
    if (myfile.is_open()) {
    	std::cout << "El archivo se abrió";

	for (int idx2 = 0; idx2 < rows; idx2++){
		for (int idy2 = 0; idy2 < cols; idy2++){
			myfile >> positions[idx2][idy2];
		}
		
	}
	myfile.close();
    } else {
	std::cout << "El archivo no abrió";
    }

    //subscribers
    //read_joint_sub = nh.subscribe("/robotis/present_joint_states",1, callbackJointStates);
    ball_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/ball_center",5, callbackBallCenter);
    imu_sub = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/open_cr/imu",1, callbackImu);

    //publishers
    init_pose_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/base/ini_pose",0);
    dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/dxl_torque",0);
    write_head_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/head_control/set_joint_states",0);
    write_head_joint_offset_pub = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/head_control/set_joint_states_offset",0);
    action_pose_pub = nh.advertise<std_msgs::Int32>("/robotis_" + std::to_string(robot_id) + "/action/page_num",0);
    walk_command_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/walking/command",0);
    set_walking_param_pub = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/set_params",0);
    
    //services
    set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis_" + std::to_string(robot_id) + "/set_present_ctrl_modules");
    is_running_client = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis_" + std::to_string(robot_id) + "/action/is_running");
    get_param_client = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/get_params");

    ros::start();
    ros::Rate loop_rate(SPIN_RATE);
	
    //wait for starting of op3_manager
    std::string manager_name = "/op3_manager";
	
    while (ros::ok()) {
        ros::Duration(1.0).sleep();
	
	if (checkManagerRunning(manager_name) == true) {
	    break;
	    ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
	}
	ROS_WARN("Waiting for op3 manager");
    }
    readyToDemo();
	
    //node loop
    sensor_msgs::JointState write_msg;
    write_msg.header.stamp = ros::Time::now();
    
    //standing up
    ros::Duration(5.0).sleep();
    goAction(9);
    ros::Duration(5.0).sleep();
    ros::Time prev_time_walk = ros::Time::now();
    ros::Time prev_time = ros::Time::now();

    while (ros::ok()) {
        ros::Rate loop_rate(SPIN_RATE);
        ros::spinOnce();
        
        if (ball_position.x != 999 && ball_position.y != 999) {
	    setModule("head_control_module");
            if (tracking()) {
                ros::Duration(1.0).sleep();
                setModule("walking_module");
                ros::Time curr_time_walk = ros::Time::now();
                ros::Duration dur_w = curr_time_walk - prev_time_walk;
                double delta_time_w = dur_w.nsec * 0.000000001 + dur_w.sec;
                prev_time_walk = curr_time_walk;

                count_not_found_ = 0;

                distance_to_ball = CAMERA_HEIGHT * tan(M_PI * 0.5 + current_ball_tilt - hip_pitch_offset_);

                if (distance_to_ball < 0) {
                    distance_to_ball *= (-1);
                }

                
                std::cout << distance_to_ball << std::endl;
                if ((distance_to_ball < distance_to_kick)) { //&& (fabs(ball_x_angle) < 25.0) to kick
                    count_to_kick_ += 1;	
                    std::cout << count_to_kick_ << std::endl;
                    if (count_to_kick_ > 20) {
                        std::string command = "stop";
                        goWalk(command);
                        if (current_ball_pan > 0) { 
                            std::cout << "PATEA DERECHA" << std::endl;
                            goAction(84); //left kick
                        }
                        else { 
                            std::cout << "PATEA IZQUIERDA" << std::endl;
                            goAction(83); //right kick
                        }
                    } else if (count_to_kick_ > 15) {
                    getWalkingParam();
                    setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);

                    std_msgs::String command_msg;
                    command_msg.data = "start";
                    walk_command_pub.publish(command_msg);
                    }
                }
            } else {
                count_to_kick_ = 0;
            }

            double fb_move = 0.0, rl_angle = 0.0;
            double distance_to_walk = distance_to_ball - distance_to_kick;

            calcFootstep(distance_to_walk, current_ball_pan, delta_time_w, fb_move, rl_angle);

            getWalkingParam();
            setWalkingParam(fb_move, 0, rl_angle, true);
            
            std_msgs::String command_msg;
            command_msg.data = "start";
            walk_command_pub.publish(command_msg);
        } else {
            std::string command = "stop";
            goWalk(command);
            //ros::Duration(1.5).sleep();
            setModule("head_control_module");
            search();
        }
    }
    return 0;
}

void callbackBallCenter(const geometry_msgs::Point& msg) {
    ball_position.x = msg.x; 
    ball_position.y = msg.y; 
}

void turn() {
    t += 1 * head_direction;

    if (t >= 360)
        head_direction = -1;
    else if (t <= -360)
        head_direction = 1;

    x_target = 60*sin(t);
    y_target = 15*cos(1*-t*head_direction) - 30;

    x_target = (x_target*M_PI)/180;
    y_target = (y_target*M_PI)/180;

    publishHeadJoint(x_target, y_target);

    if (!start_search.toSec())
        start_search =  ros::Time::now();
    if (!end_search) {
        if ((now_search.toSec() - start_search.toSec()) > 15)
            end_search = true;
        
        now_search = ros::Time::now();
    } else {
        turn_search();
        ros::Duration(1.0).sleep();
        start_search = ros::Time(0);
        end_search = false;
        now_search = ros::Time(0);
    }
}

void search() {
    if (!start_time.toSec()) {
        start_time = ros::Time::now();
    } 
    if (!end_time) {
        if ((now_time.toSec() - start_time.toSec()) > 3) {
            end_time = true;
        } 
        now_time = ros::Time::now();
    } else {
        start_time = ros::Time(0);
        end_time = false;
        now_time = ros::Time(0);
    } 
    turn();
}

bool tracking() {
    double xerror = 0;
    double yerror = 0;
    xerror = (320 - ball_position.x) * 70 / 320; //-atan(ball_position.x * tan(FOV_WIDTH));
    yerror = (240 - ball_position.y) * 70 / 240; //-atan(ball_position.y * tan(FOV_HEIGHT));

    //std::cout << "xerror: " << xerror << std::endl;
    //std::cout << "yerror: " << yerror << std::endl;

    if (xerror >= 10 || xerror <= -10) {
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
        return false;
    } else {
        return true;
    }
}

void publishHeadJoint(double pan, double tilt) {
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

void turn_search(){
  //node loop
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();
  
  for (int i = 1; i <= 6; i++)
  {
    setModule("none");
    //std::cout  << "Derechaaaa D:" << std::endl;
    //Levantar pie derecho 
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(-0.7091);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(-1.4131);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(0.7091 + rest_inc_giro);
    write_msg.name.push_back("r_hip_yaw");
    write_msg.position.push_back(0.1746*1.5);
    write_msg.name.push_back("l_hip_yaw");
    write_msg.position.push_back(-0.1746*1.5);

    write_msg.name.push_back("l_hip_roll");
    write_msg.position.push_back(-0.0873);
    write_msg.name.push_back("r_hip_roll");
    write_msg.position.push_back(0.0873);
    write_msg.name.push_back("l_ank_roll");
    write_msg.position.push_back(-0.0873);
    write_msg.name.push_back("r_ank_roll");
    write_msg.position.push_back(0.0873);
    write_joint_pub.publish(write_msg);

    //Bajar pie derecho
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(positions[rows-1][0]);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(positions[rows-1][1]);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(positions[rows-1][2] + rest_inc_giro);
    write_joint_pub.publish(write_msg);
    
    //Levantar pie izquierdo
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(0.7091);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(1.4131);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(-0.7091 - rest_inc_giro);
    write_msg.name.push_back("r_hip_yaw");
    write_msg.position.push_back(0);
    write_msg.name.push_back("l_hip_yaw");
    write_msg.position.push_back(0);
    write_joint_pub.publish(write_msg);
    
    //Bajar pie izquierdo
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(positions[rows-1][3]);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(positions[rows-1][4]);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(positions[rows-1][5] - rest_inc_giro);
    write_joint_pub.publish(write_msg);
  }
}

void waitFollowing() {
    count_not_found_++;

    if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
        setWalkingParam(0.0, 0.0, 0.0);
}

void calcFootstep(double target_distance, double target_angle, double delta_time, double& fb_move, double& rl_angle) {
    double next_movement = current_x_move_;
    if (target_distance < 0)
        target_distance = 0.0;

    double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP);
    accum_period_time += delta_time;
    if (accum_period_time > (current_period_time  / 4)) {
        accum_period_time = 0.0;
        if ((target_distance * 0.1 / 2) < current_x_move_)
            next_movement -= UNIT_FB_STEP;
        else
            next_movement += UNIT_FB_STEP;
    }
    fb_goal = fmin(next_movement, fb_goal);
    fb_move = fmax(fb_goal, MIN_FB_STEP);

    double rl_goal = 0.0;
    if (fabs(target_angle) * 180 / M_PI > 5.0) {
        double rl_offset = fabs(target_angle) * 0.2;
        rl_goal = fmin(rl_offset, MAX_RL_TURN);
        rl_goal = fmax(rl_goal, MIN_RL_TURN);
        rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

        if (target_angle < 0)
            rl_angle *= (-1);
    }
}

void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance) {
  current_walking_param.balance_enable = balance;
  current_walking_param.x_move_amplitude = x_move + SPOT_FB_OFFSET;
  current_walking_param.y_move_amplitude = y_move + SPOT_RL_OFFSET;
  current_walking_param.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

  set_walking_param_pub.publish(current_walking_param);

  current_x_move_ = x_move;
  current_r_angle_ = rotation_angle;
}

bool getWalkingParam() {
  
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if (get_param_client.call(walking_param_msg)) {
    current_walking_param = walking_param_msg.response.parameters;

    // update ui
    ROS_INFO_COND(DEBUG_PRINT, "Get walking parameters");

    return true;
  }
  else {
    ROS_ERROR("Fail to get walking parameters.");

    return false;
  }
}

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg) {
  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / 3.141516);
  
  double pitch = rpy_orientation.coeff(1, 0);

  if (present_pitch_ == 0) 
    present_pitch_ = pitch;
  else
    present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  if (present_pitch_ > FALL_FORWARD_LIMIT) {
    goAction(122);
    setModule("none");
  } else if (present_pitch_ < FALL_BACK_LIMIT) {
    goAction(1);
    setModule("none");
    ros::Duration(1).sleep();
    goAction(82);
    setModule("none");
  } else {
    state = 0;
  }
}

void goInitPose() {
    std_msgs::String init_msg;
    init_msg.data = "ini_pose";
    init_pose_pub.publish(init_msg);
}

void goAction(int page) {
    setModule("action_module");
    ROS_INFO("Action pose");

    std_msgs::Int32 action_msg;
    action_msg.data = page;
    action_pose_pub.publish(action_msg);
}

void goWalk(std::string& command) {
    setModule("walking_module");
    if (command == "start") {
        getWalkingParam();
        setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);
    }

    std_msgs::String command_msg;
    command_msg.data = command;
    walk_command_pub.publish(command_msg);
}

void readyToDemo() {
    ROS_INFO("Start read-write demo");
    torqueOnAll();
    ROS_INFO("Torque on all joints");

    //send message for going init posture
    goInitPose();
    ROS_INFO("Go init pose");

    //wait while ROBOTIS-OP3 goes to the init posture
    ros::Duration(4.0).sleep();

    setModule("none");
}

bool checkManagerRunning(std::string& manager_name) {
    std::vector<std::string> node_list;
    ros::master::getNodes(node_list);

    for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++) {
        if (node_list[node_list_idx] == manager_name)
            return true;
    }
    ROS_ERROR("Can't find op3_manager");
    return false;
}

void setModule(const std::string& module_name) {
    robotis_controller_msgs::SetModule set_module_srv;
    set_module_srv.request.module_name = module_name;

    if (set_joint_module_client.call(set_module_srv) == false) {
        ROS_ERROR("Failed to set module");
        return;
    }
    return ;
}

void torqueOnAll() {
    std_msgs::String check_msg;
    check_msg.data = "check";
    dxl_torque_pub.publish(check_msg);
}

bool isActionRunning() {
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



