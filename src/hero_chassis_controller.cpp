#define PI 3.1415926
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller{

HeroChassisController::HeroChassisController()
: cmd_(0),loop_count(0)  {}

HeroChassisController::~HeroChassisController(){
    sub_commond.shutdown();
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
    // 获取关节
    front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ = 
      effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

    if (!pid_controller_.init(ros::NodeHandle(root_nh, "pid")))
        return false;

    controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
    (root_nh, "state", 1));

    sub_command_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::setCmdCallback, this);

    return true;
}
void HeroChassisController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup){
  pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
}

void HeroChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup){
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void HeroChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min){
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

void HeroChassisController::printDebug(){
  pid_controller_.printValues();
}

std::string HeroChassisController::getJointName(){
  return joint_.getName();
}

void HeroChassisController::setCommand(double cmd){
  cmds_ = cmd;
}

void HeroChassisController::getCommand(double& cmd){
  cmd = cmds_;
}

void HeroChassisController::starting(const ros::Time& time){
  cmds_ = 0.0;
  pid_controller_.reset();
}

void Kinematics_Init(void){
    perimeter = (float)(WHEEL_DIAMETER*PI);
    
    _Float32 r_x = D_X/2;
    _Float32 r_y = D_Y/2;
    xpy = (r_x + r_y);
}

void Kinematics_Inverse(_Float32 linear_x, _Float32 linear_y, _Float32 angular_z){
	_Float32 v_tx   = linear_x;
	_Float32 v_ty   = linear_y;
	_Float32 omega = _Float32 angular_z;
	v_w[4] = {0};
	
	v_w[0] = v_tx - v_ty - xpy*omega;
	v_w[1] = v_tx + v_ty + xpy*omega;
	v_w[2] = v_tx + v_ty - xpy*omega;
	v_w[3] = v_tx - v_ty + xpy*omega;
}

void HeroChassisController::setCmdCallback(geometry_msgs::Twist& cmd){
    Kinematics_Inverse(cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period){
    double error[4] = {0};
    error[0] = v_w[0] - front_left_joint_.getVelocity();
    error[1] = v_w[1] - front_right_joint_.getVelocity();
    error[2] = v_w[2] - back_left_joint_.getVelocity();
    error[3] = v_w[3] - back_right_joint_.getVelocity();

    double commanded_effort[4];
    commanded_effort[0] = pid_controller_.computeCommand(error[0], period);
    commanded_effort[1] = pid_controller_.computeCommand(error[1], period);
    commanded_effort[2] = pid_controller_.computeCommand(error[2], period);
    commanded_effort[3] = pid_controller_.computeCommand(error[3], period);

    front_left_joint_.setCommand(commanded_effort[0]);
    front_right_joint_.setCommand(commanded_effort[1]);
    back_left_joint_.setCommand(commanded_effort[2]);
    back_right_joint_.setCommand(commanded_effort[3]);
}
