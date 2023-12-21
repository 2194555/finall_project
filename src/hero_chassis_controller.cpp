#define PI 3.1415926
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller{

HeroChassisController::HeroChassisController()
: cmds_(0),loop_count_(0),perimeter(0),WHEEL_DIAMETER(0) {}

HeroChassisController::~HeroChassisController()  {}

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

    for(int i = 0; i < 4; i++)
        if (!pid_controller_[i].init(ros::NodeHandle(root_nh, "pid")))
            return false;
    if (!root_nh.getParam("front_left_joint_/pid/p",p[0]));
        return false;
    if (!root_nh.getParam("front_left_joint_/pid/i",i[0]));
        return false;
    if (!root_nh.getParam("front_left_joint_/pid/d",d[0]));
        return false;
    if (!root_nh.getParam("front_right_joint_/pid/p",p[1]));
        return false;
    if (!root_nh.getParam("front_right_joint_/pid/i",i[1]));
        return false;
    if (!root_nh.getParam("front_right_joint_/pid/d",d[1]));
        return false;
    if (!root_nh.getParam("back_left_joint_/pid/p",p[2]));
        return false;
    if (!root_nh.getParam("back_left_joint_/pid/i",i[2]));
        return false;
    if (!root_nh.getParam("back_left_joint_/pid/d",d[2]));
        return false;
    if (!root_nh.getParam("back_right_joint_/pid/p",p[3]));
        return false;
    if (!root_nh.getParam("back_right_joint_/pid/i",i[3]));
        return false;
    if (!root_nh.getParam("back_right_joint_/pid/d",d[3]));
        return false;

    controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
    (root_nh, "state", 1));

    sub_command_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::setCmdCallback, this);

    return true;
}
void HeroChassisController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup, int &n){
  pid_controller_[n].setGains(p,i,d,i_max,i_min,antiwindup);
}

void HeroChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup, int &n){
  pid_controller_[n].getGains(p,i,d,i_max,i_min,antiwindup);
}

void HeroChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, int &n){
  bool dummy;
  pid_controller_[n].getGains(p,i,d,i_max,i_min,dummy);
}

void HeroChassisController::printDebug(int &n){
  pid_controller_[n].printValues();
}

void HeroChassisController::setCmd(double cmd, int &n){
  cmd_[n] = cmd;
}

/*void HeroChassisController::getCmd(double& cmd){
  //正运动学回推底盘速度
}*/

void HeroChassisController::starting(const ros::Time& time){
  cmds_ = 0.0;
  for(int i=0; i<4; i++)
   pid_controller_[i].reset();
  
}

void HeroChassisController::Kinematics_Init(){
    perimeter = (float)(WHEEL_DIAMETER*PI);
    
    _Float32 r_x = D_X/2;
    _Float32 r_y = D_Y/2;
    xpy = (r_x + r_y);
}

void HeroChassisController::Kinematics_Inverse(_Float32 linear_x, _Float32 linear_y, _Float32 angular_z){
	_Float32 v_tx   = linear_x;
	_Float32 v_ty   = linear_y;
	_Float32 omega = angular_z;
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
    commanded_effort[0] = pid_controller_[0].computeCommand(error[0], period);
    commanded_effort[1] = pid_controller_[1].computeCommand(error[1], period);
    commanded_effort[2] = pid_controller_[2].computeCommand(error[2], period);
    commanded_effort[3] = pid_controller_[3].computeCommand(error[3], period);

    front_left_joint_.setCommand(commanded_effort[0]);
    front_right_joint_.setCommand(commanded_effort[1]);
    back_left_joint_.setCommand(commanded_effort[2]);
    back_right_joint_.setCommand(commanded_effort[3]);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}