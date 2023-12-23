#define PI 3.1415926
#define I_MAX 1
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller{

HeroChassisController::HeroChassisController()
: cmds_(0),loop_count_(0),perimeter(0),WHEEL_DIAMETER(0),D_X(0.4),D_Y(0.6) {}

HeroChassisController::~HeroChassisController(){
    sub_command_.shutdown();
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
    // 获取关节
    joint_[0]=
      effort_joint_interface->getHandle("left_front_wheel_joint");
    joint_[1]=
      effort_joint_interface->getHandle("right_front_wheel_joint");
    joint_[2]= 
      effort_joint_interface->getHandle("left_back_wheel_joint");
    joint_[3]=
      effort_joint_interface->getHandle("right_back_wheel_joint");

    if (!pid_controller_[0].init(ros::NodeHandle(root_nh, "pid_fl")))
        return false;
    if (!pid_controller_[1].init(ros::NodeHandle(root_nh, "pid_fr")))
        return false;
    if (!pid_controller_[2].init(ros::NodeHandle(root_nh, "pid_bl")))
        return false;
    if (!pid_controller_[3].init(ros::NodeHandle(root_nh, "pid_br")))
        return false;
    if (!root_nh.getParam("hero/front_left_joint_/pid/p",p[0]));
        return false;
    if (!root_nh.getParam("hero/front_left_joint_/pid/i",i[0]));
        return false;
    if (!root_nh.getParam("hero/front_left_joint_/pid/d",d[0]));
        return false;
    if (!root_nh.getParam("hero/front_right_joint_/pid/p",p[1]));
        return false;
    if (!root_nh.getParam("hero/front_right_joint_/pid/i",i[1]));
        return false;
    if (!root_nh.getParam("hero/front_right_joint_/pid/d",d[1]));
        return false;
    if (!root_nh.getParam("hero/back_left_joint_/pid/p",p[2]));
        return false;
    if (!root_nh.getParam("hero/back_left_joint_/pid/i",i[2]));
        return false;
    if (!root_nh.getParam("hero/back_left_joint_/pid/d",d[2]));
        return false;
    if (!root_nh.getParam("hero/back_right_joint_/pid/p",p[3]));
        return false;
    if (!root_nh.getParam("hero/back_right_joint_/pid/i",i[3]));
        return false;
    if (!root_nh.getParam("hero/back_right_joint_/pid/d",d[3]));
        return false;
    for(int j = 0; j < 4; j++){
        controller_state_publisher_[j].reset(
        new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
        (root_nh, "state", 1));
        controller_state_publisher_[j]->msg_.p=p[j];
        controller_state_publisher_[j]->msg_.i=i[j];
        controller_state_publisher_[j]->msg_.d=d[j];
        controller_state_publisher_[j]->msg_.i_clamp=I_MAX;
    }

    Kinematics_Init();

    sub_command_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::setCmdCallback, this);

    return true;
}
void HeroChassisController::setGains(const double &p, const double &i, const double &d, 
                                    const double &i_max, const double &i_min, const bool &antiwindup, int &n){
  pid_controller_[n].setGains(p,i,d,i_max,i_min,antiwindup);
}

void HeroChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, 
                                    bool &antiwindup, int &n){
  pid_controller_[n].getGains(p,i,d,i_max,i_min,antiwindup);
}

void HeroChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, int &n){
  bool dummy;
  pid_controller_[n].getGains(p,i,d,i_max,i_min,dummy);
}

void HeroChassisController::printDebug(int &n){
  pid_controller_[n].printValues();
}

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
	
	v_w[0] = v_tx - v_ty - xpy*omega;
	v_w[1] = v_tx + v_ty + xpy*omega;
	v_w[2] = v_tx + v_ty - xpy*omega;
	v_w[3] = v_tx - v_ty + xpy*omega;
}

void HeroChassisController::setCmdCallback(geometry_msgs::Twist& cmd){
    Kinematics_Inverse(cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period){
    double error[4] = {0}, power=1;
    error[0] = v_w[0] - joint_[0].getVelocity();
    error[1] = v_w[1] - joint_[1].getVelocity();
    error[2] = v_w[2] - joint_[2].getVelocity();
    error[3] = v_w[3] - joint_[3].getVelocity();

    double commanded_effort[4];
    commanded_effort[0] = pid_controller_[0].computeCommand(error[0], period);
    commanded_effort[1] = pid_controller_[1].computeCommand(error[1], period);
    commanded_effort[2] = pid_controller_[2].computeCommand(error[2], period);
    commanded_effort[3] = pid_controller_[3].computeCommand(error[3], period);

    joint_[0].setCommand(power/commanded_effort[0]);
    joint_[1].setCommand(power/commanded_effort[1]);
    joint_[2].setCommand(power/commanded_effort[2]);
    joint_[3].setCommand(power/commanded_effort[3]);

    if(loop_count_ % 10 == 0){
        for(int n=0; n<4; n++){
            if(controller_state_publisher_[n] && controller_state_publisher_[n]->trylock()){
                controller_state_publisher_[n]->msg_.header.stamp = time;
                controller_state_publisher_[n]->msg_.process_value = joint_[n].getVelocity();
                controller_state_publisher_[n]->msg_.error = error[n];
                controller_state_publisher_[n]->msg_.time_step = period.toSec();
                controller_state_publisher_[n]->msg_.command = commanded_effort[n];

                double dummy;
                bool antiwindup;

                getGains(controller_state_publisher_[n]->msg_.p,
                controller_state_publisher_[n]->msg_.i,
                controller_state_publisher_[n]->msg_.d,
                controller_state_publisher_[n]->msg_.i_clamp,
                dummy,
                antiwindup,
                n);
                controller_state_publisher_[n]->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_[n]->unlockAndPublish();
            }

        }
    }
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}