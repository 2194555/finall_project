#pragma once
//#ifndef HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
//#define HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H


#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"

namespace hero_chassis_controller{

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
  public:
    HeroChassisController() = default;

    ~HeroChassisController() override = default;

    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void setCmd(double cmd, int &n);

    //void getCmd(double & cmd);

    void starting(const ros::Time& time) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void getGains(double &p, double &i, double &d, double &i_max, double &i_min, int &n);

    void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup, int &n);

    void printDebug(int &n);

    void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false, int &n);

    void Kinematics_Init();

    void Kinematics_Inverse(_Float32 linear_x, _Float32 linear_y, _Float32 angular_z);

    hardware_interface::JointHandle joint_[4];

    double cmd_[4],cmds_;
    
    private:
        int loop_count_;

        _Float32 perimeter,xpy,v_w[4],D_X,D_Y,p[4],i[4],d[4],WHEEL_DIAMETER;

        ros::Time last_change_;

        control_toolbox::Pid pid_controller_[4];

        std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_[4] ;

        ros::Subscriber sub_command_;

        void setCmdCallback(geometry_msgs::Twist& cmd);
    };
}

//#endif