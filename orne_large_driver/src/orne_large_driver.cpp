/**
  * orne-large Driver
**/

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <cmath>
#include "orne_large_driver/blm.h"

class MBDriver : public hardware_interface::RobotHW
{
    private:
        hardware_interface::JointStateInterface    jnt_state_interface_;
        hardware_interface::VelocityJointInterface jnt_vel_interface_;
        double cmd_[2];
        double pos_[2];
        double vel_[2];
        double last_vel_[2];
        double eff_[2];
        double dir_[2];
        double prev_dir_[2];
        BLMData blm;
        ros::Time previous_time_;

        double max_velocity;
    public:
        MBDriver() {
            ros::NodeHandle nh("~");
            std::string serial_port;
            nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
            ROS_INFO("serial_port: %s", serial_port.c_str());
            if(BLM_Init(&blm, serial_port.c_str()) < 0){
                ROS_ERROR("could not init blm \n");
                exit(0);
            }
            if(nh.getParam("max_velocity", max_velocity)){
                ROS_INFO("max_velocity: %f", max_velocity);
            }

            pos_[0] = pos_[1] = 0.0;
            vel_[0] = vel_[1] = 0.0;
            last_vel_[0] = last_vel_[1] = 0.0;
            eff_[0] = eff_[1] = 0.0;
            cmd_[0] = cmd_[1] = 0.0;
            dir_[0] = dir_[1] = 0.0;
            prev_dir_[0] = prev_dir_[1] = 0.0;
            previous_time_ = ros::Time::now();

            hardware_interface::JointStateHandle state_handle_1("left_wheel_hinge", &pos_[0], &vel_[0], &eff_[0]);
            jnt_state_interface_.registerHandle(state_handle_1);

            hardware_interface::JointStateHandle state_handle_2("right_wheel_hinge", &pos_[1], &vel_[1], &eff_[1]);
            jnt_state_interface_.registerHandle(state_handle_2);

            registerInterface(&jnt_state_interface_);

            hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("left_wheel_hinge"), &cmd_[0]);
            jnt_vel_interface_.registerHandle(vel_handle_1);

            hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("right_wheel_hinge"), &cmd_[1]);
            jnt_vel_interface_.registerHandle(vel_handle_2);

            registerInterface(&jnt_vel_interface_);

            ROS_INFO("COMPLETE");
        }

        void update(){
            BLM_Set_Angular_Velocity_RAD_PER_SEC(&blm, 1, std::min(fabs(cmd_[0]), max_velocity));
            BLM_Set_Angular_Velocity_RAD_PER_SEC(&blm, 2, std::min(fabs(cmd_[1]), max_velocity));

            if (cmd_[0] != 0.0) dir_[0] = cmd_[0];
            if (cmd_[1] != 0.0) dir_[1] = cmd_[1];
            if (prev_dir_[0] * dir_[0] <= 0.0) BLM_Change_Mode(&blm, 1,   dir_[0]);
            if (prev_dir_[1] * dir_[1] <= 0.0) BLM_Change_Mode(&blm, 2,  -dir_[1]);

            prev_dir_[0] = dir_[0];
            prev_dir_[1] = dir_[1];
            
            double ang_vel_rad_per_sec = 0.0;
            BLM_Read_Current_Angular_Velocity_RAD_PER_SEC(&blm, 1, &ang_vel_rad_per_sec);
            if(std::abs(ang_vel_rad_per_sec) < max_velocity)
            {
                last_vel_[0] = vel_[0];
                vel_[0] =   ang_vel_rad_per_sec;
            }
            else
            {
                vel_[0] = last_vel_[0];
            }
            BLM_Read_Current_Angular_Velocity_RAD_PER_SEC(&blm, 2, &ang_vel_rad_per_sec);
            if(std::abs(ang_vel_rad_per_sec) < max_velocity)
            {
                last_vel_[1] = vel_[1];
                vel_[1] = -ang_vel_rad_per_sec;
            }
            else
            {
                vel_[1] = last_vel_[1];
            }
            ros::Duration duration  = ros::Time::now() - previous_time_;
            pos_[0] += vel_[0] * duration.toSec();
            pos_[1] += vel_[1] * duration.toSec();
            previous_time_ = ros::Time::now();
        }
        
        ~MBDriver(){
            BLM_Close(&blm);
        }

        ros::Time getTime() const{return ros::Time::now();}
        ros::Duration getPeriod() const{return ros::Duration(0.10);}
};

int main(int argc, char **argv){
    ros::init(argc, argv, "blmc_driver"); 
    ros::NodeHandle nh;

    MBDriver robot;

    controller_manager::ControllerManager cm(&robot, nh);

    ros::Rate rate(1.0/robot.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
        cm.update(robot.getTime(), robot.getPeriod());
        robot.update();
        rate.sleep();
    }
    spinner.stop();

    return 0;

}
