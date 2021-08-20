/**
  * orne-large Driver with Arduino
**/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

class MBDriver : public hardware_interface::RobotHW
{
    private:
        hardware_interface::JointStateInterface    jnt_state_interface_;
        hardware_interface::VelocityJointInterface jnt_vel_interface_;
        double cmd_vel_[2];
        double pos_[2];
        double vel_[2];
        double eff_[2];
        double max_velocity;
		
		ros::NodeHandle nh;
		ros::Publisher cmd_vel_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/orne_large_arduino/cmd_vel",1);
		ros::Subscriber joint_state_sub_ = nh.subscribe("/orne_large_arduino/joint_state", 1, &MBDriver::joint_state_callback, this);
    public:
        MBDriver() {
            ros::NodeHandle nh("~");
            if(nh.getParam("max_velocity", max_velocity)){
                ROS_INFO("~max_velocity: %f", max_velocity);
            }
            pos_[0] = pos_[1] = 0.0;
            vel_[0] = vel_[1] = 0.0;
            eff_[0] = eff_[1] = 0.0;
            cmd_vel_[0] = cmd_vel_[1] = 0.0;

            hardware_interface::JointStateHandle state_handle_1("left_wheel_hinge", &pos_[0], &vel_[0], &eff_[0]);
            jnt_state_interface_.registerHandle(state_handle_1);

            hardware_interface::JointStateHandle state_handle_2("right_wheel_hinge", &pos_[1], &vel_[1], &eff_[1]);
            jnt_state_interface_.registerHandle(state_handle_2);

            registerInterface(&jnt_state_interface_);

            hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("left_wheel_hinge"), &cmd_vel_[0]);
            jnt_vel_interface_.registerHandle(vel_handle_1);

            hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("right_wheel_hinge"), &cmd_vel_[1]);
            jnt_vel_interface_.registerHandle(vel_handle_2);

            registerInterface(&jnt_vel_interface_);

            ROS_INFO("COMPLETE");
        }

        void update(){
			std_msgs::Float32MultiArray cmd_vel;
			cmd_vel.data.push_back(std::max(std::min(cmd_vel_[0], max_velocity), -max_velocity));
			cmd_vel.data.push_back(std::max(std::min(cmd_vel_[1], max_velocity), -max_velocity));
			cmd_vel_pub_.publish(cmd_vel);
        }
		
		void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg){
			pos_[0] = msg->position[0];
			pos_[1] = msg->position[1];
			vel_[0] = msg->velocity[0];
			vel_[1] = msg->velocity[1];
		}
        
        ros::Time getTime() const{return ros::Time::now();}
        ros::Duration getPeriod() const{return ros::Duration(0.033);}
};

int main(int argc, char **argv){
    ros::init(argc, argv, "blm_driver_with_arduino"); 
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
