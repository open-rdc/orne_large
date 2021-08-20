#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "debug.h"
#include "orne_large_arduino.h"
#include "BLMMotor.h"

const int joint_num = 2;
ros::NodeHandle  nh;
sensor_msgs::JointState joint_state;
BLMMotor blm_motor;

char *joint_name[joint_num] = {"left_hinge", "left_hinge"};
float joint_pos[joint_num], joint_vel[joint_num], joint_eff[joint_num];
enum {
  LEFT = 0,
  RIGHT
};

/**
 * @brief cmd_vel 速度指令のコールバック関数
 * @detail トピックはFloat32MultiArray型．[pitch, yaw, length]
*/
void cmdVelCallBack(const std_msgs::Float32MultiArray &msg){
    float left_target_vel_rad_p_s  = msg.data[0];
    float right_target_vel_rad_p_s = msg.data[1];
    blm_motor.setTargetAngularVelocities(left_target_vel_rad_p_s, right_target_vel_rad_p_s);
}

//! cmd_vel 速度指令のサブスクライバの定義
ros::Subscriber<std_msgs::Float32MultiArray> cmd_vel_sub("~cmd_vel", cmdVelCallBack);

// 各軸の状態を配信するパブリッシャの定義
ros::Publisher joint_state_pub("~joint_state", &joint_state);
/**
 * @brief 各軸の状態を配信するためのデータを持つオブジェクトを初期化する関数
*/
void initPublishArray(){
    joint_state.name_length     = joint_num;
    joint_state.velocity_length = joint_num;
    joint_state.position_length = joint_num;
    joint_state.effort_length   = joint_num;
    joint_state.name            = joint_name;
    joint_state.position        = joint_pos;
    joint_state.velocity        = joint_vel;
    joint_state.effort          = joint_eff;
}

/**
 * @brief Arduinoが起動すると最初に実行される関数
*/

void setup()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    initPublishArray();
    nh.subscribe(cmd_vel_sub);
    nh.advertise(joint_state_pub);

    while(!nh.connected()) {
        nh.spinOnce();
        delay(10);
    }
    delay(500);
}

/**
 * @brief Arduinoのメインループ
*/
void loop()
{
    blm_motor.measureCurrentAngles();
    double left_current_angle_rad, right_current_angle_rad;
    double left_current_vel_rad_p_s, right_current_vel_rad_p_s;
    blm_motor.getCurrentAngles(&left_current_angle_rad, &right_current_angle_rad);
    blm_motor.getCurrentAngularVelocities(&left_current_vel_rad_p_s, &right_current_vel_rad_p_s);
    joint_pos[0] =  left_current_angle_rad;
    joint_pos[1] = -right_current_angle_rad;
    joint_vel[0] =  left_current_vel_rad_p_s;
    joint_vel[1] = -right_current_vel_rad_p_s;
    joint_state.header.stamp = nh.now();
    joint_state_pub.publish(&joint_state);
    
    // rosserial_arduinoではspinOnceするとサブスクライバやらサービスやらが全部うごく
    nh.spinOnce();
    delay(20);
}
