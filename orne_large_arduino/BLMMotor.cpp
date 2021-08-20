#include "BLMMotor.h"
#include <Arduino.h>
#include <HardwareSerial.h>

const int LEFT_FORWARD_PIN  = 22;
const int LEFT_REVERSE_PIN  = 23;
const int LEFT_SPEED_PIN  = 44;
const int LEFT_PULSE_PIN  = 3;

const int RIGHT_FORWARD_PIN = 25;
const int RIGHT_REVERSE_PIN = 26;
const int RIGHT_SPEED_PIN = 45;
const int RIGHT_PULSE_PIN = 2;

int left_pulse  = 0;
int right_pulse = 0;

void left_pulse_interrupt() {
    left_pulse ++;
}

void right_pulse_interrupt() {
    right_pulse ++;
}

BLMMotor::BLMMotor() :
left_current_vel_rad_p_s_(0), right_current_vel_rad_p_s_(0),
    left_dir_(1), right_dir_(1), left_prev_pulse_(0), right_prev_pulse_(0) {
    TCCR5B = (TCCR5B & 0b11111000) | 1;
    pinMode(LEFT_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_REVERSE_PIN, OUTPUT);
    pinMode(RIGHT_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_REVERSE_PIN, OUTPUT);
    attachInterrupt(LEFT_PULSE_PIN , left_pulse_interrupt, RISING);
    attachInterrupt(RIGHT_PULSE_PIN, right_pulse_interrupt, RISING);
    prev_time_micro_s_ = micros();
}

BLMMotor::~BLMMotor() {
    digitalWrite(LEFT_FORWARD_PIN , LOW);
    digitalWrite(LEFT_REVERSE_PIN , LOW);
    digitalWrite(RIGHT_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_REVERSE_PIN, LOW);
}

const double GEAR_RATIO = 100.0;
const double MAX_RPM = 4000.0;
const double COM_MAX_VOL = 4.5;
const double AD_MAX_VOL = 4.5;

void BLMMotor::setTargetAngularVelocities(double left_target_vel_rad_p_s, double right_target_vel_rad_p_s) {
    left_target_vel_rad_p_s_  = left_target_vel_rad_p_s ;
    right_target_vel_rad_p_s_ = right_target_vel_rad_p_s;

    double left_motor_target_vel_rpm  = left_target_vel_rad_p_s  / (2.0 * M_PI) * GEAR_RATIO * 60.0;
    if (fabs(left_motor_target_vel_rpm)  < 80) left_motor_target_vel_rpm  = 0;
    double right_motor_target_vel_rpm = right_target_vel_rad_p_s / (2.0 * M_PI) * GEAR_RATIO * 60.0;
    if (fabs(right_motor_target_vel_rpm) < 80) right_motor_target_vel_rpm = 0;
    digitalWrite(LEFT_FORWARD_PIN, left_motor_target_vel_rpm > 0 ? HIGH : LOW);
    digitalWrite(LEFT_REVERSE_PIN, left_motor_target_vel_rpm > 0 ? LOW : HIGH);    
    analogWrite(LEFT_SPEED_PIN , min(fabs(left_motor_target_vel_rpm ) / MAX_RPM, 1.0) * 255 * COM_MAX_VOL / AD_MAX_VOL);
    digitalWrite(RIGHT_FORWARD_PIN, right_motor_target_vel_rpm > 0 ? LOW : HIGH);
    digitalWrite(RIGHT_REVERSE_PIN, right_motor_target_vel_rpm > 0 ? HIGH : LOW);
    analogWrite(RIGHT_SPEED_PIN, min(fabs(right_motor_target_vel_rpm) / MAX_RPM, 1.0) * 255 * COM_MAX_VOL / AD_MAX_VOL);
}

void BLMMotor::getCurrentAngularVelocities(double *left_current_vel_rad_p_s, double *right_current_vel_rad_p_s) {
    *left_current_vel_rad_p_s  = left_current_vel_rad_p_s_ ;
    *right_current_vel_rad_p_s = right_current_vel_rad_p_s_;
}

void BLMMotor::getCurrentAngles(double *left_current_ang_rad, double *right_current_ang_rad) {
    *left_current_ang_rad  = left_current_ang_rad_ ;
    *right_current_ang_rad = right_current_ang_rad_;
}

void BLMMotor::measureCurrentAngles() {
    static bool left_near_0 = false, right_near_0 = false;
    const double PULSE_PER_ROT = 30.0;
    const int THRESHOLD_CHANGE_DIR = 3;
    unsigned long current_time = micros();
    unsigned long diff_time_micro_s = current_time - prev_time_micro_s_;
    prev_time_micro_s_ = current_time;
    int left_pulse_temp = left_pulse;
    left_pulse -= left_pulse_temp;
    int right_pulse_temp = right_pulse;
    right_pulse -= right_pulse_temp;

    double left_dir = left_target_vel_rad_p_s_ > 0 ? 1.0 : left_target_vel_rad_p_s_ < 0 ? -1.0 : 0.0;
    if (left_dir != 0.0 && left_dir_ != left_dir) {
      if (left_pulse_temp == 0 || (left_prev_pulse_ == 1 && left_pulse_temp > 2) || (left_near_0 && left_pulse_temp >= 4)) {
        left_dir_  = left_dir ;
        left_near_0 = false;
      } else {
        if (left_pulse_temp <= 2) left_near_0 = true;
      }
    }
    left_prev_pulse_ = left_pulse_temp;
    double left_diff_ang_rad  = left_dir_ * left_pulse_temp  * (2.0 * M_PI) / PULSE_PER_ROT / GEAR_RATIO;

    double right_dir = right_target_vel_rad_p_s_ > 0 ? -1.0 : right_target_vel_rad_p_s_ < 0 ? 1.0 : 0.0;
    if (right_dir != 0.0 && right_dir_ != right_dir) {
      if (right_pulse_temp == 0 || (right_prev_pulse_ == 1 && right_pulse_temp > 2) || (right_near_0 && right_pulse_temp >= 4)) {
        right_dir_  = right_dir ;
        right_near_0 = false;
      } else {
        if (right_pulse_temp <= 2) right_near_0 = true;
      }
    }
    right_prev_pulse_ = right_pulse_temp;
    double right_diff_ang_rad = right_dir_ * right_pulse_temp * (2.0 * M_PI) / PULSE_PER_ROT / GEAR_RATIO;

    left_current_vel_rad_p_s_  = left_diff_ang_rad  / ((double)diff_time_micro_s / 1000000.0);
    right_current_vel_rad_p_s_ = right_diff_ang_rad / ((double)diff_time_micro_s / 1000000.0);

    left_current_ang_rad_ += left_diff_ang_rad;
    right_current_ang_rad_+= right_diff_ang_rad;
}
