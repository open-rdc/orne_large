/**
 * BLM motor library for arduino (header)
 * Yasuo Hayashibara, 2019
 * Chiba Institute of Technology
**/

#ifndef BLM_MOTOR_H_
#define BLM_MOTOR_H_


class BLMMotor
{
public:
    BLMMotor();
    ~BLMMotor();
    void setTargetAngularVelocities(double left_target_vel_rad_p_s, double right_target_vel_rad_p_s);
    void getCurrentAngularVelocities(double *left_current_vel_rad_p_s, double *right_current_vel_rad_p_s);
    void getCurrentAngles(double *left_current_ang_rad, double *right_current_ang_rad);
    void measureCurrentAngles();
private:
    double left_target_vel_rad_p_s_, right_target_vel_rad_p_s_;
    double left_current_vel_rad_p_s_, right_current_vel_rad_p_s_;
    double left_current_ang_rad_, right_current_ang_rad_;

    double right_dir_, left_dir_;
    unsigned long prev_time_micro_s_;
    int left_prev_pulse_, right_prev_pulse_;
};
#endif
