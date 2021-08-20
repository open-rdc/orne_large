#ifndef MB_ARDUINO
#define MB_ARDUINO

const int ENCODER_PULSE_PER_ROTATE = 2000;
const int ID_PRISMATIC  = 2;
const int ID_PITCH      = 3;
const int ID_YAW        = 4;
const double REDUCTION_RATIO_PRISMATIC  = 4.3 * 32.0 / 27.0;
const double REDUCTION_RATIO_PITCH      = 49.0 * 42.0 / 12.0;
const double REDUCTION_RATIO_YAW        = 49.0 * 32.0 / 19.0;
const double ROLLER_DIAMETER_M          = 0.0154; // check
const double POTENTIOMETER_VOLTAGE2ANGLE_RAD = 340.0 * (2 * M_PI / 360.0) / 4.0;

const int ID_AHM36A     = 5;

void initialize();
void controlStart();
void startProfilePositionControl();
void startCyclicSynchronousPositionControl();
void faultReset();
void shutDownControl();

//! 角度の正規化
float maxPI(float angle_rad){
  while(angle_rad >  M_PI) angle_rad -= 2 * M_PI;
  while(angle_rad < -M_PI) angle_rad += 2 * M_PI;
  return angle_rad;
}

#endif
