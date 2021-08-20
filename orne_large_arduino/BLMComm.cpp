#include "BLMComm.h"
#include <Arduino.h>
#include <HardwareSerial.h>

//#define DEBUG 1
#define BAUDRATE 115200
#define SERIAL_PORT Serial3

const int MAX_BUF_NUM = 64;
const double REDUCTION_RATIO = 100.0;
const double MIN_TO_SEC = 60.0;

static bool isPortOpened = false;

BLMComm::BLMComm(int id):id_(id) {
    if (!isPortOpened) {
        // data bit: 8, parity: even, stop bit: 1
        SERIAL_PORT.begin(BAUDRATE, SERIAL_8E1);
        isPortOpened = true;
    }
};


BLMComm::~BLMComm() {
    if (isPortOpened) {
        SERIAL_PORT.end();
    }
}

void BLMComm::ServoOn(bool is_servo_on) {

}

/**
 * @brief set angular velocity (RPM)
 * @param[in] angular_velocity_rpm target angular velocity (RPM)
 */
void BLMComm::WriteAngularVelocityRPM(int angular_velocity_rpm)
{
    short data[] = {0, 0};
    if(angular_velocity_rpm <   80) angular_velocity_rpm=0;
    if(angular_velocity_rpm > 4000) angular_velocity_rpm=4000;
    data[1] = angular_velocity_rpm;
    WriteRegister(REG_TARGET_SPEED, 2, data);
}

/**
 * @brief set angular velocity (rad/s)
 * @param angular_velocity_rad_per_sec target angular velocity (rad/s)
 */
void BLMComm::WriteAngularVelocityRadPS(double angular_velocity_rad_per_sec){
    int angular_velocity_rpm = angular_velocity_rad_per_sec / (2.0 * M_PI) * REDUCTION_RATIO * MIN_TO_SEC;
    WriteAngularVelocityRPM(angular_velocity_rpm);
}

/**
 * @brief calculate CRC (Cyclic Redundancy Check)
 * @param buff send data
 * @param length length of data
 * @return CRC
 */
unsigned short BLMComm::CalcCRC16(unsigned char *buff, int length)
{
    unsigned short crc = 0xFFFF;
    unsigned char carrayFlag;
    int i1,i2;
  
    for(i1 = 0; i1 < length; i1++) {
        crc ^= buff[i1];
        for(i2 = 0; i2 < 8; i2++){
            carrayFlag = crc & 1;
            crc = crc >> 1;
            if(carrayFlag){
                crc ^= 0xA001;
            } 
        } 
    }
    return crc;
}

/**
 * @brief send/reveive data
 * @param[in] buff send data
 * @param[out] buff received data
 * @param[in] length length of send data
 * @return number of received data
 */
int BLMComm::Communication(unsigned char *buff, int length, float wait_s = 0.03)
{
    int num = length;

    unsigned short value = CalcCRC16(buff, length);
    buff[num++] = value & 0xFF;
    buff[num++] = (value >> 8) & 0xFF;
    SERIAL_PORT.write(buff, num);
    delay(wait_s * 1000);
    int num_recv_data = SERIAL_PORT.readBytes(buff, MAX_BUF_NUM);
    return num_recv_data;
}

/**
 * @brief write data to register
 * @param[in] register_add address of register
 * @param[in] num_data number of data
 * @param[in] data data
 * @return number of received data
 */
void BLMComm::WriteRegister(int register_add, int num_data, short *data)
{
    unsigned char buff[MAX_BUF_NUM];
    int num = 0;

    buff[num++] = id_;
    buff[num++] = COMM_WRITE;
    buff[num++] = UPPER_BYTE(register_add); // レジスタアドレス（上位）
    buff[num++] = LOWER_BYTE(register_add); // レジスタアドレス（下位）
    buff[num++] = UPPER_BYTE(num_data); // レジスタの個数（上位）
    buff[num++] = LOWER_BYTE(num_data); // レジスタの個数（下位）
    buff[num++] = num_data * 2; // バイト数の２倍

    for(int i = 0; i < num_data; i ++) {
        buff[num++] = UPPER_BYTE(data[i]);
        buff[num++] = LOWER_BYTE(data[i]);
    }

    Communication(buff, num);
}

/**
 * @brief set configration
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 */
void BLMComm::WriteConfiguration()
{
    short data[] = {0, 1};
    WriteRegister(REG_CONFIG, 2, data);
}

/**
 * @brief set reverse the direction of wheel
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 */
void BLMComm::WriteReverse()
{
    short data[] = {0, 0};
    WriteRegister(REG_CONFIG_DIR, 2, data);
    WriteConfiguration(); // 反映C
}

/**
 * @brief read data (2 word: 32 bit)
 * @param register_add address of register
 * @param value received value
 * @return number of received data
 */
int BLMComm::ReadRegister2word(int register_add, short *value) {
    const int NUM_DATA = 2;
    unsigned char buff[MAX_BUF_NUM];
    int num = 0;

    buff[num++] = id_;
    buff[num++] = COMM_READ;
    buff[num++] = UPPER_BYTE(register_add); // レジスタアドレス（上位）
    buff[num++] = LOWER_BYTE(register_add); // レジスタアドレス（下位）
    buff[num++] = UPPER_BYTE(NUM_DATA); // レジスタの個数（上位）
    buff[num++] = LOWER_BYTE(NUM_DATA); // レジスタの個数（下位）

    int res = Communication(num, 0.01);
    *value = (int)(((unsigned int)buff[3] << 24) + ((unsigned int)buff[4] << 16) + ((unsigned int)buff[5] << 8) + (unsigned int)buff[6]);
    return res;
}

/**
 * @brief read current angular velocity (PRM)
 * @return current angular velocity (RPM)
 */
int BLMComm::ReadCurrentAngularVelocityRPM() {
    int angular_velocity_rpm;
    ReadRegister2word(REG_READ_SPEED, &angular_velocity_rpm);
    return angular_velocity_rpm;
}

/**
 * @brief read current angular velocity (rad/s)
 * @return current angular velocity (rad/s)
 */
double BLMComm::ReadCurrentAngularVelocityRadPS() {
    int angular_velocity_rpm = ReadCurrentAngularVelocityRPM();
    return (double)angular_velocity_rpm * (2.0 * M_PI) / REDUCTION_RATIO / MIN_TO_SEC;
}

/**
 * @brief read target angular velocity (PRM)
 * @return target angular velocity (RPM)
 */

int BLMComm::ReadTargetAngularVelocityRPM() {
    int angular_velocity_rpm;
    ReadRegister2word(REG_TARGET_SPEED, &angular_velocity_rpm);
    return angular_velocity_rpm;
}

/**
 * @brief read current angular velocity (rad/s)
 * @return target angular velocity (rad/s)
 */
double BLMComm::ReadTargetAngularVelocityRadPS() {
    int angular_velocity_rpm = ReadTargetAngularVelocityRPM();
    return (double)angular_velocity_rpm * (2.0 * M_PI) / REDUCTION_RATIO / MIN_TO_SEC;
}

/**
 * @brief set mode ot the motor
 * @param cmd mode (MODE_CW, MODE_CCW, MODE_STOP, MODE_BRAKE)
 */
void BLMComm::WriteMode(unsigned char cmd){
    short data[] = {0, 0};
    data[1] = cmd;
    WriteRegister(REG_COMMAND, 2, data);    
}

/**
 * @brief change mode ot the motor
 * @param angular_velocity angular velocity (0:STOP, >0: CW, <0: CCW)
 */
void BLMComm::WriteChangeMode(double angular_velocity){
    if(angular_velocity >= 0){
        WriteMode(MODE_CW);
    } else{
        WriteMode(MODE_CCW);
    }
}
