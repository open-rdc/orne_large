#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include "orne_large_driver/blm.h"
#include "ftdi.h"
#include <math.h>

#define BAUDRATE B115200

const int MAX_BUF_NUM = 64;
const double REDUCTION_RATIO = 100.0;
const double MIN_TO_SEC = 60.0;

int BLM_Init(BLMData *r, const char *serial_port){
    printf("BLM initialize\n");
    assert(r);
    r->debug = 0;
    
    struct termios tio;

    r->fd = open(serial_port, O_RDWR | O_NOCTTY);
    if(ioctl(r->fd, TCGETS, &tio)){
        return -1;
    };

    tio.c_cflag &= ~PARODD;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= PARENB;
    tio.c_cflag |= CS8;
    tio.c_cflag |= CREAD;
    tio.c_cflag |= CLOCAL;
    tio.c_iflag = IGNBRK | IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;
    cfsetispeed( &tio, BAUDRATE );
    cfsetospeed( &tio, BAUDRATE );
    
    tcflush(r->fd, TCIOFLUSH);
    if(ioctl(r->fd, TCSETS, &tio)){
        return -1;
    };

    struct serial_struct sstruct;

    if(ioctl(r->fd, TIOCGSERIAL, &sstruct) < 0){
        printf("error: could not get comm ioctl\n");
        exit(-1);
    }
    sstruct.custom_divisor = sstruct.baud_base / BAUDRATE;
    sstruct.flags |= ASYNC_SPD_CUST;

    if(ioctl(r->fd, TIOCSSERIAL, &sstruct) < 0){
        printf("error: could not get custom comm baud divisor\n");
        exit(-1);
    }
    return 0;
}

int BLM_Close(BLMData *r){
    printf("blm close\n");
    assert(r);
    close(r->fd);
    return 0;
}

/**
 * @brief calculate CRC (Cyclic Redundancy Check)
 * @param buff send data
 * @param length length of data
 * @return CRC
 */
unsigned short BLM_Calc_CRC16(unsigned char *buff, int length)
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
 * @param[in] r BLMData
 * @param[in] buff send data
 * @param[out] buff received data
 * @param[in] length length of send data
 * @return number of received data
 */
int BLM_Communication(BLMData *r, unsigned char *buff, int length, float wait_s = 0.03)
{
    int num = length;

    unsigned short value = BLM_Calc_CRC16(buff, length);
    buff[num++] = value & 0xFF;
    buff[num++] = (value >> 8) & 0xFF;

    if(r->debug){
        struct timeval time;
        gettimeofday(&time, NULL);
        printf("%06d: ", (int)time.tv_usec);
        for(int i = 0; i < num; i++) printf("%02X ", buff[i]);
        printf("\n");
    }
    write(r->fd, buff, num);
    
    usleep((int)(wait_s * 1000000));

    int num_recv_data = read(r->fd, buff, MAX_BUF_NUM);
    if(r->debug){
        printf("RECV %02d: ", num_recv_data);
        for(int i = 0; i < num_recv_data; i++) printf("%02X ",buff[i]);
        printf("\n");
    }
    return num_recv_data;
}


/**
 * @brief write data to register
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param[in] register_add address of register
 * @param[in] num_data number of data
 * @param[in] data data
 * @return number of received data
 */
void BLM_Write_Register(BLMData *r, int id, int register_add, int num_data, short *data)
{
    unsigned char buff[MAX_BUF_NUM];
    int num = 0;

    buff[num++] = id;
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

    BLM_Communication(r, buff, num);
}

/**
 * @brief set configration
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 */
void BLM_Set_Configuration(BLMData *r, int id)
{
    short data[] = {0, 1};
    BLM_Write_Register(r, id, REG_CONFIG, 2, data);
}

/**
 * @brief set reverse the direction of wheel
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 */
void BLM_Set_Reverse(BLMData *r, int id)
{
    short data[] = {0, 0};
    BLM_Write_Register(r, id, REG_CONFIG_DIR, 2, data);
    BLM_Set_Configuration(r, id); // 反映C
}

/**
 * @brief set angular velocity (RPM)
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param[in] angular_velocity_rpm target angular velocity (RPM)
 */
void BLM_Set_Angular_Velocity_RPM(BLMData *r, int id, int angular_velocity_rpm)
{
    short data[] = {0, 0};
    if(angular_velocity_rpm <   80) angular_velocity_rpm=0;
    if(angular_velocity_rpm > 4000) angular_velocity_rpm=4000;
    data[1] = angular_velocity_rpm;
    BLM_Write_Register(r, id, REG_TARGET_SPEED, 2, data);
}

/**
 * @brief set angular velocity (rad/s)
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param angular_velocity_rad_per_sec target angular velocity (rad/s)
 */
void BLM_Set_Angular_Velocity_RAD_PER_SEC(BLMData *r, int id, double angular_velocity_rad_per_sec){
    int angular_velocity_rpm = angular_velocity_rad_per_sec / (2.0 * M_PI) * REDUCTION_RATIO * MIN_TO_SEC;
    BLM_Set_Angular_Velocity_RPM(r, id, angular_velocity_rpm);
}

/**
 * @brief read data (2 word: 32 bit)
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param register_add address of register
 * @param value received value
 * @return number of received data
 */
int BLM_Read_Register_2word(BLMData *r, int id, int register_add, int *value) {
    const int NUM_DATA = 2;
    unsigned char buff[MAX_BUF_NUM];
    int num = 0;

    buff[num++] = id;
    buff[num++] = COMM_READ;
    buff[num++] = UPPER_BYTE(register_add); // レジスタアドレス（上位）
    buff[num++] = LOWER_BYTE(register_add); // レジスタアドレス（下位）
    buff[num++] = UPPER_BYTE(NUM_DATA); // レジスタの個数（上位）
    buff[num++] = LOWER_BYTE(NUM_DATA); // レジスタの個数（下位）

    int res = BLM_Communication(r, buff, num, 0.01);
    *value = (int)(((unsigned int)buff[3] << 24) + ((unsigned int)buff[4] << 16) + ((unsigned int)buff[5] << 8) + (unsigned int)buff[6]);
    return res;
}

/**
 * @brief read current angular velocity (PRM)
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param angular_velocity_rpm current angular velocity (RPM)
 * @return number of received data
 */
int BLM_Read_Current_Angular_Velocity_RPM(BLMData *r, int id, int *angular_velocity_rpm) {
    return BLM_Read_Register_2word(r, id, REG_READ_SPEED, angular_velocity_rpm);
}

/**
 * @brief read current angular velocity (rad/s)
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param angular_velocity_rad_per_sec current angular velocity (rad/s)
 * @return number of received data
 */
int BLM_Read_Current_Angular_Velocity_RAD_PER_SEC(BLMData *r, int id, double *angular_velocity_rad_per_sec) {
    int angular_velocity_rpm;
    int res = BLM_Read_Current_Angular_Velocity_RPM(r, id, &angular_velocity_rpm);
    *angular_velocity_rad_per_sec = (double)angular_velocity_rpm * (2.0 * M_PI) / REDUCTION_RATIO / MIN_TO_SEC;
}

/**
 * @brief read target angular velocity (PRM)
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param angular_velocity_rpm target angular velocity (RPM)
 * @return number of received data
 */

int BLM_Read_Target_Angular_Velocity_RPM(BLMData *r, int id, int *angular_velocity_rpm) {
    return BLM_Read_Register_2word(r, id, REG_TARGET_SPEED, angular_velocity_rpm);
}

/**
 * @brief read current angular velocity (rad/s)
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param angular_velocity_rad_per_sec target angular velocity (rad/s)
 * @return number of received data
 */
int BLM_Read_Target_Angular_Velocity_RAD_PER_SEC(BLMData *r, int id, double *angular_velocity_rad_per_sec) {
    int angular_velocity_rpm;
    int res = BLM_Read_Target_Angular_Velocity_RPM(r, id, &angular_velocity_rpm);
    *angular_velocity_rad_per_sec = (double)angular_velocity_rpm * (2.0 * M_PI) / REDUCTION_RATIO / MIN_TO_SEC;
}

/**
 * @brief set mode ot the motor
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param cmd mode (MODE_CW, MODE_CCW, MODE_STOP, MODE_BRAKE)
 */
void BLM_Mode(BLMData *r, int id, unsigned char cmd){
    short data[] = {0, 0};
    data[1] = cmd;
    BLM_Write_Register(r, id, REG_COMMAND, 2, data);    
}

/**
 * @brief change mode ot the motor
 * @param[in] r BLMData
 * @param[in] id id (0-31, 0:broadcast)
 * @param angular_velocity angular velocity (0:STOP, >0: CW, <0: CCW)
 */
void BLM_Change_Mode(BLMData *r, int id, double angular_velocity){
    if(angular_velocity >= 0){
        BLM_Mode(r, id, MODE_CW);
    } else{
        BLM_Mode(r, id, MODE_CCW);
    }    
}
