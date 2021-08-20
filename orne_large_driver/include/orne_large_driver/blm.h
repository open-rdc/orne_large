/**
 * BLM motor library (header)
 * Copyright 2018 - Joshua Supratman
 * Chiba Institute of Technology
**/

#ifndef BLM_H_
#define BLM_H_

typedef struct{
    int fd;
    unsigned int debug;
} BLMData; 

#define UPPER_BYTE(i)		((i) >> 8)
#define LOWER_BYTE(i)		((i) & 0xff)

#define COMM_READ			0x03
#define COMM_WRITE_SINGLE	0x06
#define COMM_WRITE			0x10

#define REG_COMMAND			0x007C
#define REG_READ_SPEED		0x00CE
#define REG_CONFIG			0x018C
#define REG_CONFIG_DIR		0x0384
#define REG_TARGET_SPEED	0x0484

#define MODE_DATA_2			0x02
#define MODE_STOP			0x20
#define MODE_BRAKE_FREE		0x80

#define MODE_CW				(0x08 | MODE_DATA_2 | MODE_STOP | MODE_BRAKE_FREE)
#define MODE_CCW			(0x10 | MODE_DATA_2 | MODE_STOP | MODE_BRAKE_FREE)

#define NUM_2				2

// low level comms
int BLM_Init(BLMData *r, const char *serial_port);
int BLM_Close(BLMData *r);
unsigned short BLM_Calc_CRC16(unsigned char *buff, int length);
int BLM_Communication(BLMData *r, unsigned char *buff, int length, float wait_s);
void BLM_Write_Register(BLMData *r, int id, int register_add, int num_data, short *data);

// motor setting commands
void BLM_Set_Configuration(BLMData *r, int id);
void BLM_Set_Reverse(BLMData *r, int id);
void BLM_Set_Angular_Velocity_RPM(BLMData *r, int id, int angular_velocity_rpm);
void BLM_Set_Angular_Velocity_RAD_PER_SEC(BLMData *r, int id, double angular_velocity_rad_per_sec);

// motor action commands
void BLM_Mode(BLMData *r, int id, unsigned char cmd);
void BLM_Change_Mode(BLMData *r, int id, double angular_velocity);

// motor observation commands
int BLM_Read_Register_2word(BLMData *r, int id, int register_add, short *value);
int BLM_Read_Current_Angular_Velocity_RPM(BLMData *r, int id, short *angular_velocity_rpm);
int BLM_Read_Current_Angular_Velocity_RAD_PER_SEC(BLMData *r, int id, double *angular_velocity_rad_per_sec);
int BLM_Read_Target_Angular_Velocity_RPM(BLMData *r, int id, short *angular_velocity_rpm);
int BLM_Read_Target_Angular_Velocity_RAD_PER_SEC(BLMData *r, int id, double *angular_velocity_rad_per_sec);
#endif
