/**
 * BLM motor library for arduino (header)
 * Yasuo Hayashibara, 2019
 * Chiba Institute of Technology
**/

#ifndef BLM_COMM_H_
#define BLM_COMM_H_

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

class BLMComm
{
public:
    // low level commands
    BLMComm(int id);
    ~BLMComm();
    void ServoOn(bool is_servo_on);
    float operator = (double obj) { this->WriteAngularVelocityRadPS(obj); }
    void WriteAngularVelocityRPM(int angular_velocity_rpm);
    void WriteAngularVelocityRadPS(double angular_velocity_rad_per_sec);
    int ReadCurrentAngularVelocityRPM();
    double ReadCurrentAngularVelocityRadPS();
    int ReadTargetAngularVelocityRPM();
    double ReadTargetAngularVelocityRadPS();

private:
    unsigned short CalcCRC16(unsigned char *buff, int length);
    int Communication(unsigned char *buff, int length, float wait_s);
    void WriteRegister(int register_add, int num_data, short *data);

    // motor setting commands
    void WriteConfiguration();
    void WriteReverse();

    // motor action commands
    void WriteMode(unsigned char cmd);
    void WriteChangeMode(double angular_velocity);

    // motor observation commands
    int ReadRegister2word(int register_add, short *value);

    int id_;
};

#endif
