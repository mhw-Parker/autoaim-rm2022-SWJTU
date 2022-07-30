//
// Created by luojunhui on 7/22/20.
//

#ifndef TRANFORM_T_SERIAL_H
#define TRANFORM_T_SERIAL_H

#include <iostream>
#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <cerrno>      // ERROR Number Definitions
#include <termios.h>
#include <csignal>
#include <cstring>
#include <cstdio>

#define PC2STM32 "/dev/ttyUSB0"//串口位置


/*--------------------------------暂定协议-------------------------------------*/

// 四字节对齐，因此长度为4的整数
#define     SEND_LENGTH        16
#define     RECEIVE_LENGTH     16
//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xA5)
//end字节,协议固定为0xA5
#define    VISION_TOF         (0xA6)

/**---------------------------------------SEND DATA PROTOCOL--------------------------------------------**/
/**    ----------------------------------------------------------------------------------------------------
FIELD  |  A5  |  yaw  |  pitch  |  find  |   id   |   shoot  |   A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1  |   4   |    4    |    1   |    1   |    1     |   1  |
       ----------------------------------------------------------------------------------------------------
**/
/**---------------------------------------SEND DATA PROTOCOL--------------------------------------------**/


/**---------------------------------------RECEIVE DATA PROTOCOL-----------------------------------------------------------------------------**/
/**    -----------------------------------------------------------------------------------------------------------------------------------------
FIELD  |  head  |  yawAngle  |  pitchAngle  |  bullet  |  targetMode  |  targetColor |   A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1    |     4      |      4      |     4     |      1       |       1      |   1   |
------------------------------------------------------------------------------------------------------------------------------------------------
**/
/**---------------------------------------RECEIVE DATA PROTOCOL------------------------------------------------------------------------------**/
/**DESCRIPTION:
 * head: 0xA5
 * CmdID: Command ID
 * yawAngle: current yaw angle of pan-tilt
 * pitchAngle: current pitch angle of pan-tilt
 * yawSpeed: current spin speed of yaw direction
 * pitchSpeed: current pitch angle of pan-tilt
 * targetMode: the mode of vision task(AUTOSHOOT/ENERGY)
 * targetColor: blue or red enemy
 */

using namespace std;

/**
 * @brief receive data structure
 */
struct ReceiveData
{
    float yawAngle = 0;
    float pitchAngle = 0;
    float bulletSpeed = 0;
    uint8_t targetMode = 0;
    uint8_t targetColor = 0;// 0 red target, 1 blue target
};

class SendData
{
public:
    SendData(float &yaw, float &pitch, bool find, int num, bool cmd) : yaw(yaw), pitch(pitch), find(find), id(num), cmd(cmd) {};
    float yaw = 0;
    float pitch = 0;
    int id  = 0;
    uint8_t find = 0;
    uint8_t cmd = 0; // 1 shoot
};

/**
 * @brief SerialPort
 * @param filename 串口名字
 * @param buadrate 串口波特率,用于stm32通信是0-B115200,用于stm32通信是1-B921600
 */
class Serial
{
private:
    int fd;
    int nSpeed;
    char nEvent;
    int nBits;
    int nStop;
    uint8_t buff[SEND_LENGTH];
    uint8_t  buffRead[100];
    int readCount;
    int maxReadTime;
    static int set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop);
public:
    bool serial_state{};
    explicit Serial(int nSpeed = 115200, char nEvent = 'N', int nBits = 8, int nStop = 1);
    ~Serial();
    void pack(float yaw, float pitch, uint8_t find, uint8_t CarID, uint8_t shoot);
    bool InitPort(int nSpeed = 115200, char  nEvent = 'N', int nBits = 8, int nStop = 1);
    bool WriteData();
    bool ReadData(struct ReceiveData& buffer);
};

#endif //TRANFORM_T_SERIAL_H
