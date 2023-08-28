#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "main.h"

//处理接收的数据
#define get_receive_measure(pre, data)                                                                              \
    {                                                                                                               \
        (pre)->Switch = (data)[0];                                                                                  \
        (pre)->balanc_yaw = (float)((data)[1] <<24 | (data)[2] << 16 | (data)[3] << 8 | (data)[4]);                 \
        (pre)->kp1 = (float)((data)[5] <<24 | (data)[6] << 16 | (data)[7] << 8 | (data)[8]);                        \
        (pre)->ki1 = (float)((data)[9] <<24 | (data)[10] << 16 | (data)[11] << 8 | (data)[12]);                     \
        (pre)->kd1 = (float)((data)[13] <<24 | (data)[14] << 16 | (data)[15] << 8 | (data)[16]);                    \
        (pre)->MaxOutPut1 = (float)((data)[17] <<24 | (data)[18] << 16 | (data)[19] << 8 | (data)[20]);             \
        (pre)->I_limit1 = (float)((data)[21] <<24 | (data)[22] << 16 | (data)[23] << 8 | (data)[24]);               \
        (pre)->kp2 = (float)((data)[25] <<24 | (data)[26] << 16 | (data)[27] << 8 | (data)[28]);                    \
        (pre)->ki2 = (float)((data)[29] <<24 | (data)[30] << 16 | (data)[31] << 8 | (data)[32]);                    \
        (pre)->kd2 = (float)((data)[33] <<24 | (data)[34] << 16 | (data)[35] << 8 | (data)[36]);                    \
        (pre)->MaxOutPut2 = (float)((data)[37] <<24 | (data)[38] << 16 | (data)[39] << 8 | (data)[40]);             \
        (pre)->I_limit2 = (float)((data)[41] <<24 | (data)[42] << 16 | (data)[43] << 8 | (data)[44]);               \
        (pre)->speed = (float)((data)[45] <<24 | (data)[46] << 16 | (data)[47] << 8 | (data)[48]);                  \
        (pre)->turn = (float)((data)[49] <<24 | (data)[50] << 16 | (data)[51] << 8 | (data)[52]);                   \
    }                                                                                                               

//接收数据结构体
typedef struct
{
    uint8_t Switch;         //遥控开关
    float balanc_yaw;       //平衡时的yaw角
    //直立环
    float kp1;            
    float ki1;
    float kd1;
    float MaxOutPut1;
    float I_limit1;
    //速度环
    float kp2;
    float ki2;
    float kd2;
    float MaxOutPut2;
    float I_limit2;

    float speed;          //前进速度
    float turn;           //转向速度
    uint16_t err;         //记录数据包错误次数
} t_receive;

uint8_t transmit_buffer[31] = {0};
uint8_t receive_buffer[56] = {0};


#endif
