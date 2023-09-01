#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "main.h"
#include "usart.h"
#include "string.h"

#define Uart_handl huart6

//处理接收的数据
// #define get_receive_measure(pre, data)                                                                              \
//     {                                                                                                               \
//         (pre)->Switch = (data)[1];                                                                                  \
//         (pre)->balanc_pit = (float)((data)[2] <<24 | (data)[3] << 16 | (data)[4] << 8 | (data)[5]);                 \
//         (pre)->kp1 = (float)((data)[6] <<24 | (data)[7] << 16 | (data)[8] << 8 | (data)[9]);                        \
//         (pre)->ki1 = (float)((data)[10] <<24 | (data)[11] << 16 | (data)[12] << 8 | (data)[13]);                    \
//         (pre)->kd1 = (float)((data)[14] <<24 | (data)[15] << 16 | (data)[16] << 8 | (data)[17]);                    \
//         (pre)->MaxOutPut1 = (float)((data)[18] <<24 | (data)[19] << 16 | (data)[20] << 8 | (data)[21]);             \
//         (pre)->I_limit1 = (float)((data)[22] <<24 | (data)[23] << 16 | (data)[24] << 8 | (data)[25]);               \
//         (pre)->kp2 = (float)((data)[26] <<24 | (data)[27] << 16 | (data)[28] << 8 | (data)[29]);                    \
//         (pre)->ki2 = (float)((data)[30] <<24 | (data)[31] << 16 | (data)[32] << 8 | (data)[33]);                    \
//         (pre)->kd2 = (float)((data)[34] <<24 | (data)[35] << 16 | (data)[36] << 8 | (data)[37]);                    \
//         (pre)->MaxOutPut2 = (float)((data)[38] <<24 | (data)[39] << 16 | (data)[40] << 8 | (data)[41]);             \
//         (pre)->I_limit2 = (float)((data)[42] <<24 | (data)[43] << 16 | (data)[44] << 8 | (data)[45]);               \
//         (pre)->speed = (float)((data)[46] <<24 | (data)[47] << 16 | (data)[48] << 8 | (data)[49]);                  \
//         (pre)->turn = (float)((data)[50] <<24 | (data)[51] << 16 | (data)[52] << 8 | (data)[53]);                   \
//     }                                                                                                               

//接收数据结构体
typedef struct
{
    uint8_t Switch;         //遥控开关
    float balanc_pit;       //平衡时的pit角
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

    float speed;          //前进速度(-1~1)
    float turn;           //转向速度(-1~1)
    uint16_t err;         //记录数据包错误次数
} t_receive;

void BlueTooth_init();
void Uart_transmit_measure(void);
void get_data(uint8_t *data);


#endif
