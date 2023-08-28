/***************************************************************************************************************
 * 用于和手机蓝牙调试器通讯
 * 包含数据包解析和形成
 * 
 * Create by 羽栖 on 2023/8/28
*****************************************************************************************************************/
#include "BlueTooth.h"
#include "mpu6500_driver.h"
#include "CAN_receive.h"
#include "pid.h"

uint8_t transmit_buffer[31] = {0};
uint8_t receive_buffer[56] = {0};
extern motor_measure_t       motor_chassis[2];
extern imu_t                 imu;			//IMU数据储存
pid_t                 Pid_bal, Pid_spe;
float bal_pit = 0;             //平衡时的pit角
float speed = 0;                //速度
float turn  = 0;                //转向速度
t_receive receive_measure = {0};      //储存接收数据

/**
  * @brief          初始化蓝牙通讯
  * @param          无
  * @retval         none
  */
void BlueTooth_init()
{
  __HAL_UART_ENABLE_IT(&Uart_handl, UART_IT_IDLE);            //开启串口空闲中断
  HAL_UART_Receive_DMA(&Uart_handl, receive_buffer, 56);      //开启Dma传输
  transmit_buffer[0] = 0xA5;
  transmit_buffer[30] = 0x5A;
}

/**
  * @brief          串口发送数据包
  * @param          无
  * @retval         none
  */
void Uart_transmit_measure(void)
{
  //右轮数据
  transmit_buffer[1] = (motor_chassis[0].speed_rpm >> 8);
  transmit_buffer[2] = motor_chassis[0].speed_rpm;
  transmit_buffer[3] = (motor_chassis[0].given_current >> 8);
  transmit_buffer[4] = motor_chassis[0].given_current;
  //左轮数据
  transmit_buffer[5] = (motor_chassis[0].speed_rpm >> 8);
  transmit_buffer[6] = motor_chassis[0].speed_rpm;
  transmit_buffer[7] = (motor_chassis[0].given_current >> 8);
  transmit_buffer[8] = motor_chassis[0].given_current;
  //yaw角
  memcpy(&transmit_buffer[9], &imu.yaw, 4);
  //pit角
  memcpy(&transmit_buffer[13], &imu.pit, 4);  
  //rol角
  memcpy(&transmit_buffer[17], &imu.rol, 4);    
  //iout
  memcpy(&transmit_buffer[21], &Pid_bal.iout, 4);  
  //pos_out
  memcpy(&transmit_buffer[25], &Pid_bal.pos_out, 4);
  
  uint32_t temp = 0;
  for(uint8_t i = 1; i < 29; i++)
  {
    temp += transmit_buffer[i];
  }
  transmit_buffer[29] = temp & 0xFF;
  HAL_UART_Transmit_DMA(&Uart_handl, transmit_buffer, 31);
}

/**
  * @brief          处理接收的数据包
  * @param          data :接收数据数组
  * @retval         none
  */

void get_data(uint8_t *data)
{
  uint32_t temp=0;
  if(data[0] == 0xA5 && data[55] == 0x5A)
  {
    for(uint8_t i = 1; i < 54; i++)
    {
      temp += data[i];
    }
    if((temp & 0xFF) == data[54])
    {
      receive_measure.Switch = receive_buffer[1];
      memcpy(&receive_measure.balanc_pit, &receive_buffer[2], 13 * sizeof(float));
    }
    else
    {
      receive_measure.err ++;
    }
  }
  else
  {
    receive_measure.err ++;
  }
  bal_pit = receive_measure.balanc_pit;
  pid_param_init(&Pid_bal, POSITION_PID, receive_measure.MaxOutPut1, receive_measure.I_limit1, receive_measure.kp1, receive_measure.ki1, receive_measure.kd1);
  pid_param_init(&Pid_spe, POSITION_PID, receive_measure.MaxOutPut2, receive_measure.I_limit2, receive_measure.kp2, receive_measure.ki2, receive_measure.kd2);
  speed = receive_measure.speed;
  turn = receive_measure.turn;
}