/*********************************************************************************************************
*RM_A���mpu6500����������ist8310�����ǿ��Ӧ��������
*��ͨ�˲�����Ԫ����̬����

�ο�������
* [IMU ���������ݶ�ȡ](https://github.com/RoboMaster/DevelopmentBoard-Examples/tree/master/Imu)

Create by ���� on 2023/8/20
*********************************************************************************************************/
#include "main.h"
#include "mpu6500_driver.h"
#include "mpu6500_map.h"
#include "spi.h"
#include "string.h"
#include "math.h"

#define BOARD_DOWN (1)                                       //����ѡ����Ԫ������ѡ��
#define IST8310                   
#define Kp 2.0f                                              /* 
                                                              * ��������
                                                              * �ںϼ��ٶȼƺʹ�ǿ�� 
																															*/
#define Ki 0.01f                                             /* 
                                                              * ��������
                                                              * ������ƫ������
																															*/								

extern SPI_HandleTypeDef hspi5;
static uint8_t        tx, rx;                     //�����д����
volatile float        q0 = 1.0f;		              //��Ԫ��ϵ��
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;
static uint8_t        tx_buff[14] = { 0xff };			//MPU6500���ݱ��������ٶȣ��¶ȣ��Ƕȣ�
uint8_t               mpu_buff[14]; // ����IMUԭʼ����
uint8_t               ist_buff[6];  //����IST8310ԭʼ����
mpu_data_t            mpu_data;			//����MPU���ݾ��
imu_t                 imu={0};			//IMU���ݴ���
uint8_t               id;				    //����ID mpu6500ID��0x70
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz; 		//����IMU����ֵ
volatile uint32_t     last_update, now_update;                /* ��������, ��λ ms */
volatile float        exInt, eyInt, ezInt;                   /* ������ʾ */

/**
 * @brief       ���ټ���ƽ�������������� 1/Sqrt(x)
 * @param       X:���ֻ���Ҫ����
 * @retval      1/Sqrt(x)
 */
 float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}

/**
 * @brief       mpu6500����д
 * @param       reg��д�ĵ�ַ
 *              data��д������
 * @retval      0
 */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
  MPU_NSS_LOW;       //��ʼͨѶ
  tx = reg & 0x7f;   //��һλΪ0��дģʽ��
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx , &rx, 1, 55);     //д���ַ
  tx = data;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx , &rx, 1, 55);     //д������
  MPU_NSS_HIGH;      //����ͨѶ
  return 0;
}

/**
 * @brief       mpu6500���ζ�
 * @param       reg�����ĵ�ַ
 * @retval      ��������
 */
uint8_t mpu_read_byte(uint8_t const reg)
{
  MPU_NSS_LOW;       //��ʼͨѶ
  tx = reg | 0x80;   //��һλΪ1����ģʽ��
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx , &rx, 1, 55);     //д���ַ
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx , &rx, 1, 55);     //д������
  MPU_NSS_HIGH;      //����ͨѶ
  return rx;
}

/**
 * @brief       mpu6500���ζ��������
 * @param       regAddr�����ĵ�ַ
 *              pData��  ���ݻ���
 *              len��    ���ݳ���
 * @retval      0
 */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
  MPU_NSS_LOW;       //��ʼͨѶ
  tx = regAddr | 0x80;   //��һλΪ1����ģʽ��
  tx_buff[0] = tx;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx , &rx, 1, 55);             //д���ַ
  HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff , pData, len, 55);     //д������
  MPU_NSS_HIGH;      //����ͨѶ
  return 0;
}

/**
 * @brief       ͨ��MPU6500������дIST8310�Ĵ���
 * @param       addr��  �����ַ
 *              data��  д������
 * @retval      ��
 */
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* �ȹرմӻ�1 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);/*��CTRLλ7д0�رմӻ�  д1��ʼ�Ӵӻ�������*/
    HAL_Delay(2);
		/* �Ӵӻ�1 addr��ַ ��ʼ�������� */
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    HAL_Delay(2);
		/* �ӻ�1�������data*/
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    HAL_Delay(2);
		/* �򿪴ӻ�1����1���ֽ� */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);/*λ0~3���Ӵӻ��������ֽ�*/
    /* �ȴ�ȷ�����ݷ��� */
    HAL_Delay(10);
}

/**
 * @brief       ͨ��MPU6500������ȡIST8310�Ĵ���
 * @param       addr��  ��ȡ��ַ
 * @retval      ��������
 */
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
		/* �Ӵӻ�4 addr��ַ ��ʼ�������� */
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    HAL_Delay(10);
		/* �ȿ����ӻ�4 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    HAL_Delay(10);
		/* ���ӻ�4��ȡ�����ݴ���retval*/
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* ��ȡ��رմӻ�4 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);
		/* ����retval*/
    return retval;
}

/**
 * @brief       ��ʼ���ӻ�0����I2C�Ĵ�����ȡ
 * @param       device_address��  �豸�����ַ
 *              reg_base_addr��   �Ĵ���ӳ���ַ
 *              data_num��        ���ݸ���
 * @retval      ��������
 */
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * ����IST8310�豸��ַ 
     * ʹ�ôӻ�1�����Զ����䵥����ģʽ 
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);    //���ôӻ�1�������ַ
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);    //����ist8310���ֽ�
    HAL_Delay(2);

    /* ʹ�ôӻ�0�Զ���ȡ���� */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);   /*ADDRλ7Ϊ1�Ƕ�ȡ*/
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    HAL_Delay(2);

    /* ÿ8��mpu6500�ڲ��������ڣ�����һ��I2C���ݶ�ȡ */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    HAL_Delay(2);
    /* ʹ�ܴӻ�1���ӻ�0��ʱ����*/
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    HAL_Delay(2);
    /* ʹ�ܴӻ�1�Զ����� */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* �ȴ�6��ʱ������ (��С�ȴ�ʱ��Ϊ�ڲ�ƽ�����õ�16��) */
    HAL_Delay(6); 
    /* ʹ�ܴӻ�0�������ݶ�ȡ */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    HAL_Delay(2);
}

/**
 * @brief       ��ʼ��ist8310
 * @param       ��
 * @retval      ��
 */
uint8_t ist8310_init()
{
  /*ʹ��I2C��ģʽ ��λI2C��ģʽ��������λSPIģʽ */
	mpu_write_byte(MPU6500_USER_CTRL, 0x30);
	HAL_Delay(10);
	/* MPU6500��ʱ��400khz */
  mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
  HAL_Delay(10);
	/* �ӻ�1д */
	mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
  HAL_Delay(10);
	/* �ӻ�4�� */
  mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  HAL_Delay(10);
	/* IST8310_R_CONFB 0x01 = ��λ */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
  HAL_Delay(10);
	
  if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
      return 1;

	/* �����λ */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
  HAL_Delay(10);

	/* ��IST8310����Ϊ����״̬ */
  ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
  if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
       return 2;
  HAL_Delay(10);

  /* ����״̬ �޳�ʼ�� */
  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
  if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
       return 3;
  HAL_Delay(10);
		
  /* ���óɵ�����״̬, x,y,z �� 16 �βɼ��� 1 ��ƽ��ֵ */
  ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
  if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
  HAL_Delay(10);

  /* ����/��λ��������,����ģʽ */
  ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
  if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
       return 5;
  HAL_Delay(10);

  /* �رմӻ�1�ʹӻ�4 */
  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(10);
  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);

  /* ���ô򿪴ӻ�0 */
  mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  HAL_Delay(100);
  return 0;
}

/**
 * @brief       ��ȡIST8310������
 * @param       buff ���ݻ���
 * @retval      ��
 */
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}

/**
 * @brief       ����imu 6500�����ǲ�����Χ
 * @param       fsr: ��Χ
 * @retval      0
 */
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);   //GYRO_CONFIG��λ4��3ѡ�������Χ��00 = ��250dps�� 01= ��500dps�� 10 = ��1000dps�� 11 = ��2000dps��
}

/**
 * @brief       ����imu 6500���ٲ�����Χ
 * @param       fsr: ��Χ
 * @retval      0
 */
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);  //ACCEL_CONFIG��λ4��3ѡ�������Χ����2g (00), ��4g (01), ��8g (10), ��16g (11)��
}

/**
 * @brief       ��ȡIMU����
 * @param       ��
 * @retval      ��
 */
void mpu_get_data()
{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    ist8310_get_data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
	
    imu.temp = 21 + mpu_data.temp / 333.87f;
	  /* 2000dps -> rad/s */
	  imu.wx   = mpu_data.gx / 16.384f / 57.3f; 
    imu.wy   = mpu_data.gy / 16.384f / 57.3f; 
    imu.wz   = mpu_data.gz / 16.384f / 57.3f;
}

/**
 * @brief       ��ȡMPU6500��ƫ�������ݣ���ȡ300���Լ�С��
 * @param       ��
 * @retval      ��
 */
void mpu_offset_call(void)
{
	int i;
	for (i=0; i<300;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
	
		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		HAL_Delay(5);
	}
	mpu_data.ax_offset=mpu_data.ax_offset / 300;
	mpu_data.ay_offset=mpu_data.ay_offset / 300;
	mpu_data.az_offset=mpu_data.az_offset / 300;
	mpu_data.gx_offset=mpu_data.gx_offset / 300;
	mpu_data.gy_offset=mpu_data.gx_offset / 300;
	mpu_data.gz_offset=mpu_data.gz_offset / 300;
}

/**
 * @brief       ��ʼ��mpu6500��ist3810
 * @param       ��
 * @retval      0
 */
uint8_t mpu_device_init(void)
{
	HAL_Delay(100);

	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* �����豸*/ 
																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* ������ʱ��Դ���� */ 
																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* ���� Acc & Gyro */ 
																			{ MPU6500_CONFIG, 0x04 },         /* ��ͨ�˲� Ƶ��41Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* ʹ�ܵ�ͨ�˲���  ���� Acc ��ͨ�˲� */ 
																			{ MPU6500_USER_CTRL, 0x20 },};    /* ʹ�� AUX */ 
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		HAL_Delay(1);
	}

	mpu_set_gyro_fsr(3); 		
	mpu_set_accel_fsr(2);

	ist8310_init();
	mpu_offset_call();
	return 0;
}

/**
 * @brief       ��ʼ����Ԫ��
 * @param       ��
 * @retval      ��
 */
void init_quaternion(void)
{
	int16_t hx, hy;//hz;
	
	hx = imu.mx;
	hy = imu.my;
	//hz = imu.mz;
	
	#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0) 
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
}

/**
 * @brief       ����IMU��ֵ
 * @param       ��
 * @retval      ��
 */
void imu_ahrs_update(void) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0,tempq1,tempq2,tempq3;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;   

	gx = imu.wx;
	gy = imu.wy;
	gz = imu.wz;
	ax = imu.ax;
	ay = imu.ay;
	az = imu.az;
	mx = imu.mx;
	my = imu.my;
	mz = imu.mz;

	now_update  = HAL_GetTick(); //ms
	halfT       = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	/* ������ƽ���� */
	norm = inv_sqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	#ifdef IST8310
		norm = inv_sqrt(mx*mx + my*my + mz*mz);          
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm; 
	#else
		mx = 0;
		my = 0;
		mz = 0;		
	#endif
	/* ����ͨ���Ĳο�����*/
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* ����������ͨ������(v��w)   */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
	
	/* 
	 * ����ǳ��Ĳο������봫������������Ĳ��֮�� 
	 */
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;
		
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
	}
	
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/*����Ԫ����׼�� */
	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

/**
 * @brief       ����imu����̬
 * @param       ��
 * @retval      ��
 */
void imu_attitude_update(void)
{
	/* yaw    -pi----pi */
	imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3; 
	/* pitch  -pi/2----pi/2 */
	imu.pit = -asin(-2*q1*q3 + 2*q0*q2)* 57.3;   
	/* roll   -pi----pi  */	
	imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
}
