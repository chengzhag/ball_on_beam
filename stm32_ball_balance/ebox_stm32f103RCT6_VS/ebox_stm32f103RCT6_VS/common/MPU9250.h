/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  shentq
  * @version V1.2
  * @date    2016/08/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  * No part of this software may be used for any commercial activities by any form 
  * or means, without the prior written consent of shentq. This specification is 
  * preliminary and is subject to change at any time without notice. shentq assumes
  * no responsibility for any errors contained herein.
  * <h2><center>&copy; Copyright 2015 shentq. All Rights Reserved.</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __MPU6050_H
#define __MPU6050_H

#include "ebox.h"
#include "math.h"
/*模式选择，0为mpu6500模式，1为AK8963模式*/
#define MPU6500   0
#define AK8963    1

/*MPU6050 Register Address ------------------------------------------------------------*/
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18

#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I			0x75	//IIC地址寄存器(默认数值0x68，只读)
#define SLAVEADDRESS	0xD0	//IIC写入时的地址字节数据

//AK9863
#define INT_PIN_CFG       0x37
#define SLAVEAWRITE       0xD1
#define RA_MAG_ADDRESS		0x18
#define RA_MAG_INFO		    0x01
#define RA_MAG_ST1	        0x02
#define RA_MAG_XOUT_L		0x03
#define RA_MAG_XOUT_H		0x04
#define RA_MAG_YOUT_L		0x05
#define RA_MAG_YOUT_H		0x06
#define RA_MAG_ZOUT_L		0x07
#define RA_MAG_ZOUT_H		0x08
#define RA_MAG_ST2		    0x09
#define MAG_CNTL1           0x0A
#define MAG_CNTL2           0x0B
#define MAG_TEST1           0x0D
//AHRS算法数据
#define betaDef		0.2f		// 2 * proportional gain
/*
#define sampleFreq	125.0f		// sample frequency in Hz  采样率 100 HZ  10ms  修改此频率可增加变化速度
#define Kp 2.25f	//比例增益支配收敛率accellrometer/magnetometer           //0.6 0.7       2.5
#define Ki 0.0055f //积分增益执行速率陀螺仪的衔接gyroscopeases           //0.002  0.004   0.005
#define halfT 0.004f//采样周期的一半，若周期为10ms,则一般为0.005s        // 1      1      0.2
*/
//传感器原始数据
typedef struct  sensor_data
{
	short X;
	short Y;
	short Z;
}SENSOR_DATA;
//处理后的数据
typedef struct  imu_data
{
	float X;
	float Y;
	float Z;
}IMU_DATA;

class Mpu9250
{
public:
	float sample;   //采样率

	Mpu9250(I2c *i2c)
    {
        this->i2c = i2c;
    };
    void        begin(uint32_t speed);
	//设定模式
	void        mode(u8 mode);
	int16_t 	get_data_2byte(uint8_t reg_address);
	//字节读取，用于读取特定寄存器
	int8_t   	get_data_1byte(uint8_t reg_address);
	//连续地址寄存器数据读取
    int8_t 		get_data(uint8_t reg_address, int16_t *buf, uint8_t num_to_read);
	//往寄存器写数据
	void        write_data(u8 reg_address, u8 data);
	//获取设备id
    void        get_id(uint8_t *id);
	//任何模式下对MPU6050寄存器的读取，常常读取0x37，可以检测是否正确切换到磁力计模式
	uint8_t     get_data_MPU(uint8_t reg_address);




private:
    I2c         *i2c;
    uint32_t    speed;
	u8 salve_flag;
	u8 salve_adress;

};

#endif


class Mpu9250_Ahrs:public Mpu9250
{
public:
	//构造函数
	Mpu9250_Ahrs(I2c *i2c);
	//原始数据获取，不成功返回-1.-2。成功没有设定返回值，未知返回值
	int Get_MPU9250_Data(void);
	//测试函数：测试初始数据是否成功获取
	void get_data_buf(int16_t *mpu, int16_t *AK);

	//ADC转换，将ADC数据转换为直观数据
	void AHRS_Dataprepare(void);
	//ADC数据转换测试
	void get_data_adc(float *mpu, float *AK);
	//加速度计校正
	void Acc_Correct(void);
	//陀螺仪校正
	void Gyro_Correct(void);
	//磁力计校正
	void Mag_Correct(void);
	//姿态解算得出欧拉角，从外部传入参数
	void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	//调用内部参数
	void AHRSupdate(void);
	//获取欧拉角
	void get_data_ahrs(float *Pitch, float *Roll, float *Yaw);
	//测试代码：测试q0,q1,q2,q3
	void get_data_q(float *q);
	void update_data(void);
	//快速逆平方根
	float invSqrt(float x);
	//参数设置，方便调参数
	void set_parameter(float m_kp, float m_ki, float m_sample);
	//参数大洋，验证参数设置正确与否:比例系数，微分系数，采样率，采样率对应的寄存器设置参数，采样周期的一半
	void get_parameter(float *m_kp, float *m_ki, float *m_samplefreq, uint8_t *sampleH, float *m_halfT);
private:


	
	//处理前数据
	 SENSOR_DATA Gyrobuf;//陀螺仪
	 SENSOR_DATA Accbuf; //加速度
	 SENSOR_DATA Magbuf;//磁力计

	 SENSOR_DATA Accoffset;//加速度偏移量
	 SENSOR_DATA Gyrooffset;//陀螺仪偏移量
	 SENSOR_DATA Magoffset;//磁力计偏移量

	 //处理后的数据
	  IMU_DATA GyroFinal;
	  IMU_DATA AccFinal;
	  IMU_DATA MagFinal;
	  //最终参数
	   float temp;//温度
	   float Pitch;
	   float Roll;
	   float Yaw;
	   float Pitch_off;
	   float Roll_off;
	   float Yaw_off;


	   float q0 , q1  , q2 , q3;	// quaternion of sensor frame relativ
	   float exInt, eyInt, ezInt;
	   float Ki, Kp;
	   float halfT;
	   float  sampleFreq;
};