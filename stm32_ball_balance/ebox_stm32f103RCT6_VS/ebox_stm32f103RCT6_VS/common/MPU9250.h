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
/*ģʽѡ��0Ϊmpu6500ģʽ��1ΪAK8963ģʽ*/
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

#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
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
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I			0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define SLAVEADDRESS	0xD0	//IICд��ʱ�ĵ�ַ�ֽ�����

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
//AHRS�㷨����
#define betaDef		0.2f		// 2 * proportional gain
/*
#define sampleFreq	125.0f		// sample frequency in Hz  ������ 100 HZ  10ms  �޸Ĵ�Ƶ�ʿ����ӱ仯�ٶ�
#define Kp 2.25f	//��������֧��������accellrometer/magnetometer           //0.6 0.7       2.5
#define Ki 0.0055f //��������ִ�����������ǵ��ν�gyroscopeases           //0.002  0.004   0.005
#define halfT 0.004f//�������ڵ�һ�룬������Ϊ10ms,��һ��Ϊ0.005s        // 1      1      0.2
*/
//������ԭʼ����
typedef struct  sensor_data
{
	short X;
	short Y;
	short Z;
}SENSOR_DATA;
//����������
typedef struct  imu_data
{
	float X;
	float Y;
	float Z;
}IMU_DATA;

class Mpu9250
{
public:
	float sample;   //������

	Mpu9250(I2c *i2c)
    {
        this->i2c = i2c;
    };
    void        begin(uint32_t speed);
	//�趨ģʽ
	void        mode(u8 mode);
	int16_t 	get_data_2byte(uint8_t reg_address);
	//�ֽڶ�ȡ�����ڶ�ȡ�ض��Ĵ���
	int8_t   	get_data_1byte(uint8_t reg_address);
	//������ַ�Ĵ������ݶ�ȡ
    int8_t 		get_data(uint8_t reg_address, int16_t *buf, uint8_t num_to_read);
	//���Ĵ���д����
	void        write_data(u8 reg_address, u8 data);
	//��ȡ�豸id
    void        get_id(uint8_t *id);
	//�κ�ģʽ�¶�MPU6050�Ĵ����Ķ�ȡ��������ȡ0x37�����Լ���Ƿ���ȷ�л���������ģʽ
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
	//���캯��
	Mpu9250_Ahrs(I2c *i2c);
	//ԭʼ���ݻ�ȡ�����ɹ�����-1.-2���ɹ�û���趨����ֵ��δ֪����ֵ
	int Get_MPU9250_Data(void);
	//���Ժ��������Գ�ʼ�����Ƿ�ɹ���ȡ
	void get_data_buf(int16_t *mpu, int16_t *AK);

	//ADCת������ADC����ת��Ϊֱ������
	void AHRS_Dataprepare(void);
	//ADC����ת������
	void get_data_adc(float *mpu, float *AK);
	//���ٶȼ�У��
	void Acc_Correct(void);
	//������У��
	void Gyro_Correct(void);
	//������У��
	void Mag_Correct(void);
	//��̬����ó�ŷ���ǣ����ⲿ�������
	void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	//�����ڲ�����
	void AHRSupdate(void);
	//��ȡŷ����
	void get_data_ahrs(float *Pitch, float *Roll, float *Yaw);
	//���Դ��룺����q0,q1,q2,q3
	void get_data_q(float *q);
	void update_data(void);
	//������ƽ����
	float invSqrt(float x);
	//�������ã����������
	void set_parameter(float m_kp, float m_ki, float m_sample);
	//����������֤����������ȷ���:����ϵ����΢��ϵ���������ʣ������ʶ�Ӧ�ļĴ������ò������������ڵ�һ��
	void get_parameter(float *m_kp, float *m_ki, float *m_samplefreq, uint8_t *sampleH, float *m_halfT);
private:


	
	//����ǰ����
	 SENSOR_DATA Gyrobuf;//������
	 SENSOR_DATA Accbuf; //���ٶ�
	 SENSOR_DATA Magbuf;//������

	 SENSOR_DATA Accoffset;//���ٶ�ƫ����
	 SENSOR_DATA Gyrooffset;//������ƫ����
	 SENSOR_DATA Magoffset;//������ƫ����

	 //����������
	  IMU_DATA GyroFinal;
	  IMU_DATA AccFinal;
	  IMU_DATA MagFinal;
	  //���ղ���
	   float temp;//�¶�
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