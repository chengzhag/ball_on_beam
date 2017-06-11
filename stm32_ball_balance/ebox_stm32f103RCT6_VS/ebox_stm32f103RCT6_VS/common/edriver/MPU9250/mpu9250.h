#ifndef __MPU9250_H
#define __MPU9250_H

#include "ebox.h"


class MPU9250
{
	float q0 , q1 , q2 , q3 ;//四元数

	struct rx_s
	{
		unsigned char header[3];
		unsigned char cmd;
	};

	struct hal_s
	{
		unsigned char sensors;
		unsigned char dmp_on;
		unsigned char wait_for_tap;
		volatile unsigned char new_gyro;
		unsigned short report;
		unsigned short dmp_features;
		unsigned char motion_int_mode;
		struct rx_s rx;
	}hal;

	/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
	* because it's declared extern elsewhere.
	*/
	volatile unsigned char rx_new;

	/* The sensors can be mounted onto the board in any orientation. The mounting
	* matrix seen below tells the MPL how to rotate the raw data from thei
	* driver(s).
	* TODO: The following matrices refer to the configuration on an internal test
	* board at Invensense. If needed, please modify the matrices to match the
	* chip-to-body matrix for your particular set up.
	*/
	signed char gyro_orientation[9];

	enum packet_type_e
	{
		PACKET_TYPE_ACCEL,
		PACKET_TYPE_GYRO,
		PACKET_TYPE_QUAT,
		PACKET_TYPE_TAP,
		PACKET_TYPE_ANDROID_ORIENT,
		PACKET_TYPE_PEDO,
		PACKET_TYPE_MISC
	};


	//声明相关变量
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];

	//设置参数
	uint16_t sampleRate;
	Uart* uart;

	//私有函数

	//转换安装角矩阵
	/* These next two functions converts the orientation matrix (see
	* gyro_orientation) to a scalar representation for use by the DMP.
	* NOTE: These functions are borrowed from Invensense's MPL.
	*/
	unsigned short inv_row_2_scale(const signed char *row);

	unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);

public:

	MPU9250();

	

	//校准，放置水平
	void calibrate(void);

	//初始化mpu9250和dmp
	void begin(uint16_t sampleRate=200);

	//获取姿态角
	void readDMP(float &pitch, float &roll, float &yaw);


	//以下设置均在初始化之前进行

	//设置采样率
	void setSampleRate(uint16_t sampleRate);
	//通过3*3矩阵设置安装方向
	void setOrientationMatrix(signed char* matrix);
	//设置使能的传感器

	//设置debug用uart
	void setUartDebug(Uart* uartX);
};








#endif
