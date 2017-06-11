#include "mpu9250.h"

#include "stm32f10x.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "stm32_iic.h"

//#define __MPU9250_DEBUG

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define q30  1073741824.0f


//误差纠正
#define  Pitch_error  /*1.0*/0
#define  Roll_error   /*-2.0*/0
#define  Yaw_error    0.0


MPU9250::MPU9250() :
	q0(1), q1(0), q2(0), q3(0),
	sampleRate(200),
	uart(&uart1)
{
	signed char defaultMatrix[] = { 1,0,0,
	0,1,0,
	0,0,1 };
	setOrientationMatrix(defaultMatrix);
}

void MPU9250::setOrientationMatrix(signed char* matrix)
{
	for (int i = 0; i < 9; i++)
	{
		gyro_orientation[i] = matrix[i];
	}
}

void MPU9250::setUartDebug(Uart* uartX)
{
	uart = uartX;
}

unsigned short MPU9250::inv_row_2_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}

unsigned short MPU9250::inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;

	/*
	XYZ  010_001_000 Identity Matrix
	XZY  001_010_000
	YXZ  010_000_001
	YZX  000_010_001
	ZXY  001_000_010
	ZYX  000_001_010
	*/

	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;


	return scalar;
}

void MPU9250::calibrate(void)
{
	int result;

	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
	if (result == 0x7)
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		uart->printf("setting bias succesfully ......\n");
	}
	else
	{
		uart->printf("bias has not been modified ......\n");
	}
}

void MPU9250::begin(uint16_t sampleRate)
{
	setSampleRate(sampleRate);
	int result = 0;

	i2cInit();      //IIC总线的初始化
	//i2c2.take_i2c_right(500000);
	//i2c2.begin(500000);
	//i2c2.release_i2c_right();


	uart->printf("mpu initialization......\n ");
	result = mpu_init();

	if (!result)   //返回0代表初始化成功
	{
		uart->printf("mpu initialization complete......\n ");

		//mpu_set_sensor
		if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
		{
			uart->printf("mpu_set_sensor complete ......\n");
		}
		else
		{
			uart->printf("mpu_set_sensor come across error ......\n");
		}

		//mpu_configure_fifo
		if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
		{
			uart->printf("mpu_configure_fifo complete ......\n");
		}
		else
		{
			uart->printf("mpu_configure_fifo come across error ......\n");
		}

		//mpu_set_sample_rate
		if (!mpu_set_sample_rate(sampleRate))
		{
			uart->printf("mpu_set_sample_rate complete ......\n");
		}
		else
		{
			uart->printf("mpu_set_sample_rate error ......\n");
		}

		//dmp_load_motion_driver_firmvare
		if (!dmp_load_motion_driver_firmware())
		{
			uart->printf("dmp_load_motion_driver_firmware complete ......\n");
		}
		else
		{
			uart->printf("dmp_load_motion_driver_firmware come across error ......\n");
		}

		//dmp_set_orientation
		if (!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
		{
			uart->printf("dmp_set_orientation complete ......\n");
		}
		else
		{
			uart->printf("dmp_set_orientation come across error ......\n");
		}

		//dmp_enable_feature
		if (!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
			DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
			DMP_FEATURE_GYRO_CAL))
		{
			uart->printf("dmp_enable_feature complete ......\n");
		}
		else
		{
			uart->printf("dmp_enable_feature come across error ......\n");
		}

		//dmp_set_fifo_rate
		if (!dmp_set_fifo_rate(sampleRate))
		{
			uart->printf("dmp_set_fifo_rate complete ......\n");
		}
		else
		{
			uart->printf("dmp_set_fifo_rate come across error ......\n");
		}

		//runSelfTest();

		if (!mpu_set_dmp_state(1))
		{
			uart->printf("mpu_set_dmp_state complete ......\n");
		}
		else
		{
			uart->printf("mpu_set_dmp_state come across error ......\n");
		}

	}
}

void MPU9250::readDMP(float &pitch, float &roll, float &yaw)
{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
		/* Gyro and accel data are written to the FIFO by the DMP in chip
		* frame and hardware units. This behavior is convenient because it
		* keeps the gyro and accel outputs of dmp_read_fifo and
		* mpu_read_fifo consistent.
		*/
		/*     if (sensors & INV_XYZ_GYRO )
		send_packet(PACKET_TYPE_GYRO, gyro);
		if (sensors & INV_XYZ_ACCEL)
		send_packet(PACKET_TYPE_ACCEL, accel); */
		/* Unlike gyro and accel, quaternions are written to the FIFO in
		* the body frame, q30. The orientation is set by the scalar passed
		* to dmp_set_orientation during initialization.
		*/
		/*四元数解姿态*/
		if (sensors & INV_WXYZ_QUAT)
		{
			q0 = quat[0] / q30;
			q1 = quat[1] / q30;
			q2 = quat[2] / q30;
			q3 = quat[3] / q30;

			pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + Pitch_error; // pitch
			roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + Roll_error; // roll
			yaw = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3 + Yaw_error;
		}
}

void MPU9250::setSampleRate(uint16_t sampleRate)
{
	this->sampleRate = sampleRate;
}
