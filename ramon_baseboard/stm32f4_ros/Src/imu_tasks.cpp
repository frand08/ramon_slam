#include <ros.h>
#include "MPU9250.h"
#include <sensor_msgs/Imu.h>
#include <ramon_msgs/ImuWithMag.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "mainpp.h"

extern I2C_HandleTypeDef hi2c1;

extern osThreadId MPU9250TaskHandle;
extern osThreadId IMUCalibrationTaskHandle;

extern osMessageQId IMUQueueHandle;

extern ros::NodeHandle nh;

extern osMessageQId PublishQueueHandle;

void i2c_read_bytes(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t * dest)
{
	HAL_I2C_Mem_Read(&hi2c1,address<<1,subAddress,I2C_MEMADD_SIZE_8BIT,&dest[0],(uint16_t)count,100);
}


void i2c_write_bytes(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t *data)
{
	HAL_I2C_Mem_Write(&hi2c1,address<<1,subAddress,I2C_MEMADD_SIZE_8BIT,&data[0],(uint16_t)count,100);
}

void StartMPU9250Task(void const * argument)
{
	sensor_msgs::Imu *imu_send;

	accel_data accel;
	gyro_data gyro;
	mag_data mag;
	float temp;
	float q[4];
	bool first_time = true;
	osEvent event;

//	c_MPU9250 mpu9250(&HAL_Delay,&i2c_read_bytes,&i2c_write_bytes);

	float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3

	MPU9250_params params;
	params.accel_scale = c_MPU9250::E_ACC_SCALE::AFS_2G;
	params.gyro_scale = c_MPU9250::E_GYRO_SCALE::GFS_250DPS;
	params.mag_scale = c_MPU9250::E_MAG_SCALE::MFS_16BITS;
	params.mag_mode = c_MPU9250::E_MAG_HZ::MFREQ_100HZ;
	params.mag_offset_bias[0] = +187.5;
	params.mag_offset_bias[1] = +190.5;
	params.mag_offset_bias[2] = -131.5;
	params.mag_scale_bias[0] = 1.1395;
	params.mag_scale_bias[1] = 0.9245;
	params.mag_scale_bias[2] = 0.9608;
	params.delay_ms = &HAL_Delay;
	params.read_bytes = &i2c_read_bytes;
	params.write_bytes = &i2c_write_bytes;
	params.beta = sqrt(3.0f / 4.0f) * GyroMeasError;

	c_MPU9250 mpu9250(params);


	imu_send = (sensor_msgs::Imu *) argument;

	if(!mpu9250.init())
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
		osThreadTerminate(MPU9250TaskHandle);
	}

	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
	//Covarianzas: Se toman solo las varianzas (xx,yy,zz), las covarianzas son despreciadas

	//Total RMS Noise del gyro = 0.1 º/s-rms (Datasheet)
	//RMS = MEAN² + VAR, con VAR = SIGMA² -> Como es de ruido : MEAN = 0
	//Esto ya es a 1 sigma ya que asi se define a la varianza

	/* FIXME: Revisar valores de covarianzas, sobre todo los factores que se multiplican, si son necesarios o no*/
	//Quiero que este en rad/s
	imu_send->angular_velocity_covariance[0] = 0.1f*PI/180.0f;
	imu_send->angular_velocity_covariance[1] = 0;
	imu_send->angular_velocity_covariance[2] = 0;
	imu_send->angular_velocity_covariance[3] = 0;
	imu_send->angular_velocity_covariance[4] = 0.1f*PI/180.0f;
	imu_send->angular_velocity_covariance[5] = 0;
	imu_send->angular_velocity_covariance[6] = 0;
	imu_send->angular_velocity_covariance[7] = 0;
	imu_send->angular_velocity_covariance[8] = 0.1f*PI/180.0f;

	//Total RMS Noise del acc = 8 mg-rms (Datasheet)
	//RMS = MEAN² + VAR, con VAR = SIGMA² -> Como es de ruido : MEAN = 0
	//Esto ya es a 1 sigma ya que asi se define a la varianza

	//Quiero que este en m/s²
	imu_send->linear_acceleration_covariance[0] = 0.008f*GRAVITY;
	imu_send->linear_acceleration_covariance[1] = 0;
	imu_send->linear_acceleration_covariance[2] = 0;
	imu_send->linear_acceleration_covariance[3] = 0;
	imu_send->linear_acceleration_covariance[4] = 0.008f*GRAVITY;
	imu_send->linear_acceleration_covariance[5] = 0;
	imu_send->linear_acceleration_covariance[6] = 0;
	imu_send->linear_acceleration_covariance[7] = 0;
	imu_send->linear_acceleration_covariance[8] = 0.008f*GRAVITY;

	imu_send->orientation_covariance[0] = 0.01745f;
	imu_send->orientation_covariance[1] = 0.0f;
	imu_send->orientation_covariance[2] = 0.0f;
	imu_send->orientation_covariance[3] = 0.0f;
	imu_send->orientation_covariance[4] = 0.01745f;
	imu_send->orientation_covariance[5] = 0.0f;
	imu_send->orientation_covariance[6] = 0.0f;
	imu_send->orientation_covariance[7] = 0.0f;
	imu_send->orientation_covariance[8] = 0.15708f;
//	imu_send->header.stamp = nh.now();

	for(;;)
	{
		// Receives the deltat between each IMU measurement
		event = osMessageGet(IMUQueueHandle,osWaitForever);
		if(event.status == osEventMessage)
		{
			if(nh.connected())
			{
				// Update ROS time
				imu_send->header.stamp.sec = nh.now().sec;
				imu_send->header.stamp.nsec = nh.now().nsec;
			}
			// Esta en NED tomando ref del magnetometro

			mpu9250.read_accel_data(accel);  // Read the x/y/z adc values
			mpu9250.read_gyro_data(gyro);  // Read the x/y/z adc values
			mpu9250.read_mag_data(mag);  // Read the x/y/z adc values
			mpu9250.read_temp_data(temp);  // Read the adc values


			if(first_time)
			{
				first_time = false;
			}
			else
			{
				// Calculo de quaternions
				// Busco NED (o no?), y como el gyro y accel son:
				// 				Y+
				//				^
				//				|
				//				|
				//				·----> X+
				//				Z+
				// Y el magnetometro
				// 				X+
				//				^
				//				|
				//				|
				//				x----> Y+
				//				Z+
				// Paso el gyro y accel al coordinate frame del mag (que es NED)

				// So if we want te quaternions properly aligned we need to feed
				// into the Madgwick function
				// Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity
				// is by convention positive down, we need to invert the accel data,
				// so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the
				// Madgwick function to get North along the accel +x-axis, East along
				// the accel -y-axis, and Down along the accel -z-axis.
				mpu9250.madgwick_quaternion_update(
						-accel.x, accel.y, accel.z,
						gyro.x*PI/180.0f, -gyro.y*PI/180.0f, -gyro.z*PI/180.0f,
						mag.y,  -mag.x, mag.z,
						q,
						float(event.value.v/1000000.0f));

				/* raw linear acceleration to m/sec² */
				imu_send->linear_acceleration.x = accel.x*GRAVITY;
				imu_send->linear_acceleration.y = accel.y*GRAVITY;
				imu_send->linear_acceleration.z = accel.z*GRAVITY;

				/* raw angular velocity to rad/sec */
				imu_send->angular_velocity.x = gyro.x*PI/180.0f;
				imu_send->angular_velocity.y = gyro.y*PI/180.0f;
				imu_send->angular_velocity.z = gyro.z*PI/180.0f;

				imu_send->orientation.w = q[0];
				imu_send->orientation.x = q[1];
				imu_send->orientation.y = q[2];
				imu_send->orientation.z = q[3];

				/* FIXME: Ver si osWaitForever es adecuado, ademas de cambiar arg */
				osMessagePut(PublishQueueHandle,0,0);
			}
		}
	}
}

void StartIMUCalibrationTask(void const * argument)
{
	ramon_calibration::ImuWithMag *imucal_send;

	accel_data accel;
	gyro_data gyro;
	mag_data mag;
	float temp;

	osEvent event;

	MPU9250_params params;
	params.accel_scale = c_MPU9250::E_ACC_SCALE::AFS_2G;
	params.gyro_scale = c_MPU9250::E_GYRO_SCALE::GFS_250DPS;
	params.mag_scale = c_MPU9250::E_MAG_SCALE::MFS_16BITS;
	params.mag_mode = c_MPU9250::E_MAG_HZ::MFREQ_100HZ;

	params.delay_ms = &HAL_Delay;
	params.read_bytes = &i2c_read_bytes;
	params.write_bytes = &i2c_write_bytes;

	c_MPU9250 mpu9250(params);


	imucal_send = (ramon_calibration::ImuWithMag *) argument;

	if(!mpu9250.init())
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
		osThreadTerminate(IMUCalibrationTaskHandle);
	}

	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
	//Covarianzas: Se toman solo las varianzas (xx,yy,zz), las covarianzas son despreciadas

	//Total RMS Noise del gyro = 0.1 º/s-rms (Datasheet)
	//RMS = MEAN² + VAR, con VAR = SIGMA² -> Como es de ruido : MEAN = 0
	//Esto ya es a 1 sigma ya que asi se define a la varianza

	/* FIXME: Revisar valores de covarianzas, sobre todo los factores que se multiplican, si son necesarios o no*/
	//Quiero que este en rad/s
	imucal_send->angular_velocity_covariance[0] = 0.1f*PI/180.0f;
	imucal_send->angular_velocity_covariance[1] = 0;
	imucal_send->angular_velocity_covariance[2] = 0;
	imucal_send->angular_velocity_covariance[3] = 0;
	imucal_send->angular_velocity_covariance[4] = 0.1f*PI/180.0f;
	imucal_send->angular_velocity_covariance[5] = 0;
	imucal_send->angular_velocity_covariance[6] = 0;
	imucal_send->angular_velocity_covariance[7] = 0;
	imucal_send->angular_velocity_covariance[8] = 0.1f*PI/180.0f;

	//Total RMS Noise del acc = 8 mg-rms (Datasheet)
	//RMS = MEAN² + VAR, con VAR = SIGMA² -> Como es de ruido : MEAN = 0
	//Esto ya es a 1 sigma ya que asi se define a la varianza

	//Quiero que este en m/s²
	imucal_send->linear_acceleration_covariance[0] = 0.008f*GRAVITY;
	imucal_send->linear_acceleration_covariance[1] = 0;
	imucal_send->linear_acceleration_covariance[2] = 0;
	imucal_send->linear_acceleration_covariance[3] = 0;
	imucal_send->linear_acceleration_covariance[4] = 0.008f*GRAVITY;
	imucal_send->linear_acceleration_covariance[5] = 0;
	imucal_send->linear_acceleration_covariance[6] = 0;
	imucal_send->linear_acceleration_covariance[7] = 0;
	imucal_send->linear_acceleration_covariance[8] = 0.008f*GRAVITY;

	// Magnetometer covariance
	/* TODO: define mag covariance */
	imucal_send->magnetic_field_covariance[0] = 1;
	imucal_send->magnetic_field_covariance[1] = 0;
	imucal_send->magnetic_field_covariance[2] = 0;
	imucal_send->magnetic_field_covariance[3] = 0;
	imucal_send->magnetic_field_covariance[4] = 1;
	imucal_send->magnetic_field_covariance[5] = 0;
	imucal_send->magnetic_field_covariance[6] = 0;
	imucal_send->magnetic_field_covariance[7] = 0;
	imucal_send->magnetic_field_covariance[8] = 1;

	osDelay(1000);
	for(;;)
	{
			event = osMessageGet(IMUQueueHandle,osWaitForever);
			if(event.status == osEventMessage)
			{
				if(nh.connected())
				{
					// Update ROS time
					imucal_send->header.stamp.sec = nh.now().sec;
					imucal_send->header.stamp.nsec = nh.now().nsec;
				}

				mpu9250.read_accel_data_uncalibrated(accel);  // Read the x/y/z adc values
				mpu9250.read_gyro_data_uncalibrated(gyro);  // Read the x/y/z adc values
				mpu9250.read_mag_data_uncalibrated(mag);  // Read the x/y/z adc values
//				mpu9250.read_temp_data(temp);  // Read the adc values

				/* uncalibrated linear_acceleration in m/s^2 */
				imucal_send->linear_acceleration.x = accel.x*GRAVITY;
				imucal_send->linear_acceleration.y = accel.y*GRAVITY;
				imucal_send->linear_acceleration.z = accel.z*GRAVITY;

				/* uncalibrated angular velocity to rad/sec */
				imucal_send->angular_velocity.x = gyro.x*PI/180.0f;
				imucal_send->angular_velocity.y = gyro.y*PI/180.0f;
				imucal_send->angular_velocity.z = gyro.z*PI/180.0f;

				/* uncalibrated magnetic_field in Tesla */
				/* 1 miliGauss = 1e-7 Tesla */
				imucal_send->magnetic_field.x = mag.x / 10000000.0;
				imucal_send->magnetic_field.y = mag.y / 10000000.0;
				imucal_send->magnetic_field.z = mag.z / 10000000.0;

				/* Send message */
				osMessagePut(PublishQueueHandle,2,0);
			}
	}
}
