#include <ros.h>
#include "stm32f4xx_hal.h"
#include "mainpp.h"
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <ramon_calibration/ImuWithMag.h>
#include <string>
//#include <nmea_msgs/Sentence.h>
#include <sstream>

#include "cmsis_os.h"

void StartMPU9250Task(void const * argument);

#ifdef IMU_CALIBRATION
void StartIMUWithMagnetometerTask(void const * argument);
#endif

void StartGPSTask(void const * argument);

void StartMovementTask(void const * argument);
void movementCallback(const std_msgs::String& mov);

extern osMessageQId PublishQueueHandle;
extern osThreadId IMUWithMagnetometerTaskHandle;
extern osThreadId GPSTaskHandle;
extern osThreadId MovementTaskHandle;
extern osThreadId MPU9250TaskHandle;

/* Node creation */
ros::NodeHandle nh;

extern "C"
{
	void StartPublishTask(void const * argument)
	{
		osEvent event;

	/** Messages and tasks **/

		/* IMU publisher */
		sensor_msgs::Imu imu;
		ros::Publisher pub_imu("imu_data",&imu);

		/* Definition of mpu9250_task */
		const osThreadDef_t os_thread_def_mpu9250_task = \
		{ (char*)"mpu9250_task", StartMPU9250Task, osPriorityNormal, 0, 256};

		/* IMU calibration publisher */
		ramon_calibration::ImuWithMag imu_mag;
		ros::Publisher pub_imumag("imu_cal_data",&imu_mag);

		/* Definition of magnetometer_task */
		const osThreadDef_t os_thread_def_imuwithmagnetometer_task = \
		{ (char*)"magnetometer_task", StartIMUWithMagnetometerTask, osPriorityNormal, 0, 256};

		/* GPS publisher */
		sensor_msgs::NavSatFix gps;
		ros::Publisher pub_gps("gps_fix",&gps);

		/* Definition of gps_task */
		const osThreadDef_t os_thread_def_gps_task = \
		{ (char*)"mpu9250_task", StartGPSTask, osPriorityNormal, 0, 256};

		/* Movement subscriber */
		ros::Subscriber<std_msgs::String> sub_movement("move", &movementCallback);

		/* Definition of movement_task */
		const osThreadDef_t os_thread_def_movement_task = \
		{ (char*)"movement_task", StartMovementTask, osPriorityNormal, 0, 256};

	/** Node init + advertise/subscribe **/

		nh.initNode();

#	ifndef IMU_CALIBRATION
		nh.advertise(pub_imu);
#	else
		nh.advertise(pub_imumag);
#	endif

#	ifdef GPS_DATA_SEND
		nh.advertise(pub_gps);
#	endif

#	ifdef MOVEMENT_DATA_RECEIVE
		nh.subscribe(sub_movement);
#	endif
	/** Creation of tasks **/

#	ifndef IMU_CALIBRATION
		/* Creation of mpu9250_task */
		MPU9250TaskHandle = osThreadCreate(&os_thread_def_mpu9250_task, (void*)&imu);
#	else
		/* Creation of imuwithmagnetometer task */
		IMUWithMagnetometerTaskHandle = osThreadCreate(&os_thread_def_imuwithmagnetometer_task, (void*)&imu_mag);
#	endif

#	ifdef GPS_DATA_SEND
		/* Creation of gps_task */
		GPSTaskHandle = osThreadCreate(&os_thread_def_gps_task, (void*)&gps);
#	endif

#	ifdef MOVEMENT_DATA_RECEIVE
		/* Creation of movement_task */
		MovementTaskHandle = osThreadCreate(&os_thread_def_movement_task, NULL);
#	endif

		HAL_GPIO_WritePin(LD0_GPIO_Port,LD0_Pin,GPIO_PIN_RESET);

	/** Loop **/

		for(;;)
		{
			// Chequea si hubo evento (no lo espera). Si se encuentra conectado, transmite.
			event = osMessageGet(PublishQueueHandle,osWaitForever);
			if(event.status == osEventMessage)
			{
				if(nh.connected())
				{
					switch(event.value.v)
					{
						/* IMU message */
						case 0:
							pub_imu.publish(&imu);
							break;

						/* GPS */
						case 1:
							pub_gps.publish(&gps);
							break;

						/* MPU9250 calibration */
						case 2:
							pub_imumag.publish(&imu_mag);
							break;

						default:
							break;
					}
				}
			}
			nh.spinOnce();
		}
	}

	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
	{
		if(huart->Instance == USART3)
			nh.getHardware()->flush();
	}


	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		// Si no esta esto, rbuflen overflow, que se traduce en:
		// Lost sync with device, restarting...
		if(huart->Instance == USART3)
			nh.getHardware()->reset_rbuf();
	}
}
