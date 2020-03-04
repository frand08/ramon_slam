#include <ros.h>
#include "stm32f4xx_hal.h"
#include "mainpp.h"
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <string>
//#include <nmea_msgs/Sentence.h>
#include <sstream>

#include "cmsis_os.h"

// En caso de querer calibrar el magnetometro unicamente uso este define, sino es modo normal
//#define MAG_CALIBRATION

#ifdef MAG_CALIBRATION
void StartMagnetometerTask(void const * argument);
#else
void StartMPU9250Task(void const * argument);
void StartGPSTask(void const * argument);
void StartMovementTask(void const * argument);
void movementCallback(const std_msgs::String& mov);
#endif

extern osMessageQId PublishQueueHandle;
#ifdef MAG_CALIBRATION
extern osThreadId MagnetometerTaskHandle;
#else
extern osThreadId GPSTaskHandle;
extern osThreadId MovementTaskHandle;
extern osThreadId MPU9250TaskHandle;
#endif

/* Node creation */
ros::NodeHandle nh;

extern "C"
{
	void StartPublishTask(void const * argument)
	{
		osEvent event;

		/* Magnetometer publisher */
		sensor_msgs::MagneticField magnetometer;
		ros::Publisher pub_mag("mag_data",&magnetometer);

		/* IMU publisher */
		sensor_msgs::Imu imu;
		ros::Publisher pub_imu("imu_data",&imu);

		/* GPS publisher */
		sensor_msgs::NavSatFix gps;
		ros::Publisher pub_gps("gps_fix",&gps);

	#ifdef MAG_CALIBRATION
		/* Definition of magnetometer_task */
		const osThreadDef_t os_thread_def_magnetometer_task = \
		{ (char*)"magnetometer_task", StartMagnetometerTask, osPriorityNormal, 0, 256};

	#else
		/* Movement subscriber */
		ros::Subscriber<std_msgs::String> sub_movement("move", &movementCallback);

		/* Definition of mpu9250_task */
		const osThreadDef_t os_thread_def_mpu9250_task = \
		{ (char*)"mpu9250_task", StartMPU9250Task, osPriorityNormal, 0, 256};

		/* Definition of gps_task */
		const osThreadDef_t os_thread_def_gps_task = \
		{ (char*)"mpu9250_task", StartGPSTask, osPriorityNormal, 0, 256};


		/* Definition of movement_task */
		const osThreadDef_t os_thread_def_movement_task = \
		{ (char*)"movement_task", StartMovementTask, osPriorityNormal, 0, 256};
		#endif

		nh.initNode();

	#ifdef MAG_CALIBRATION
		nh.advertise(pub_mag);

	#else
		nh.advertise(pub_imu);
//		nh.advertise(pub_gps);
		nh.subscribe(sub_movement);
	#endif

	#ifdef MAG_CALIBRATION
		/* Creation of magnetometer_task */
		MagnetometerTaskHandle = osThreadCreate(&os_thread_def_magnetometer_task, (void*)&magnetometer);

	#else
		/* Creation of mpu9250_task */
		MPU9250TaskHandle = osThreadCreate(&os_thread_def_mpu9250_task, (void*)&imu);

		/* Creation of gps_task */
		GPSTaskHandle = osThreadCreate(&os_thread_def_gps_task, (void*)&gps);

		/* Creation of gps_task */
		MovementTaskHandle = osThreadCreate(&os_thread_def_movement_task, NULL);

	#endif
		HAL_GPIO_WritePin(LD0_GPIO_Port,LD0_Pin,GPIO_PIN_RESET);

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
						/* IMU */
						case 0:
							pub_imu.publish(&imu);
							break;

						/* GPS */
						case 1:
							pub_gps.publish(&gps);
							break;

						/* Magnetometer */
						case 2:
							pub_mag.publish(&magnetometer);
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
