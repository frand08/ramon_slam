#ifndef MPU9250_H
#define MPU9250_H

//#include "mbed.h"
#include "math.h"
#include "stm32f4xx_hal.h"

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif  


typedef struct MPU9250_params_t
{
	I2C_HandleTypeDef hi2c;				// I2C connected to MPU9250, eq, hi2c1.

	uint8_t accel_scale;     				// AFS_2G, AFS_4G, AFS_8G, AFS_16G
	uint8_t gyro_scale; 				// GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	uint8_t mag_scale; 					// MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
	uint8_t mag_mode;       			// Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR

	// scale resolutions per LSB for the sensors
	float accel_res;
	float gyro_res;
	float mag_res;

	float deltat;                			// integration interval for both filter schemes
	float q[4];    						// vector to hold quaternion
	float e_int[3];  						// vector to hold integral error for Mahony method

	float beta; 							// compute beta

} MPU9250_params;

typedef struct accel_data_t
{
	float x;
	float y;
	float z;
} accel_data;

typedef struct gyro_data_t
{
	float x;
	float y;
	float z;
} gyro_data;

typedef struct mag_data_t
{
	float x;
	float y;
	float z;
} mag_data;

typedef struct MPU9250_data_t
{
	accel_data accel;
	gyro_data gyro;
	mag_data mag;
	float temp;
	float q[4];
} MPU9250_data;


#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

#define GRAVITY -9.81f		//Valor de la gravedad (modulo)
#define PI 		3.14159265358979323846f

#ifdef __cplusplus

class c_MPU9250
{
    private:
		float m_accel_bias[3];
		uint8_t m_accel_scale;     				// AFS_2G, AFS_4G, AFS_8G, AFS_16G

		float m_beta; 							// compute beta

		float m_deltat;             			// integration interval for both filter schemes

		float m_e_int[3];  						// vector to hold integral error for Mahony method

		float m_gyro_bias[3];
		uint8_t m_gyro_scale; 					// GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

		I2C_HandleTypeDef m_hi2c;

		float m_mag_bias[3];
		float m_mag_calibration[3];				// Factory mag calibration
		uint8_t m_mag_mode;       				// Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
		uint8_t m_mag_scale; 					// MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution

		float m_q[4];    						// vector to hold quaternion


		// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
		// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
		void f_calibrate_mpu9250(void);

		// Puntero a funcion para tomar una funcion externa para el delay de milisegundos
	    void(*f_delay_ms)(uint32_t);
		void(*f_read_bytes)(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t * dest);
		void(*f_write_bytes)(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t *data);

		float f_get_accel_res();
		float f_get_gyro_res();
		uint8_t f_get_mag_mode(uint8_t);
		float f_get_mag_res();

		void f_init_ak8963(void);
		void f_init_mpu9250(void);

		// Accelerometer and gyroscope self test; check calibration wrt factory settings
		void f_mpu9250_self_test(float * destination); // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass

		/* TODO: Generalizar las funciones de read y write byte(s) */
//		uint8_t f_read_byte(uint8_t address, uint8_t subAddress);
//		void f_read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
		void f_reset_mpu9250(void);

//		void f_write_bytes(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t data);

    public:
		// Set initial input parameters
		enum E_ACC_SCALE : uint8_t
		{
		  AFS_2G = 0,
		  AFS_4G,
		  AFS_8G,
		  AFS_16G
		};

		enum E_GYRO_SCALE : uint8_t
		{
		  GFS_250DPS = 0,
		  GFS_500DPS,
		  GFS_1000DPS,
		  GFS_2000DPS
		};

		enum E_MAG_SCALE : uint8_t
		{
		  MFS_14BITS = 0, 	// 0.6 mG per LSB
		  MFS_16BITS      	// 0.15 mG per LSB
		};

		enum E_MAG_HZ : uint8_t
		{
		  MFREQ_8HZ = 0, 		// 8Hz Magnetometer data ODR
		  MFREQ_100HZ   		// 100Hz Magnetometer data ODR
		};

		c_MPU9250(void(*delay_func_ptr)(uint32_t),
				 void(*read_bytes_ptr)(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t *data),
				 void(*write_func_ptr)(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t *data));

		/* TODO: Hacer un constructor adecuado para recibir parametros */
		c_MPU9250(MPU9250_params);
		~c_MPU9250();

		// Check for IMU status and initializes both chips
		bool init(void);

		// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
		// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
		// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
		// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
		// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
		// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
		void madgwick_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *q, float);
		// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
		// measured ones.
		void mahony_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *q, float);

		void read_accel_data(accel_data &);
		void read_gyro_data(gyro_data &);
		void read_mag_data(mag_data &);
		void read_temp_data(float &);
};
#endif
#endif
