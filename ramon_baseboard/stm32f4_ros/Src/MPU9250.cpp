/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */

#include "MPU9250.h"
#include <sensor_msgs/Imu.h>
#include <math.h>
#include "cmsis_os.h"


/* PUBLIC FUNCTIONS */


c_MPU9250::c_MPU9250(void(*delay_func_ptr)(uint32_t),
					 void(*read_func_ptr)(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t *data),
					 void(*write_func_ptr)(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t *data))
{
	// parameters for 6 DoF sensor fusion calculations
	/* FIXME: GyroMeasError y todos los otros? que onda? */
	float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
//	float GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//	float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	accel_scale_ = c_MPU9250::E_ACC_SCALE::AFS_16G;     				// AFS_2G, AFS_4G, AFS_8G, AFS_16G
	gyro_scale_ = c_MPU9250::E_GYRO_SCALE::GFS_250DPS; 					// GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

	mag_scale_ = c_MPU9250::E_MAG_SCALE::MFS_16BITS; 					// MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
	mag_mode_ = E_MAG_HZ::MFREQ_100HZ;        	// Either 8 Hz (0x02) or 100 Hz (0x06) magnetometer data ODR

	// vector to hold quaternion
	q_[0] = 1.0f;
	q_[1] = 0.0f;
	q_[2] = 0.0f;
	q_[3] = 0.0f;

	// vector to hold integral error for Mahony method
	e_int_[0] = 0.0f;
	e_int_[1] = 0.0f;
	e_int_[2] = 0.0f;

	beta_ = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta

	mag_offset_bias_[0] = +187.5;  // User environmental x-axis correction
    mag_offset_bias_[1] = +190.5;  // User environmental y-axis correction
    mag_offset_bias_[2] = -131.5;  // User environmental z-axis correction

    mag_scale_bias_[0] = 1.1395;
    mag_scale_bias_[1] = 0.9245;
    mag_scale_bias_[2] = 0.9608;

    this->f_delay_ms = delay_func_ptr;
    this->f_read_bytes = read_func_ptr;
    this->f_write_bytes = write_func_ptr;
}

c_MPU9250::c_MPU9250(MPU9250_params params)
{
	accel_scale_ = params.accel_scale;     				// AFS_2G, AFS_4G, AFS_8G, AFS_16G
	gyro_scale_ = params.gyro_scale; 					// GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

	mag_scale_ = params.mag_scale; 					// MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
	mag_mode_ = params.mag_mode;        // Either 8 Hz (0x02) or 100 Hz (0x06) magnetometer data ODR

	// vector to hold quaternion
	q_[0] = 1.0f;
	q_[1] = 0.0f;
	q_[2] = 0.0f;
	q_[3] = 0.0f;

	// vector to hold integral error for Mahony method
	e_int_[0] = 0.0f;
	e_int_[1] = 0.0f;
	e_int_[2] = 0.0f;

	beta_ = params.beta;  	// compute beta

	mag_offset_bias_[0] = params.mag_offset_bias[0];  // User environmental x-axis correction in milliGauss, should be automatically calculated
    mag_offset_bias_[1] = params.mag_offset_bias[1];  // User environmental x-axis correction in milliGauss
    mag_offset_bias_[2] = params.mag_offset_bias[2];  // User environmental x-axis correction in milliGauss

    mag_scale_bias_[0] = params.mag_scale_bias[0];
    mag_scale_bias_[1] = params.mag_scale_bias[1];
    mag_scale_bias_[2] = params.mag_scale_bias[2];

    this->f_delay_ms = params.delay_ms;
    this->f_read_bytes = params.read_bytes;
    this->f_write_bytes = params.write_bytes;
}


c_MPU9250::~c_MPU9250()
{

}


bool c_MPU9250::init(void)
{
	uint8_t whoami = 0, count = 0;
	bool ret = true;

	// Read the WHO_AM_I register, this is a good test of communication
	while(count < 10 && whoami != 0x71)
	{
		this->f_read_bytes(MPU9250_ADDRESS, WHO_AM_I_MPU9250,1,&whoami);  // Read WHO_AM_I register for MPU-9250
		count++;
		this->f_delay_ms(1);
	}

	count = 0;

	if (whoami == 0x71) // WHO_AM_I should always be 0x68
	{
		this->f_delay_ms(1);

		this->f_reset_mpu9250(); // Reset registers to default in preparation for device calibration
		this->f_calibrate_mpu9250(); // Calibrate gyro and accelerometers, load biases in bias registers

//		mpu9250.writeByte(MPU9250_ADDRESS, FIFO_EN, 0x01111000);      // Enable FIFO: GYRO, ACCEL
//		mpu9250.writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00010000);   // Interrupt enable -> FIFO Overflow

		// Init
		this->f_delay_ms(2);
		this->f_init_mpu6500();
		this->f_delay_ms(1);
		this->f_init_ak8963();
		this->f_delay_ms(2);
	}
	else
	{
		ret = false;
	}
	return ret;
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void c_MPU9250::madgwick_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *q, float deltat)
{
	float q1 = q_[0], q2 = q_[1], q3 = q_[2], q4 = q_[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q_[0] = q1 * norm;
	q_[1] = q2 * norm;
	q_[2] = q3 * norm;
	q_[3] = q4 * norm;
	q[0] = q_[0];
	q[1] = q_[1];
	q[2] = q_[2];
	q[3] = q_[3];
}



// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void c_MPU9250::mahony_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *q, float deltat)
{
	float q1 = q_[0], q2 = q_[1], q3 = q_[2], q4 = q_[3];   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		e_int_[0] += ex;      // accumulate integral error
		e_int_[1] += ey;
		e_int_[2] += ez;
	}
	else
	{
		e_int_[0] = 0.0f;     // prevent integral wind up
		e_int_[1] = 0.0f;
		e_int_[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * e_int_[0];
	gy = gy + Kp * ey + Ki * e_int_[1];
	gz = gz + Kp * ez + Ki * e_int_[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q_[0] = q1 * norm;
	q_[1] = q2 * norm;
	q_[2] = q3 * norm;
	q_[3] = q4 * norm;
	q[0] = q_[0];
	q[1] = q_[1];
	q[2] = q_[2];
	q[3] = q_[3];
}


void c_MPU9250::read_accel_data(accel_data &accel)
{
	  int16_t data[3];

	  this->read_accel_data_raw(data);

	  accel.x = (float)data[0]*this->f_get_accel_res() - accel_bias_[0];  // get actual g value, this depends on scale being set
	  accel.y = (float)data[1]*this->f_get_accel_res() - accel_bias_[1];
	  accel.z = (float)data[2]*this->f_get_accel_res() - accel_bias_[2];
}

void c_MPU9250::read_accel_data_raw(int16_t *data)
{
	uint8_t rawData[6];
	  this->f_read_bytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	  data[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  data[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	  data[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void c_MPU9250::read_accel_data_uncalibrated(accel_data &accel)
{
	  int16_t data[3];

	  this->read_accel_data_raw(data);

	  accel.x = (float)data[0]*this->f_get_accel_res();  // get actual g value, this depends on scale being set
	  accel.y = (float)data[1]*this->f_get_accel_res();
	  accel.z = (float)data[2]*this->f_get_accel_res();
}

void c_MPU9250::read_gyro_data(gyro_data &gyro)
{
	  int16_t data[3];

	  this->read_gyro_data_raw(data);

	  gyro.x = (float)data[0]*this->f_get_gyro_res() - gyro_bias_[0];  // get actual gyro value, this depends on scale being set
	  gyro.y = (float)data[1]*this->f_get_gyro_res() - gyro_bias_[1];
	  gyro.z = (float)data[2]*this->f_get_gyro_res() - gyro_bias_[2];
}

void c_MPU9250::read_gyro_data_raw(int16_t *data)
{
	uint8_t rawData[6];
	this->f_read_bytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	data[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	data[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	data[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void c_MPU9250::read_gyro_data_uncalibrated(gyro_data &gyro)
{
	  int16_t data[3];

	  this->read_gyro_data_raw(data);

	  gyro.x = (float)data[0]*this->f_get_gyro_res();  // get actual gyro value, this depends on scale being set
	  gyro.y = (float)data[1]*this->f_get_gyro_res();
	  gyro.z = (float)data[2]*this->f_get_gyro_res();
}

void c_MPU9250::read_mag_data(mag_data &mag)
{

	int16_t data[3];

	if(this->read_mag_data_raw(data) >= 0)
	{
		// Offsets from calibration (MATLAB)
		mag.x = ((float)data[0] - mag_offset_bias_[0])*mag_scale_bias_[0]*this->f_get_mag_res()*mag_calibration_[0];	// get actual magnetometer value, this depends on scale being set
		mag.y = ((float)data[1] - mag_offset_bias_[1])*mag_scale_bias_[1]*this->f_get_mag_res()*mag_calibration_[1];
		mag.z = ((float)data[2] - mag_offset_bias_[2])*mag_scale_bias_[2]*this->f_get_mag_res()*mag_calibration_[2];
	}
}

void c_MPU9250::read_mag_data_uncalibrated(mag_data &mag)
{

	int16_t data[3];

	if(this->read_mag_data_raw(data) >= 0)
	{
		// Mag in miliGauss
		mag.x = (float)data[0]*this->f_get_mag_res();	// get actual magnetometer value, this depends on scale being set
		mag.y = (float)data[1]*this->f_get_mag_res();
		mag.z = (float)data[2]*this->f_get_mag_res();
	}
}

int c_MPU9250::read_mag_data_raw(int16_t *rawData)
{
	uint8_t c, auxData[7];
	this->f_read_bytes(AK8963_ADDRESS, AK8963_ST1,1,&c);
	if(c & 0x01)  // wait for magnetometer data ready bit to be set
	{
	this->f_read_bytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &auxData[0]);  // Read the six raw data and ST2 registers sequentially into data array
	c = auxData[6]; // End data read by reading ST2 register
	if(!(c & 0x08)) // Check if magnetic sensor overflow set, if not then report data
	{
		rawData[0] = (int16_t)(((int16_t)auxData[1] << 8) | auxData[0]) ;  // Turn the MSB and LSB into a signed 16-bit value
		rawData[1] = (int16_t)(((int16_t)auxData[3] << 8) | auxData[2]) ;  // Data stored as little Endian
		rawData[2] = (int16_t)(((int16_t)auxData[5] << 8) | auxData[4]) ;
		return 0;
	}
}
	  return -1;
}
void c_MPU9250::read_temp_data(float &temp)
{
	  int16_t rawTemp;
	  this->read_accel_data_raw(&rawTemp);
	  temp = ((float) rawTemp) / 333.87f + 21.0f; // Temperature in degrees Centigrade

}
void c_MPU9250::read_temp_data_raw(int16_t *rawTemp)
{
	  uint8_t rawData[2];  // x/y/z gyro register data stored here
	  this->f_read_bytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
	  *rawTemp = (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]);  // Turn the MSB and LSB into a 16-bit value

}

/* PRIVATE FUNCTIONS */


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void c_MPU9250::f_calibrate_mpu9250(void)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint8_t data_aux;	// Para escribir en el I2C
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	data_aux = 0x80;
	this->f_write_bytes(MPU9250_ADDRESS, PWR_MGMT_1, 1, &data_aux); // Write a one to bit 7 reset bit; toggle reset device
	this->f_delay_ms(100);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	data_aux = 0x01;
	this->f_write_bytes(MPU9250_ADDRESS, PWR_MGMT_1, 1, &data_aux);
	data_aux = 0x00;
	this->f_write_bytes(MPU9250_ADDRESS, PWR_MGMT_2, 1, &data_aux);
	this->f_delay_ms(200);

	// Configure device for bias calculation
	this->f_write_bytes(MPU9250_ADDRESS, INT_ENABLE, 1, &data_aux);   // Disable all interrupts
	this->f_write_bytes(MPU9250_ADDRESS, FIFO_EN, 1, &data_aux);      // Disable FIFO
	this->f_write_bytes(MPU9250_ADDRESS, PWR_MGMT_1, 1, &data_aux);   // Turn on internal clock source
	this->f_write_bytes(MPU9250_ADDRESS, I2C_MST_CTRL, 1, &data_aux); // Disable I2C master
	this->f_write_bytes(MPU9250_ADDRESS, USER_CTRL, 1, &data_aux);    // Disable FIFO and I2C master modes
	data_aux = 0x0C;
	this->f_write_bytes(MPU9250_ADDRESS, USER_CTRL, 1, &data_aux);    // Reset FIFO and DMP
	this->f_delay_ms(15);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	data_aux = 0x01;
	this->f_write_bytes(MPU9250_ADDRESS, CONFIG, 1, &data_aux);      // Set low-pass filter to 188 Hz
	data_aux = 0x00;
	this->f_write_bytes(MPU9250_ADDRESS, SMPLRT_DIV, 1, &data_aux);  // Set sample rate to 1 kHz
	this->f_write_bytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, &data_aux);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	this->f_write_bytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &data_aux); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	data_aux = 0x40;
	this->f_write_bytes(MPU9250_ADDRESS, USER_CTRL, 1, &data_aux);   // Enable FIFO
	data_aux = 0x78;
	this->f_write_bytes(MPU9250_ADDRESS, FIFO_EN, 1, &data_aux);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	this->f_delay_ms(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	data_aux = 0x00;
	this->f_write_bytes(MPU9250_ADDRESS, FIFO_EN, 1, &data_aux);        // Disable gyro and accelerometer sensors for FIFO
	this->f_read_bytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		this->f_read_bytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	/// Push gyro biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
	*/
	gyro_bias_[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	gyro_bias_[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	gyro_bias_[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	this->f_read_bytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	this->f_read_bytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	this->f_read_bytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++)
	{
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	*/
	// Output scaled accelerometer biases for manual subtraction in the main program
	accel_bias_[0] = (float)accel_bias[0]/(float)accelsensitivity;
	accel_bias_[1] = (float)accel_bias[1]/(float)accelsensitivity;
	accel_bias_[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


float c_MPU9250::f_get_accel_res()
{
	float accel_res;
	switch (accel_scale_)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
			// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case E_ACC_SCALE::AFS_2G:
			accel_res = 2.0/32768.0;
			break;
		case E_ACC_SCALE::AFS_4G:
			accel_res = 4.0/32768.0;
			break;
		case E_ACC_SCALE::AFS_8G:
			accel_res = 8.0/32768.0;
			break;
		case E_ACC_SCALE::AFS_16G:
			accel_res = 16.0/32768.0;
			break;
		default:
			accel_res = 2.0/32768.0;
			break;
	}
	return accel_res;
}

float c_MPU9250::f_get_gyro_res()
{
	float gyro_res;
	switch (gyro_scale_)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
			// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case E_GYRO_SCALE::GFS_250DPS:
			gyro_res = 250.0/32768.0;
			break;
		case E_GYRO_SCALE::GFS_500DPS:
			gyro_res = 500.0/32768.0;
			break;
		case E_GYRO_SCALE::GFS_1000DPS:
			gyro_res = 1000.0/32768.0;
			break;
		case E_GYRO_SCALE::GFS_2000DPS:
			gyro_res = 2000.0/32768.0;
			break;
		default:
			gyro_res = 250.0/32768.0;
			break;
	}
	return gyro_res;
}


uint8_t c_MPU9250::f_get_mag_mode(uint8_t mag_hz)
{
	float mag_mode;
	switch (mag_hz)
	{
		case E_MAG_HZ::MFREQ_8HZ:
			mag_mode = 0x02;
			break;
		case E_MAG_HZ::MFREQ_100HZ:
			mag_mode = 0x06;
			break;
		default:
			mag_mode = 0x06;
			break;
	}
	return mag_mode;
}

float c_MPU9250::f_get_mag_res()
{
	float mag_res;
	switch (mag_scale_)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case E_MAG_SCALE::MFS_14BITS:
			mag_res = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
			break;
		case E_MAG_SCALE::MFS_16BITS:
			mag_res = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
			break;
		default:
			mag_res = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
			break;
	}
	return mag_res;
}

void c_MPU9250::f_init_ak8963(void)
{
	  // First extract the factory calibration for each magnetometer axis
	  uint8_t rawData[3] = {0};  // x/y/z gyro calibration data stored here
	  uint8_t data = 0;
	  uint8_t data_aux = 0;		// Para hacer los writes

	  data_aux = 0x00;
	  this->f_write_bytes(AK8963_ADDRESS, AK8963_CNTL, 1, &data_aux); // Power down magnetometer
	  this->f_delay_ms(10);
	  data_aux = 0x0F;
	  this->f_write_bytes(AK8963_ADDRESS, AK8963_CNTL, 1, &data_aux); // Enter Fuse ROM access mode
	  this->f_delay_ms(10);

	  this->f_read_bytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	  mag_calibration_[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	  mag_calibration_[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
	  mag_calibration_[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;

	  data_aux = 0x00;
	  this->f_write_bytes(AK8963_ADDRESS, AK8963_CNTL, 1, &data_aux); // Power down magnetometer
	  this->f_delay_ms(10);
	  // Configure the magnetometer for continuous read and highest resolution
	  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	  data_aux = mag_scale_ << 4 | this->f_get_mag_mode(mag_mode_);
	  this->f_write_bytes(AK8963_ADDRESS, AK8963_CNTL, 1, &data_aux); // Set magnetometer data resolution and sample ODR
	  this->f_delay_ms(10);

	  this->f_read_bytes(AK8963_ADDRESS, AK8963_WHO_AM_I,1,&data);
	  if(data != 0x48)  // wait for magnetometer data ready bit to be set
	  {
		  while(1);
	  }
}


void c_MPU9250::f_init_mpu6500()
{
	uint8_t data_aux = 0;	// Para escribir
	 // Initialize MPU6500 device
	 // wake up device
	data_aux = 0x00;
	  this->f_write_bytes(MPU9250_ADDRESS, PWR_MGMT_1, 1, &data_aux); // Clear sleep mode bit (6), enable all sensors
	  this->f_delay_ms(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	 // get stable time source
	  data_aux = 0x01;
	  this->f_write_bytes(MPU9250_ADDRESS, PWR_MGMT_1, 1, &data_aux);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	 // Configure Gyro and Accelerometer
	 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	 // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	  data_aux = 0x03;
	  this->f_write_bytes(MPU9250_ADDRESS, CONFIG, 1, &data_aux);

	 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	  data_aux = 0x04;
	  this->f_write_bytes(MPU9250_ADDRESS, SMPLRT_DIV, 1, &data_aux);  // Use a 200 Hz rate; the same rate set in CONFIG above

	 // Set gyroscope full scale range
	 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3

	  this->f_read_bytes(MPU9250_ADDRESS, GYRO_CONFIG,1,&data_aux); // get current GYRO_CONFIG register value
	 // c = c & ~0xE0; // Clear self-test bits [7:5]
	  data_aux = data_aux & ~0x02; // Clear Fchoice bits [1:0]
	  data_aux = data_aux & ~0x18; // Clear AFS bits [4:3]
	  data_aux = data_aux | gyro_scale_ << 3; // Set full scale range for the gyro
	 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	  this->f_write_bytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, &data_aux); // Write new GYRO_CONFIG value to register

	 // Set accelerometer full-scale range configuration
	  this->f_read_bytes(MPU9250_ADDRESS, ACCEL_CONFIG,1,&data_aux); // get current ACCEL_CONFIG register value
	 // c = c & ~0xE0; // Clear self-test bits [7:5]
	  data_aux = data_aux & ~0x18;  // Clear AFS bits [4:3]
	  data_aux = data_aux | accel_scale_ << 3; // Set full scale range for the accelerometer
	  this->f_write_bytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &data_aux); // Write new ACCEL_CONFIG register value

	 // Set accelerometer sample rate configuration
	 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	  this->f_read_bytes(MPU9250_ADDRESS, ACCEL_CONFIG2,1,&data_aux); // get current ACCEL_CONFIG2 register value
	  data_aux = data_aux & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	  data_aux = data_aux | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	  this->f_write_bytes(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &data_aux); // Write new ACCEL_CONFIG2 register value

	 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	  // Configure Interrupts and Bypass Enable
	  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	  // can join the I2C bus and all can be controlled by the Arduino as master
//	   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	  data_aux = 0x02;
	   this->f_write_bytes(MPU9250_ADDRESS, INT_PIN_CFG, 1, &data_aux);
	   data_aux = 0x01;
	   this->f_write_bytes(MPU9250_ADDRESS, INT_ENABLE, 1, &data_aux);  // Enable data ready (bit 0) interrupt
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void c_MPU9250::f_mpu9250_self_test(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;
	uint8_t data_aux = 0; 	// Para escribir

	data_aux = 0x00;
	this->f_write_bytes(MPU9250_ADDRESS, SMPLRT_DIV, 1, &data_aux); // Set gyro sample rate to 1 kHz
	data_aux = 0x02;
	this->f_write_bytes(MPU9250_ADDRESS, CONFIG, 1, &data_aux); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	data_aux = FS<<3;
	this->f_write_bytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, &data_aux); // Set full scale range for the gyro to 250 dps
	data_aux = 0x02;
	this->f_write_bytes(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &data_aux); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	data_aux = FS<<3;
	this->f_write_bytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &data_aux); // Set full scale range for the accelerometer to 2 g

	for( int ii = 0; ii < 200; ii++) // get average current values of gyro and acclerometer
	{
		this->f_read_bytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		this->f_read_bytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++) // Get average of 200 values and store as average current readings
	{
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	data_aux = 0xE0;
	this->f_write_bytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &data_aux); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	this->f_write_bytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, &data_aux); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
//		delay(25); // Delay a while to let the device stabilize
	this->f_delay_ms(10);
	for( int ii = 0; ii < 200; ii++) // get average self-test values of gyro and acclerometer
	{
		this->f_read_bytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		this->f_read_bytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++) // Get average of 200 values and store as average self-test readings
	{
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	data_aux = 0x00;
	this->f_write_bytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &data_aux);
	this->f_write_bytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, &data_aux);
//   delay(25); // Delay a while to let the device stabilize
	this->f_delay_ms(10);

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	this->f_read_bytes(MPU9250_ADDRESS, SELF_TEST_X_ACCEL,1,&selfTest[0]); // X-axis accel self-test results
	this->f_read_bytes(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL,1,&selfTest[1]); // Y-axis accel self-test results
	this->f_read_bytes(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL,1,&selfTest[2]); // Z-axis accel self-test results
	this->f_read_bytes(MPU9250_ADDRESS, SELF_TEST_X_GYRO,1,&selfTest[3]); // X-axis gyro self-test results
	this->f_read_bytes(MPU9250_ADDRESS, SELF_TEST_Y_GYRO,1,&selfTest[4]); // Y-axis gyro self-test results
	this->f_read_bytes(MPU9250_ADDRESS, SELF_TEST_Z_GYRO,1,&selfTest[5]); // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++)
	{
		destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.; // Report percent differences
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
	}

}

void c_MPU9250::f_reset_mpu9250(void)
{
	  // reset device
	uint8_t data_aux = 0x80;
	  this->f_write_bytes(MPU9250_ADDRESS, PWR_MGMT_1,1, &data_aux); // Write a one to bit 7 reset bit; toggle reset device
	  this->f_delay_ms(100);
}
/*
void c_MPU9250::f_write_bytes(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c_,address,subAddress,I2C_MEMADD_SIZE_8BIT,&data,count,100);
}*/
