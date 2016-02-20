/*
FreeIMU.h - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011 Fabio Varesano <fabio at varesano dot net>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef FreeIMU_h
#define FreeIMU_h
#define FREEIMU_v04

//Magnetic declination angle for iCompass
#define MAG_DEC 4.6  //degrees for Flushing, NY

//Number of samples to average in iCompass
#define WINDOW_SIZE 1 //Set to 1 to turn off the Running Average

// Set filter type: 1 = Madgwick Gradient Descent, 0 - Madgwick implementation of Mahoney DCM
// in Quaternion form, 3 = Madwick Original Paper AHRS, 4 - DCM Implementation
#define MARG 0

// proportional gain governs rate of convergence to accelerometer/magnetometer
// integral gain governs rate of convergence of gyroscope biases
// set up defines for various boards in my inventory, DFROBOT and Freeimu have
// temperature calibration curves. (3.31.14)

#define twoKpDef  (2.0f * 0.75f)	//works with and without mag enabled
#define twoKiDef  (2.0f * 0.1625f)
#define betaDef  0.085f

//Used for DCM filter
const float Kp_ROLLPITCH = 1.2f;  //was .3423
const float Ki_ROLLPITCH = 0.0234f;
const float Kp_YAW = 1.75f;   // was 1.2 and 0.02
const float Ki_YAW = 0.002f;

// Other Options

#define temp_break  -1000	  //original temp_break = -4300;
#define senTemp_break  32
#define temp_corr_on_default  0
#define nsamples 75
#define instability_fix 1

// ****************************************************
// *** No configuration needed below this line      ***
// *** Unless you are defining a new IMU            ***
// ***                                              ***
// *** Define Marg= 3 factors: go to line 491       ***
// *** Define IMU Axis Alignment: go to line 500    ***
// ****************************************************

#define FREEIMU_LIB_VERSION "DEV"
#define FREEIMU_DEVELOPER "Fabio Varesano"
#define FREEIMU_FREQ "16 MHz"
#define FREEIMU_ID "FreeIMU v0.4"

#define HAS_MPU6050()
#define HAS_HMC5883L()
#define HAS_MS5611() 0 
#define HAS_PRESS() 0
#define IS_6DOM() 1
#define IS_9DOM() 1
#define HAS_AXIS_ALIGNED() 1

#include <Wire.h>
#include "I2Cdev.h"
#include "Arduino.h"
#include "calibration.h"
#include <MovingAvarageFilter.h>

#ifndef CALIBRATION_H
	#include <EEPROM.h>
#endif

#define FREEIMU_EEPROM_BASE 0x0A
#define FREEIMU_EEPROM_SIGNATURE 0x19

#if (MARG == 4)
	#include "DCM.h"
#endif

#include "MPU60X0.h"
#define FIMU_ACCGYRO_ADDR MPU60X0_DEFAULT_ADDRESS

#if HAS_PRESS()  //Setup pressure sensors and .h files
	#include <FilteringScheme.h>
	#include <AltitudeComplementary.h>
	#include <MS561101BA.h>
	#define FIMU_BARO_ADDR MS561101BA_ADDR_CSB_LOW	
#endif
  
#include <HMC58X3.h>
#include "iCompass.h"
  // HMC5843 address is fixed so don't bother to define it

#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif


class FreeIMU
{
  public:
    FreeIMU();
	void init();
	void RESET();
	void RESET_Q();
	void init(bool fastmode);
	void init(int accgyro_addr, bool fastmode);
    	
    #ifndef CALIBRATION_H
		void calLoad();
    #endif
	
    void zeroGyro();
	void initGyros();
    void getRawValues(int * raw_values);
    void getValues(float * values);
    void getQ(float * q, float * val);
    void getEuler(float * angles);
    void getYawPitchRoll(float * ypr);
    void getEulerRad(float * angles);
    void getYawPitchRollRad(float * ypr);
	float invSqrt(float x);
	//void setTempCalib(int opt_temp_cal);
	//void setSeaPress(float sea_press_inp);
	//void getQ_simple(float* q, float * val);
	//void MotionDetect(float * val);
   
	#if HAS_PRESS()
      //float getEstAltitude();
	  //float getEstAltitude(float * q, float * val, float dt2);
	  //float getBaroAlt();
      //float getBaroAlt(float sea_press);
	  //float getBaroTemperature();
	  //float getBaroPressure();
    #endif

	#if (MARG == 4)
		DCM dcm;
	#endif
	    
    HMC58X3 magn;
	iCompass maghead;	
    MPU60X0 accgyro; 
     
    #if HAS_PRESS()
    	//KalmanFilter kPress; // Altitude Kalman Filter.
      	//AltComp altComp; // Altitude Complementary Filter.
	  	//MS561101BA baro;
	#endif
     
	//Global Variables
	 
    int* raw_acc, raw_gyro, raw_magn;
    // calibration parameters
    int16_t gyro_off_x, gyro_off_y, gyro_off_z;
    int16_t acc_off_x, acc_off_y, acc_off_z, magn_off_x, magn_off_y, magn_off_z;
    float acc_scale_x, acc_scale_y, acc_scale_z, magn_scale_x, magn_scale_y, magn_scale_z;
	float val[12];
	//int8_t nsamples, temp_break, instability_fix, senTemp_break;
	int16_t DTemp, temp_corr_on; 
	float rt, senTemp, gyro_sensitivity;
	float sampleFreq; // half the sample period expressed in seconds
		
	// --------------------------------------------------------------------
	// Define Marg = 3 factors here
	// --------------------------------------------------------------------
	#define gyroMeasError 3.14159265358979 * (.50f / 180.0f) 	// gyroscope measurement error in rad/s (shown as 5 deg/s)
	#define gyroMeasDrift 3.14159265358979 * (0.02f / 180.0f) 	// gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)

	#define beta1 sqrt(3.0f / 4.0f) * gyroMeasError 			// compute beta
	#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift 				// compute zeta
	
	// --------------------------------------------------------------------
	// Define IMU Axis Alignment here
	// --------------------------------------------------------------------	
	#if HAS_AXIS_ALIGNED() //accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz
		int sensor_order[9] = {0,1,2,3,4,5,6,7,8};
		int sensor_sign[9] = {1,1,1,1,1,1,1,1,1};
	#endif 	

	// --------------------------------------------------------------------
	// No further changes below this point
	// --------------------------------------------------------------------
	
  private:
    //void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    //void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    
	bool  bSPI;
	float bx, by, bz;
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;  			// scaled integral error
    volatile float twoKp;      				// 2 * proportional gain (Kp)
    volatile float twoKi;      				// 2 * integral gain (Ki)
    volatile float q0, q1, q2, q3, q3old; 	// quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx,  integralFBy, integralFBz;
    unsigned long lastUpdate, now; 			// sample period expressed in milliseconds
	unsigned long lastUpdate1 = 0;
	unsigned long now1;
	
	//Madgwick AHRS Gradient Descent 
    volatile float beta;				// algorithm gain

	//Following lines defines Madgwicks Grad Descent Algorithm from his original paper
	// Global system variables
	float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; 	// estimated orientation quaternion elements with initial conditions
	float b_x = 1, b_z = 0; 				// reference direction of flux in earth frame
	float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

	#if(MARG == 0)
		void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
  	#elif(MARG == 1)
		//void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
		//void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	#elif(MARG == 3)
		//void MARGUpdateFilter(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
		//void MARGUpdateFilterIMU(float gx, float gy, float gz, float ax, float ay, float az);
	#endif
};

float invSqrt(float number);
void arr3_rad_to_deg(float * arr);
void Qmultiply(float *  q, float *  q1, float * q2);
void gravityCompensateAcc(float * acc, float * q);

#endif // FreeIMU_h

