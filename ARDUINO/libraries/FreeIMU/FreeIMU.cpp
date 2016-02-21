/*
FreeIMU.cpp - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011-2012 Fabio Varesano <fabio at varesano dot net>

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
-----------------------------------------------------------------------------------------------
Below changes were made by Michael J Smorto, above license still applies. 
All changes to this program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

10-27-2013   1. Modified incorporate temperature correction for gyro and accelerometer
			 2. Corrected __AVR__ to __AVR_ATmega128__
             3. Added call to get temperature from MPU-6050 in getRawValues
10-29-2013	Corrected temp coefficient array for temp
11-01-2013  Added new inverse sqrt code obtained from articles:
			http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/ and
			http://www.diydrones.com/forum/topics/madgwick-imu-ahrs-and-fast-inverse-square-root
11-05-13	1. Add code to reset MPU-6050 on start-up, implemented in the Arduino sketch as well as
			option to manually reset in processing sketch.
			2. Added LPF code line, currently set to MPU60X0_DLPF_BW_42
11-23-13	1. Changed twoKidef from 0.1 to 0.0f -
				12-12 => twoKiDef changed from 0f to 0.0025f based on article in DIYDrones
			2. Added bypass caps (0.1 and 0.01 ceramic caps on the power pins of the freeIMU
			2a. Added a 10 uF tantalum on the power bus
			3. Yaw drift stabilized very nicely
			4. Updated processing sketch to calculate heading and display based on the freescale 
			   application note and as implemented:
			   http://diydrones.com/profiles/blogs/tiltcompensated-heading
            5. update RESET function to zero out initial values
12-06-13	1. added LSM303D Polulo board instead of  calculating heading from FreeIMU
			2. added option to turn on/off temp correction, temp_corr_on
			3. added variable to set temperature break where calibration turns off, temp_break,
			note: temp_break in units directly from MPU-6050
01-02-14 to
  01-05-14  1. Recalibrated 6050 using Madgewick automated calibration rig - see paper
			2. Corrected temperature calibration coefficients to ensure that gyros are zeroed for whole
			   temp range when at rest - deleted necessary to run zerogyro routine.
			3. Acceleration coefficients adjusted so Ax = 0, Ay = 0 and Az = 1g when at rest
			4. Had to tune Ki to reduce drift - now set to 0.165
			5. Changed Kp to 1.0 - drift almost zero
01-08-14	1. Completed coding ZARA ZUPT for Zero velocity detemination - added code to the
			   processing sketch
01-09-14	1. Added a Reset Quaternion matrix only - option 2 in the sketch - to allow reset of matrix
			   after rapid movement.
11-02-2013  Todo list:
			2. add Yaw Drift correction - noticed that if the temp is above the temp cut-off
			   can get what I would consider unacceptable drift. Will also check temp calibration
			   above cut-off (high temp range)
			   
02-27-14	Fixed code issues with generic MPU6050 output for raw values.
03-02-14	1. Default to temp correction off.
			2. Updating code as temp correction only applicable to MPU-6050 or 9150 at this point.
			3. Clarified GenMPU6050 and DFROBOT defines
			4. Added code for BMP085 altimeter
03-04-14	Fixed issue with increased drift when magnetometer enabled by updating Fabio's code to the Madgwick
			code posted on his X-IO website.
03-09-14	1. With the magnetometer active Kp is better at 0.5 instead of 0.75. Will adjust as appropriate.
03-15-14	Tested code with magnetometer enabled for both Freeimu and DFROBOT. For the DFRobot which had the
			ADXL345 and ITG3200 Kp and Ki had to be adjusted to 0.15 and 0.000002 respectfully. This stabilized
			the drift but still more than the Freeimu. Code added to auto-adjust Kp and Ki depending on the board.
03-16-14	Added LPF to accelerometer and gyro
03-22-14	1. Added ArduPilot Gyro calibration code and modified AP_Math library calling the modifications
			AP_Math_freeimu to identify that changes were made
			2. Added initGyros option "g" to zero gyros from arduino code and processing sketches
			3. Changes resulted in allowing me to put back Kp and Ki to 0.75 and 0.1625 respectfully.
03-24-14	1. Added ArduPilot Filter library so I could implement a median filter on both the accelerometer
			and the gyroscope when using a ADXL345 and ITG3200. Using a 10 point median filter.
			2. Retuned Kp and Ki to 0.05 and 0.000002f, still have a bit of yaw drift. 
			3. Added a LPF filter per guidance at http://philstech.blogspot.com/2012/04/quadcopter-accelerometer-data-filtering.html.
03-25-14	Further tweaked filter factors and Kp/Ki for DFROBOT
				Median Filter = 7 point
				LPF factor = 0.6
03-31-14	Deleted all filtering and adjusted filters for DFROBOT and GENERAL MPU06050 (no mag) - had.
			little impact. Can not get the ADXL345/ITG3200 combo to stabilize as much as the MPU-6050.
04-03-14	Fixed cal curves - don't need gyro temp cal for ITG3200 since internal cal is used - seems to
			to work fine. Need to add back in filter for accelerometer when using ITG3200. 
04-09-14	1. Deleted Fabio's implementation of the AHRS filter and inserted the complete version from
			Seb Madgwick's X-io website. Seemed to fix the mag issue dramatically.
			2. Put median filter back in using a 5 point filter instead of 7
			3. Fixed zero gyro issue when using the ADXL345 chip
04-18-14
			Added 9150 library - 6050 library with getMotion9 function fully operational
			Fixed heading function so heading is aligned with the IMU axis alignment
			Put median filter on mag values
04-19-14
			Added support for GPS in Arduino and Processing Sketch using TinyGPS++ and AltSerialSoft libraries.
			TinyGPS++ : http://arduiniana.org/libraries/tinygpsplus/
			AltSoftwareSerial : https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
04-21-14	1. Added tilt compensated compass just using the HMC5883L or the 9150 on chip magnetomer
			2. Added GPS support using the TinyGPS++ and the AltSoftwareSerial libraries - modified processing
			GUI to support GPS.
04-23-14	Incorporated Butterworth filter from AP_Filter
04-28-14	Modified LS303D Pololu library for use with any magnetometer and accelerometer.
05-13-14
-------- Deleted 9150 library and added support for MPU-9150/9250 using Jeff Rowbergs AK8975 library for the magnetometer and maintained integrity of original 60X0 library
-------- Replaced Median Filter with Butterworh filters
-------- Added Filter library from ArduIMU (nice addition)
-------- Reduced size of serial sketch to fit on arduino uno for calibration
-------- Added azimuth code for heading when magnetometer and accelerometer available by extracting the code segment from the Pololu LSM303D library. Made a library called iCompass so it can be used elsewhere.
-------- In process of modifying code for use with Pololu AltIMU-10 v3.
---------------------------------------------------------------------------
05-17-14
-------- updated FreeIMU examples to work with current library
-------- Added support for the Pololu AltIMU-10 v3 IMU
--------------------------------------------------------------------------
05-24-14
-------- Corrected test for AltIMU-10 v3 and updated Kp/Ki
-------- Created a new Serial Example for when using a Arduino Due - AltSoftSerial does not work for Due.
-------- Minor correction to Processing sketch
--------------------------------------------------------------------------
06-20-14 
-------- Based on Kris Winer's work on the MPU-9250 and the AK8963 I was able to create 
-------- a basic AK8963 library modeled after that of the AK8975 library from Jeff Rowberg. 
-------- This was incorporated into FreeIMU library.
-------------------------------------------------------------------------
08-29-14
-------- incorporated changes proposed by Dan Barzilay which incorporates the Altitude Complimentary 
-------- filter and the Heading calcs that was in the Procressing sketch directly into the library.
-------- Arduino serial sketches were updated to output the heading and altimeter accordingly. As a 
-------- result of the changes the Processing sketch was simplified and the changes incorporated into 
-------- the FreeIMU_cube_Odo_Exp_v2 Gui in the experimental folder. 
------------------------------------------------------------------------
09-03-14
-------- Initial release:  Implemented the MadgwickAHRS (MARG) filter.  Option available to use either the Mahoney
-------- AHRS or the MARG from FreeIMU.h.  
------------------------------------------------------------------------
09-19-14
-------- Set up altitude to accept new sea level pressure from processing as option pxxxxxx, where xxxxxx
-------- is an the SL pressure in millibars * 100, so 1015.45 would read 101545
-------- Changed tilt compensation routine: deleting iCompass
------------------------------------------------------------------------
09-20-14
-------- Deleted iCompass and RunningAverage from FreeIMU. Compass averaging is done in the Processing 
-------- sketch using Yarmartino compass averaging method which is more address issue of discontinuity 
-------- at 0/360 degrees. 
-------- Tests on the Altimu10 v3 are much more stable. The processing code was adapted from 
-------- Christopher Baker's code at  https://github.com/SAIC-ATS/Algorithms.git. Will leave 
-------- runningaveage library in case anyone wishes to use it for other reasons.
-------- iCompass deleted as tilt compensation was not working properly. Tilt compensation is still 
-------- done from within FreeIMU.cpp using a different method.
-------- FreeIMU_serial.ino updated to reflect changes to implementation of altitude complimentary 
-------- filter from within the library as opposed to calling from the serial sketch.
------------------------------------------------------------------------
09-22-14
-------- Changed LSM303 library default sampling rate to 100hz for accel and magnetometer to adjust 
-------- for use with Teensy 3.1
-------- Updated (int) to (int16_t) in lps331 library for getTemperatureRaw.
-------- FreeIMU examples updated accordingly based on compass averaging change.
------------------------------------------------------------------------
10-03-14 
-------- Updated FreeIMU.cpp to allow easier access to change accelerometer/gyro/magneter default
-------- range settings to match description in wiki page
------------------------------------------------------------------------
10-12-14 
-------- Updated library for support to APM 2.5 (Arduimu).  This change should also work on the 
-------- APM 2.6 board as well.  MS5611 SPI library added from Ardupilot library (saved alot of 
-------- time - thanks guys).  Put back iCompass module for aligned axis.  Had typo - now fixed 
-------- so put it back in.
--------
-------- This SPI code for the MS5611 is based on the example sketch by JP B, "APM 2.6 and 
-------- MPU6000 gyro - any custom firmware out there for this?" found at
-------- https://groups.google.com/forum/#!topic/diyrovers/thIohPCNyZA. Also see 
--------  http://forum.arduino.cc/index.php?PHPSESSID=6mia4mslt5dalc1fi4ad8rl2g1&topic=125623.0
------------------------------------------------------------------------
10-19-14
-------- Several updates/corrections made to library: Fixed APM 2.5 axis alignment, deleted a 
-------- 3g library that was not there, more tweaks to beta terms, uploaded missing TinyGPS++ 
-------- library and moved Baudrate setting to beginning of sketches.
--------
-------- Incorporated MPL3115A2 pressure sensor support by Mario Cannistra 
-------- (https://github.com/mariocannistra) as well as an example using a MPU-9250 
-------- using the MPL3115A2.
------------------------------------------------------------------------
10-24-14
-------- Implemented a third version of the Madgwick filter (Gradient Descent). This version 
-------- is directly from his original paper.  Beta and Zeta have been left at their default 
-------- values in the paper.
------------------------------------------------------------------------
10-25-14
-------- Fixed several bugs that would cause the new AHRS filter to crash on compile.
-------- Jitting is still there and its a lot of fun watching the algorithm work with the 
-------- default settings for the MPU9250.  Have fun tweaking the settings.
------------------------------------------------------------------------
11-04-14
-------- Minor update. Moved motion detect algorithm to FreeIMU library proper.  Created 
-------- FreeIMU_cube_Odo_Exp_v3.pde to take motion detect from library.
------------------------------------------------------------------------
11-05-14
-------- Thanks goes to duguyiqiu for pointing out that the Arduino Due Freq was not printing 
-------- correctly. As a result I found that the legacy code did not address the I2C bus speed 
-------- for the Due or the Mega 2560 to 400hz.  This is fixed with this update.
------------------------------------------------------------------------
11-08-14
-------- Fixed getQ_simple (q's not correctly identified), added a motion transition test and
-------- and called getQ_simple to get convergence on yaw faster (used heading)
------------------------------------------------------------------------
11-12-14
-------- Incorporated central definition of sensor order & signs for MARG calls in FreeIMU.hPa
-------- getQ_simple now works for all 9dof sensors whether sensors are aligned or not.
------------------------------------------------------------------------
11-13-14
-------- Setup a gyro_sensitivity variable defined at the same time you set up your gryo
-------- Made a minor change to the MPU60X0.cpp library for MPU6000 SPI
------------------------------------------------------------------------
11-19-14
-------- Combined running average filter and simple moving average filter into the AP_Filter library.
-------- Added a capability to the Arduino serial sketch to load a permanent GPS config file from a 
-------- attached I2C eeprom.  For my setup default baud is 57600.
------------------------------------------------------------------------
11-28-14
-------- Have incorporated the XBee-Arduino library by Andrew Rapp into an example sketch 
-------- (FreeIMU_serial_ARM_CPU_XBEE.ino).  Did make a modification to it so that it would send packets 
-------- without escapes.  So please use the one in the library folder otherwise it will not work.  
-------- Incorporated the Digi Xbee Java Library into the processing GUI (FreeIMU_cube_Odo_Exp_Digi2) in 
-------- the Experimental folder.  This code has only been tested with the Mega with XBee 900HPs.
-------- NOTE: To use the Xbee-Arduino with its examples skecthes you will need to edit the Xbee.h file 
-------- and set ATAP=2 (Escpaes sent), currently set to ATAP=1 (No Escapes).
------------------------------------------------------------------------
01-01-15
-------- Merged pull request by Mauro Mombelli who fixed AHRS.h for readability and merged ARHSupdateIMU 
-------- with AHRSupdate.  Updated FreeIMU.h accordingly.
------------------------------------------------------------------------
01-09-15
-------- Updated xbee-arduino library to add new function: setAPImode(MODE). Where MODE would be set equal 
-------- to 1 or 2 to correspond to the API mode that you selected for you Xbees.  Defaults to API Mode 2.
-------------------------------------------------------------------------
03-25-15 Incorporated DCM option based on the 9 Degree of Measurement Attitude and Heading Reference System 
-------- for Sparkfun 9DOF Razor IMU (SEN-10125 and SEN-10736) and SparkFun 9DOF Sensor Stick (SEN-10183, 
-------- SEN-10321 and SEN-10724), https://github.com/ptrbrtz/razor-9dof-ahrs.git.  The DCM, MATH and 
-------- Razor_AHRS.ino files were combined into a DCM library for use with the FreeIMU library.
-------- DCM implementation was based on the original code (http://code.google.com/p/sf9domahrs/) 
-------- by Doug Weibel and Jose Julio, based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio
-------- Additional functions added to get Euler angles directly from the DCM class as well as a function
-------- to convert the DCM matrix to quaternions which is used in the example sketch to interface with
-------- with existing Processing sketches.
-------- An example file has been created to show the DCM implementation with euler angle output.
-------- Use existing Serial sketches for use with processing. No other changes are needed.
--------------------------------------------------------------------------
*/

#include "Arduino.h"
#include <inttypes.h>

//#define DEBUG
#include "FreeIMU.h"
#include "DebugUtils.h"
#include <Filter.h>
#include <Butter.h>

#if (MARG == 0)
	#include "AHRS.h"
#elif (MARG == 1)
	#include "MadgwickAHRS.h"
#elif (MARG == 3)
	#include "MARGUpdateFilter.h"
#else
	#include "DCM.h"
#endif

//#include "vector_math.h"

//initialize temperature calibrations, mjs, 10/24/13 - udapted 4-7-14
//comment out ifs and leave float 0's only if you don't temp cal boards
/*#if defined(FREEIMU_v04)
	//calibration for MPU-6050 at default scales of freeIMU
	//need to change if you use different MPU-6050 or change scales.
	//Mag cal not used - TBD
	//MPU-6050 Cal
	float c3[9] = {           0.,            0., -1.618180e-09,            0.,          0.,          0.,     0., 0.,  0.};
	float c2[9] = {4.798083e-07 ,-7.104300e-08 , -1.899410e-05, -4.387634e-08, -1.779335e-08,  4.216745e-09, 0., 0., 0. };
	float c1[9] = {1.801522e-02 ,-5.200081e-03 , -1.462879e-01, -5.878346e-04,  1.172002e-03, -6.897733e-05, 0., 0., 0. };
	float c0[9] = {      -45.61 ,	     -45.24,       -305.58,  6.699801e+00,  8.341212e+00,	-2.171155e+01, 0., 0., 0. };
#else
	float c3[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
	float c2[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
	float c1[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
	float c0[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
#endif */

enum Mscale {
  MFS_14BITS = 0, 	// 0.6 mG per LSB
  MFS_16BITS = 1    // 0.15 mG per LSB
};

//Setup accelerometer filter
butter10hz0_3 mfilter_accx;
butter10hz0_3 mfilter_accy;
butter10hz0_3 mfilter_accz;

//Setup Motion Detect Averages
//MovingAvarageFilter accnorm_avg(5);
//MovingAvarageFilter accnorm_test_avg(7);
//MovingAvarageFilter accnorm_var(7);	
//MovingAvarageFilter motion_detect_ma(7);

//Set-up constants for gyro calibration
uint8_t num_gyros = 1;
uint8_t INS_MAX_INSTANCES = 2;

FreeIMU::FreeIMU() {

	magn = HMC58X3();
	maghead = iCompass(MAG_DEC, WINDOW_SIZE, 500);
    accgyro = MPU60X0(); // I2C

	#if HAS_PRESS()
    	//kPress.KalmanInit(0.0000005,0.01,1.0,0);
        baro = MS561101BA();
  	#endif
  
  	// initialize quaternion
  	q0 = 1.0f;
  	q1 = 0.0f;
  	q2 = 0.0f;
  	q3 = 0.0f;
  	q3old = 0.0f;
  	exInt = 0.0;
  	eyInt = 0.0;
  	ezInt = 0.0;
  	twoKp = twoKpDef;
  	twoKi = twoKiDef;
  	beta = betaDef;
  	integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  	lastUpdate = 0;
  	now = 0;

  	temp_corr_on = temp_corr_on_default;
  	  
  	//initialize gyro offsets
  	gyro_off_x = 0.;
  	gyro_off_y = 0.;  
  	gyro_off_z = 0.;
  
  	#ifndef CALIBRATION_H
  		// initialize scale factors to neutral values
		acc_scale_x = 1;
		acc_scale_y = 1;
		acc_scale_z = 1;
		magn_scale_x = 1;
		magn_scale_y = 1;
		magn_scale_z = 1;
  	#else
		// get values from global variables of same name defined in calibration.h
		acc_off_x = ::acc_off_x;
		acc_off_y = ::acc_off_y;
		acc_off_z = ::acc_off_z;
		acc_scale_x = ::acc_scale_x;
		acc_scale_y = ::acc_scale_y;
		acc_scale_z = ::acc_scale_z;
		magn_off_x = ::magn_off_x;
		magn_off_y = ::magn_off_y;
		magn_off_z = ::magn_off_z;
		magn_scale_x = ::magn_scale_x;
		magn_scale_y = ::magn_scale_y;
		magn_scale_z = ::magn_scale_z;
  	#endif
}

void FreeIMU::init() {
  	init(FIMU_ACCGYRO_ADDR, false);
}

void FreeIMU::init(bool fastmode) {
  	init(FIMU_ACCGYRO_ADDR, fastmode);
}

void FreeIMU::RESET() {
	accgyro.reset();
	delay(50);
	
	//reset matrix
	RESET_Q();   
}

void FreeIMU::RESET_Q() {
	//reset matrix
	q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0;
    eyInt = 0.0;
    ezInt = 0.0;
    twoKp = twoKpDef;
    twoKi = twoKiDef;
    integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
}



//Initialize the FreeIMU I2C bus, sensors and performs gyro offsets calibration
void FreeIMU::init(int accgyro_addr, bool fastmode) {
	delay(5);
  
  	// disable internal pullups of the ATMEGA which Wire enable by default
  	#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    	// deactivate internal pull-ups for twi
    	// as per note from atmega8 manual pg167
    	cbi(PORTC, 4);
    	cbi(PORTC, 5);
  	#elif defined(__AVR_ATmega128__)
    	// deactivate internal pull-ups for twi
    	// as per note from atmega128 manual pg204
    	cbi(PORTD, 0);
    	cbi(PORTD, 1);
  	#endif

  	#if defined(__AVR_ATmega128) || defined( __AVR_ATmega2560__ ) // only valid on AVR, not on 32bit platforms (eg: Arduino 2, Teensy 3.0)
    	if(fastmode) { // switch to 400KHz I2C - eheheh
      		TWBR = ((F_CPU / 400000L) - 16) / 2; // see twi_init in Wire/utility/twi.c
    	}
    
    #endif

 	
 	/* 
 	* For Invensense MPUs:
 	*
 	* FS_SEL | Full Scale Range   | LSB Sensitivity
	 * -------+--------------------+----------------
 	* 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 	* 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 	* 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 	* 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 	*/
  
  	accgyro = MPU60X0(false, accgyro_addr);
	accgyro.initialize();
	accgyro.setDLPFMode(MPU60X0_DLPF_BW_20); 
	accgyro.setI2CMasterModeEnabled(0);
	accgyro.setI2CBypassEnabled(1);
	accgyro.setRate(0x13);			//Sets sample rate to 8000/1+7 = 1000Hz
	accgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000);
	accgyro.setFullScaleAccelRange(MPU60X0_ACCEL_FS_2);
	gyro_sensitivity = 16.4f;
	delay(5);
	delay(30);
  
  	// init HMC5843
	magn.init(false); // Don't set mode yet, we'll do that later on.
	// Calibrate HMC using self test, not recommended to change the gain after calibration.
	magn.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
	// Single mode conversion was used in calibration, now set continuous mode
	magn.setMode(0);
	delay(10);
	magn.setDOR(B110);
    
  
  	#if HAS_MS5611()
		baro.init(FIMU_BARO_ADDR);
	#endif
  	
  	// zero gyro
  	//zeroGyro();
  	//if(temp_corr_on == 0) {
  	//digitalWrite(12,HIGH);
  	initGyros(); //}

  	//digitalWrite(12,LOW);
	
  	#ifndef CALIBRATION_H
		// load calibration from eeprom
		calLoad();
  	#endif

  
  	RESET_Q();

  	float values[11];

  	//DCM filter implementation set here so we can intit with calibrated values.  All initializations have to be done first.
  	#if(MARG == 4)
    	dcm = DCM();
    	getValues( values);
    	//values[9] = maghead.iheading(1, 0, 0, values[0], values[1], values[2], values[6], values[7], values[8]);
    	dcm.setSensorVals(values);
    	dcm.DCM_init(Kp_ROLLPITCH, Ki_ROLLPITCH, Kp_YAW, Ki_YAW);
  	#endif

}


#ifndef CALIBRATION_H
	static uint8_t location; // assuming ordered reads
	void eeprom_read_var(uint8_t size, byte * var) {
  		for(uint8_t i = 0; i<size; i++) {
    		var[i] = EEPROM.read(location + i);
  		}
  		location += size;
	}

	void FreeIMU::calLoad() {
  		if(EEPROM.read(FREEIMU_EEPROM_BASE) == FREEIMU_EEPROM_SIGNATURE) { // check if signature is ok so we have good data
    		location = FREEIMU_EEPROM_BASE + 1; // reset location
    
    		eeprom_read_var(sizeof(acc_off_x), (byte *) &acc_off_x);
    		eeprom_read_var(sizeof(acc_off_y), (byte *) &acc_off_y);
    		eeprom_read_var(sizeof(acc_off_z), (byte *) &acc_off_z);
    
	    	eeprom_read_var(sizeof(magn_off_x), (byte *) &magn_off_x);
    		eeprom_read_var(sizeof(magn_off_y), (byte *) &magn_off_y);
    		eeprom_read_var(sizeof(magn_off_z), (byte *) &magn_off_z);
    
	    	eeprom_read_var(sizeof(acc_scale_x), (byte *) &acc_scale_x);
	    	eeprom_read_var(sizeof(acc_scale_y), (byte *) &acc_scale_y);
    		eeprom_read_var(sizeof(acc_scale_z), (byte *) &acc_scale_z);
    
    		eeprom_read_var(sizeof(magn_scale_x), (byte *) &magn_scale_x);
    		eeprom_read_var(sizeof(magn_scale_y), (byte *) &magn_scale_y);
    		eeprom_read_var(sizeof(magn_scale_z), (byte *) &magn_scale_z);
  		}
  		else { // neutral values
    		acc_off_x = 0;
    		acc_off_y = 0;
    		acc_off_z = 0;
 			acc_scale_x = 1;
    		acc_scale_y = 1;
    		acc_scale_z = 1;

    		magn_off_x = 0;
    		magn_off_y = 0;
    		magn_off_z = 0;
		    magn_scale_x = 1;
    		magn_scale_y = 1;
    		magn_scale_z = 1;
  		}
	}

#endif


/**
 * Populates raw_values with the raw_values from the sensors
*/
void FreeIMU::getRawValues(int * raw_values) {
    // Set raw values for Magnetometer, Pressure, Temp to 0 
    // in case you are only using an accelerometer and gyro
	// raw_values[9] will be set to MPU-6050 temp, see zeroGyro to change raw_values dimension

    raw_values[6] = 0;
    raw_values[7] = 0;
    raw_values[8] = 0;
    raw_values[9] = 0;
	
    #ifdef __AVR__
    	accgyro.getMotion6(&raw_values[0], &raw_values[1], &raw_values[2], &raw_values[3], &raw_values[4], &raw_values[5]);  	  
    	rt = accgyro.getTemperature();	  
     	raw_values[9] = rt;
	#else
    	int16_t ax, ay, az, gx, gy, gz, mx, my, mz, rt;
      	accgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	  
      	raw_values[0] = ax;
      	raw_values[1] = ay;
      	raw_values[2] = az;
      	raw_values[3] = gx;
      	raw_values[4] = gy;
      	raw_values[5] = gz;
      	rt = accgyro.getTemperature();	  
      	raw_values[9] = rt; 
    #endif
  
  	magn.getValues(&raw_values[6], &raw_values[7], &raw_values[8]);
}

/**
 * Populates values with calibrated readings from the sensors
*/
void FreeIMU::getValues(float * values) { 

	float values_cal[9] = {0.,0.,0.,0.,0.,0.,0.,0.,0.};
  	uint8_t i;

  	int16_t accgyroval[9];
	accgyro.getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], 
					   &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	
	DTemp = accgyro.getTemperature();

	values_cal[3] = (float) accgyroval[3] - gyro_off_x;
	values_cal[4] = (float) accgyroval[4] - gyro_off_y;
	values_cal[5] = (float) accgyroval[5] - gyro_off_z;
		  
    for (i = 0; i < 6; i++) {
    	if ( i < 3 ) {
        	values_cal[i] = (float) accgyroval[i];
      	}
      	else {
  			values_cal[i] = values_cal[i] / gyro_sensitivity;   //for 6050 etc
	  	}
    }	
  
    #warning Accelerometer calibration active: have you calibrated your device?
  	// remove offsets and scale accelerometer (calibration)
  	values_cal[0] = (values_cal[0] - acc_off_x) / acc_scale_x;
  	values_cal[1] = (values_cal[1] - acc_off_y) / acc_scale_y;
  	values_cal[2] = (values_cal[2] - acc_off_z) / acc_scale_z;
  
    magn.getValues(&values_cal[6]);
  
  	#warning Magnetometer calibration active: have you calibrated your device?
	values_cal[6] = (values_cal[6] - magn_off_x) / magn_scale_x;
	values_cal[7] = (values_cal[7] - magn_off_y) / magn_scale_y;
	values_cal[8] = (values_cal[8] - magn_off_z) / magn_scale_z;	
	  
  	for(int i = 0; i < 9; i++) {
		values[i] = sensor_sign[i] * values_cal[sensor_order[i]];
  	}
}

/**
 * Computes gyro offsets
*/
void FreeIMU::zeroGyro() {
	const int totSamples = nsamples;
  	int raw[11];
  	float values[11]; 
  	float tmpOffsets[] = {0,0,0};
  
  	for (int i = 0; i < totSamples; i++){
		getRawValues(raw);
		tmpOffsets[0] += raw[3];
		tmpOffsets[1] += raw[4];
		tmpOffsets[2] += raw[5]; 
	}
  
  	gyro_off_x = tmpOffsets[0] / totSamples;
  	gyro_off_y = tmpOffsets[1] / totSamples;
  	gyro_off_z = tmpOffsets[2] / totSamples;
  	delay(5);
}


void FreeIMU::initGyros() {
	
	//Code modified from Ardupilot library
	Vector3f 	last_average[INS_MAX_INSTANCES], 
				best_avg[INS_MAX_INSTANCES], 
				gyro_offset[INS_MAX_INSTANCES];

    float best_diff[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];
		
    // remove existing gyro offsets
    gyro_offset[0] = Vector3f(0,0,0);
    best_diff[0] = 0;
    last_average[0].zero();
    converged[0] = false;
    
	// the strategy is to average 50 points over 0.5 seconds, 
	// then do it again and see if the 2nd average is within 
	// a small margin of the first average

    uint8_t num_converged = 0;	
	
    // we try to get a good calibration estimate for up to 10 seconds
    // if the gyros are stable, we should get it in 1 second
	for (int16_t j = 0; j <= 30 && num_converged < 1; j++) {
		
		Vector3f gyro_sum[INS_MAX_INSTANCES], 
				 gyro_avg[INS_MAX_INSTANCES], 
				 gyro_diff[INS_MAX_INSTANCES];
		
		float diff_norm[INS_MAX_INSTANCES];
		zeroGyro();

		gyro_avg[0] = Vector3f(gyro_off_x, gyro_off_y,gyro_off_z) ;
		
		gyro_diff[0] = last_average[0] - gyro_avg[0];
        diff_norm[0] = gyro_diff[0].length();
        		
		if (converged[0]) 
			continue;
        if (j == 0) {
            best_diff[0] = diff_norm[0];
            best_avg[0] = gyro_avg[0];
        } 
        else if (gyro_diff[0].length() < ToRad(0.05f)) {
            // we want the average to be within 0.1 bit, which is 0.04 degrees/s
            last_average[0] = (gyro_avg[0] * 0.5f) + (last_average[0] * 0.5f);
            gyro_offset[0] = last_average[0];            
            converged[0] = true;
            num_converged++;
        } 
        else if (diff_norm[0] < best_diff[0]) {
            best_diff[0] = diff_norm[0];
            best_avg[0] = (gyro_avg[0] * 0.5f) + (last_average[0] * 0.5f);
        }
        
        last_average[0] = gyro_avg[0];
	}

	delay(5);
	
	if (num_converged == num_gyros) {
		gyro_off_x = gyro_offset[0].x;
		gyro_off_y = gyro_offset[0].y;
		gyro_off_z = gyro_offset[0].z;
		return;
	}

	// we've kept the user waiting long enough 
	// use the best pair we found so far
	if (!converged[0]) {
		gyro_offset[0] = best_avg[0];
	}
	
	gyro_off_x = gyro_offset[0].x;
	gyro_off_y = gyro_offset[0].y;
	gyro_off_z = gyro_offset[0].z;
	
}


/**
 * Populates array q with a quaternion representing 
 * the IMU orientation with respect to the Earth
 * 
 * @param q the quaternion to populate
 */
void FreeIMU::getQ(float * q, float * val) {
 
	getValues(val);
	now = micros();
	sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
	lastUpdate = now;

	// Set up call to the appropriate filter using the axes alignment information
	// gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
	#if(MARG == 0 || MARG == 1 || MARG == 3)
		#if IS_9DOM() && not defined(DISABLE_MAGN)
//			#if MARG == 0
				AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
//			#elif MARG == 1
//				MadgwickAHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
//			#else
//				MARGUpdateFilter(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
//			#endif
//		val[9] = maghead.iheading(1, 0, 0, val[0], val[1], val[2], val[6], val[7], val[8]);
		#else
			AHRSupdateIMU(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2]);
			val[9] = -9999.0f;
		#endif

		q[0] = q0;	
		q[1] = q1;
  		q[2] = q2;
  		q[3] = q3;

  	#endif
  
  	#if (MARG == 4 && IS_9DOM() && not defined(DISABLE_MAGN))
	    val[9] = maghead.iheading(1, 0, 0, val[0], val[1], val[2], val[6], val[7], val[8]);
		dcm.setSensorVals(val);
		dcm.G_Dt = 1./ sampleFreq;
 		dcm.calDCM();
 		dcm.getDCM2Q(q);
  	#endif
	
//		MotionDetect( val );
  
}

float def_sea_press = 1013.25;

#if HAS_MS5611() 
	/**
	* Returns an altitude estimate from barometer readings only using sea_press as current sea level pressure
	*/
	float FreeIMU::getBaroAlt(float sea_press) {
		float temp = baro.getTemperature(MS561101BA_OSR_4096);
		float press = baro.getPressure(MS561101BA_OSR_4096);
        float new_press = kPress.measureRSSI(press);
		return ((pow((sea_press / new_press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	}

	float FreeIMU::getBaroTemperature() {
		float temp1 = baro.getTemperature(MS561101BA_OSR_4096);
		return(temp1);
	}

	float FreeIMU::getBaroPressure() {
		float temp2 = baro.getPressure(MS561101BA_OSR_4096);
		return(temp2);
	}

	float FreeIMU::getBaroAlt() {
		return getBaroAlt(def_sea_press);
	}

#endif

/**
 * Returns the Euler angles in radians defined in the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for 
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 * 
 * @param angles three floats array which will be populated by the Euler angles in radians
*/
void FreeIMU::getEulerRad(float * angles) {
	
	float q[4]; // quaternion
	float val[11];
  
	getQ(q, val);
  	
  	angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); 		// psi
  	angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); 											// theta
  	angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); 	// phi
}

/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for 
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 * 
 * @param angles three floats array which will be populated by the Euler angles in degrees
*/
void FreeIMU::getEuler(float * angles) {
	
	getEulerRad(angles);
  	arr3_rad_to_deg(angles);
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in radians
*/
void FreeIMU::getYawPitchRollRad(float * ypr) {

	float q[4]; // quaternion
  	float val[11];
	float gx, gy, gz; // estimated gravity direction
  	getQ(q, val);
  
  	gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  	gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  	gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  	ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  	ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  	ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in degrees between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in degrees
*/
void FreeIMU::getYawPitchRoll(float * ypr) {
  getYawPitchRollRad(ypr);
  arr3_rad_to_deg(ypr);
}

/**
 * Sets thermal calibration on (1) or off (0) for the accelerometer 
 * and gyro calibration from the main sketch

void FreeIMU::setTempCalib(int opt_temp_cal) {
   temp_corr_on = opt_temp_cal;
   if(temp_corr_on == 1) {
		gyro_off_x = 0.0;
		gyro_off_y = 0.0;
		gyro_off_z = 0.0;
	}
	if(temp_corr_on == 0) {
		//digitalWrite(12,HIGH);
		initGyros();
		//digitalWrite(12,LOW);
	}
}
*/

/**
 * Sets sea level pressure
 * 

void FreeIMU::setSeaPress(float sea_press_inp) {

	def_sea_press = sea_press_inp;
}
*/


/**                           END OF FREEIMU                           **/
/************************************************************************/
/**                           HELP FUNCTIONS                           **/


/**
 * Compensates the accelerometer readings in the 3D vector acc expressed in the sensor frame for gravity
 * @param acc the accelerometer readings to compensate for gravity
 * @param q the quaternion orientation of the sensor board with respect to the world
*/
void gravityCompensateAcc(float * acc, float * q) {
	float g[3];
  
  	// get expected direction of gravity in the sensor frame
  	g[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  	g[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
  	g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  
  	// compensate accelerometer readings with the expected direction of gravity
  	acc[0] = acc[0] - g[0];
  	acc[1] = acc[1] - g[1];
  	acc[2] = acc[2] - g[2];
}

/**
 * Sets the Quaternion to be equal to the product of quaternions {@code q1} and {@code q2}.
 * 
 * @param q1 - the first Quaternion
 * @param q2 - the second Quaternion
 */
void Qmultiply(float *  q, float *  q1, float * q2) {
	q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	q[1] = q1[0] * q2[1] + q2[0] * q1[1] + q1[2] * q2[3] - q1[3] * q2[2];
	q[2] = q1[0] * q2[2] + q2[0] * q1[2] - q1[1] * q2[3] + q1[3] * q2[1];
	q[3] = q1[0] * q2[3] + q2[0] * q1[3] + q1[1] * q2[2] - q1[2] * q2[1];
}

/**
 * Converts a 3 elements array arr of angles expressed in radians into degrees
*/
void arr3_rad_to_deg(float * arr) {
	arr[0] *= 180/M_PI;
	arr[1] *= 180/M_PI;
	arr[2] *= 180/M_PI;
}

//int instability_fix = 1;

float FreeIMU::invSqrt(float x) {
        if (instability_fix == 0) {
        	union { 
				float f;
				int32_t i;
				} y;

			y.f = x;
			y.i = 0x5f375a86 - (y.i >> 1);
			y.f = y.f * ( 1.5f - ( x * 0.5f * y.f * y.f ) );
			return y.f;
        
        }
        else if (instability_fix == 1) {
			uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
			float tmp = *(float*)&i;
			return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
        }
        else {
			/* optimal but expensive method: */
			return 1.0f / sqrt(x);
        }
}