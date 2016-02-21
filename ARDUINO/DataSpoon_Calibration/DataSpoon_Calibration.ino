//
// DataSpoon V0.3 Serial Communication & Calibration, 21.02.2016
// Updated by Avihay Bar, avihay.bar@post.idc.ac.il
//

// Hardware libraries 
#include <HMC58X3.h>          // 3-Axis Digital Compass (Magnometer)
#include <MS561101BA.h>       // 1-Axis Barometer/Altimeter Sensor (+Temperature)
#include <MPU60X0.h>          // 6-Axis Gyro/Accelerometer

// Math libraries
#include <AP_Math_freeimu.h>  // Ardupilot math library ported to FreeIMU
#include <Butter.h>           // Butterworth filter
#include <iCompass.h>         // 
#include <DCM.h>              // Direction Cosine Matrix math library

// General Comm libraries & Utilities
#include <I2Cdev.h>           // I2C Communication
#include <Wire.h>             // Wire Communication 
#include <SPI.h>              // SPI Communication
#include <EEPROM.h>           // EEPROM Memory stores Calibration Data
#include "DebugUtils.h"
#include "CommunicationUtils.h"

// FreeIMU library
#include "FreeIMU.h"
FreeIMU my3IMU = FreeIMU();
#define MARG 0                // Choose filter method [see FreeIMU.h line 33]

#define BaudRate 115200


// Global varaibles 
float q[4];                   // Quaternion array
int   raw_values[11];         // 16/12Bit integers directly from the sensors
float ypr[3];                 // Yaw Pitch Roll - Eueler angles
char  str[128];
float val[12];                // Calibrated float measurments
float val_array[18];          // All-Data array, including 9-Axis + Pressure + Altitude + Temperature + 
char cmd, tempCorr;           // Commands arguments from the serial connection


void setup() {
  
  Serial.begin(BaudRate);
  Wire.begin();
  
  my3IMU.init(true);
  delay(500);
      
  // LED
  pinMode(13, OUTPUT);
}

void loop() {
  
  if(Serial.available()) {
    
    cmd = Serial.read();      
    //check the incoming command from serial - v / 1 / 2 / g / r / b / q / z / c / C / x / d
    // v = check version
    // 1 = reset device
    // 2 = reset quaternion matrix
    // g = reset gyro
    // r = get 11-axis RAW (uncalibrated) integer values from sensors (3 Accel., 3 Gyro., 3 Magno., 1 Temperature, 1 Pressure)
    // b = used for Calibration: get 9-axis RAW (uncalibrated) values from sensors (3 Accel., 3 Gyro., 3 Magno.)
    // q = get the quaternion matrix from board
    // z = get full 18 fields of data (4 Quaternion, 3 Accel., 3 Gyro., 3 Magno., 2 Temperature, 1 Pressure, 1 Altitude, 1 Freq)
    // a = same as z, but Kalman filtered
    // c = write calibration data to EEPROM 
    // C = readback calbration data from EEPROM
    // x = clear calibration data on EEPROM
    // d = debug mode
    //
    // Command used in calibration - v / b / c / C / x
    // Command used in Processing Cube - v / z / 1 / 2 / g / 
    // Commands CANCELED = t / f / p / a
    
    
    if(cmd=='v') {
      Serial.print("DataSpoon alpha 0.3\n");
    }
    else if(cmd=='1'){
      my3IMU.init(true);
    }
    else if(cmd=='2'){
      my3IMU.RESET_Q();           
    }
    else if(cmd=='g'){
      my3IMU.initGyros();
    }
    else if(cmd=='r') {
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
        my3IMU.getRawValues(raw_values);
        sprintf(str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", 
                      raw_values[0], raw_values[1], raw_values[2], 
                      raw_values[3], raw_values[4], raw_values[5], 
                      raw_values[6], raw_values[7], raw_values[8], 
                      raw_values[9]);
        Serial.print(str);
        #if HAS_PRESS()
          Serial.print(my3IMU.getBaroTemperature()); Serial.print(",");
          Serial.print(my3IMU.getBaroPressure()); Serial.print(",");
        #else
          Serial.print("0,0,");
        #endif
        Serial.print(millis()); Serial.print(",");Serial.println("\r\n");
      }
    }
    else if(cmd=='b') {
      uint8_t count = serial_busy_wait();
      for(uint8_t i = 0; i < count; i++) {
        my3IMU.getRawValues(raw_values);
        writeArr(raw_values, 6, sizeof(int)); 
        my3IMU.magn.getValues(&raw_values[0], &raw_values[1], &raw_values[2]);
        writeArr(raw_values, 3, sizeof(int));
        Serial.println();
      }
    }
    else if(cmd == 'q') {
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
        my3IMU.getQ(q, val);
        serialPrintFloatArr(q, 4);
        Serial.println("");
      }
    }
    else if(cmd == 'z') {
      float val_array[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
        my3IMU.getQ(q, val);

        // Quaternion data
	      val_array[0] = (q[0]);
        val_array[1] = (q[1]);
        val_array[2] = (q[2]);
        val_array[3] = (q[3]);
        // Accel.
        val_array[4] = (val[0]);
        val_array[5] = (val[1]);
        val_array[6] = (val[2]);
        // Gyro.
        val_array[7] = (val[3] * M_PI/180);
        val_array[8] = (val[4] * M_PI/180);
        val_array[9] = (val[5] * M_PI/180);
        // Magno.
        val_array[10] = (val[6]);
        val_array[11] = (val[7]);
        val_array[12] = (val[8]);
        
        #if HAS_PRESS()
          // Temperature, Pressure, Freq, Height(?)
          val_array[13] = (my3IMU.getBaroTemperature());
          val_array[14] = (my3IMU.getBaroPressure());
          val_array[15] = my3IMU.sampleFreq;
          val_array[16] = val[9];
          val_array[17] = val[10];
        #else 
          val_array[13] = (my3IMU.DTemp/340.) + 35.;
          val_array[15] = my3IMU.sampleFreq;
          val_array[16] = val[9];
        #endif

        serialPrintFloatArr(val_array,18);
        Serial.print('\n');
                   
      }
    } 
    #ifndef CALIBRATION_H
      if(cmd == 'c') {
        const uint8_t eepromsize = sizeof(float) * 6 + sizeof(int) * 6;
        while(Serial.available() < eepromsize) ; // wait until all calibration data are received
          EEPROM.write(FREEIMU_EEPROM_BASE, FREEIMU_EEPROM_SIGNATURE);
        for(uint8_t i = 1; i<(eepromsize + 1); i++) {
          EEPROM.write(FREEIMU_EEPROM_BASE + i, (char) Serial.read());
        }
        my3IMU.calLoad(); // reload calibration
        digitalWrite(13, HIGH);
        delay(1000);
        digitalWrite(13, LOW);
      }
      else if(cmd == 'x') {
        EEPROM.write(FREEIMU_EEPROM_BASE, 0); // reset signature
        my3IMU.calLoad(); // reload calibration
      }
    #endif
    else if(cmd == 'C') { // check calibration values
      Serial.print("acc offset: ");
      Serial.print(my3IMU.acc_off_x);
      Serial.print(",");
      Serial.print(my3IMU.acc_off_y);
      Serial.print(",");
      Serial.print(my3IMU.acc_off_z);
      Serial.print("\n");
      
      Serial.print("magn offset: ");
      Serial.print(my3IMU.magn_off_x);
      Serial.print(",");
      Serial.print(my3IMU.magn_off_y);
      Serial.print(",");
      Serial.print(my3IMU.magn_off_z);
      Serial.print("\n");
      
      Serial.print("acc scale: ");
      Serial.print(my3IMU.acc_scale_x);
      Serial.print(",");
      Serial.print(my3IMU.acc_scale_y);
      Serial.print(",");
      Serial.print(my3IMU.acc_scale_z);
      Serial.print("\n");
      
      Serial.print("magn scale: ");
      Serial.print(my3IMU.magn_scale_x);
      Serial.print(",");
      Serial.print(my3IMU.magn_scale_y);
      Serial.print(",");
      Serial.print(my3IMU.magn_scale_z);
      Serial.print("\n");
    }
    else if(cmd == 'd') { // debugging outputs
      while(1) {
        my3IMU.getRawValues(raw_values);
        sprintf(str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8], raw_values[9], raw_values[10]);
        Serial.print(str);
        Serial.print('\n');
        my3IMU.getQ(q, val);
        serialPrintFloatArr(q, 4);
        Serial.println("");
        my3IMU.getYawPitchRoll(ypr);
        Serial.print("Yaw: ");
        Serial.print(ypr[0]);
        Serial.print(" Pitch: ");
        Serial.print(ypr[1]);
        Serial.print(" Roll: ");
        Serial.print(ypr[2]);
        Serial.println("");
      }
    }
  }
}

char serial_busy_wait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}

const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 511;

void eeprom_serial_dump_column() {
  // counter
  int i;

  // byte read from eeprom
  byte b;

  // buffer used by sprintf
  char buf[10];

  for (i = EEPROM_MIN_ADDR; i <= EEPROM_MAX_ADDR; i++) {
    b = EEPROM.read(i);
    sprintf(buf, "%03X: %02X", i, b);
    Serial.println(buf);
  }
}
