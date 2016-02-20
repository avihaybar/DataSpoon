/**  ..... FreeIMU library ....
 * Example program for using the FreeIMU connected to an Arduino Leonardo.
 * The program reads sensor data from the FreeIMU, computes the yaw, pitch
 * and roll using the FreeIMU library sensor fusion and use them to move the
 * mouse cursor. The mouse is emulated by the Arduino Leonardo using the Mouse
 * library.
 * 
 * @author Fabio Varesano - fvaresano@yahoo.it
 *
 * MODIFIED by Ronit Slyper rslyper@umich.edu Nov 2014
 *
 * UPDATED by Avihay Bar (avihay.bar@post.idc.ac.il) 24DEC2015
 * 
 */
 
/*   ..... Adafruit nRF8001 libary ....
 * Written by Kevin Townsend/KTOWN  for Adafruit Industries.
 * MIT license, check LICENSE for more information
 *
 **/

// Hardware libraries 
#include <HMC58X3.h>          // 3-Axis Digital Compass (Magnometer)
#include <MS561101BA.h>       // 1-Axis Barometer/Altimeter Sensor (+Temperature)
#include <MPU60X0.h>          // 6-Axis Gyro/Accelerometer

// Math libraries
#include <AP_Math_freeimu.h>  //
#include <Filter.h>           //
#include <iCompass.h>         // 

// General Comm libraries
#include <I2Cdev.h>           // I2C Communication
#include <Wire.h>             // Wire Communication 
#include <SPI.h>              // SPI Communication
#include <EEPROM.h>           // EEPROM Memory stores Calibration Data

#include "FreeIMU.h"
#include "DebugUtils.h"
#include "Adafruit_BLE_UART_noTXdelay.h"


// Connect CLK/MISO/MOSI to hardware SPI, On Leonardo: CLK = 15, MISO = 14, MOSI = 16
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 7
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
aci_evt_opcode_t status = laststatus;

FreeIMU my3IMU = FreeIMU();

float valf[11];
int16_t val[11];
uint8_t sendbuffer[20];
int concat_last_millis = 0;
int32_t currentTime; //Limited to 24-bit due to BT Bandwidth limitations


void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println(F("DataSpoon V0.3, Updated 24DEC2015"));
  
  delay(500);
  my3IMU.init();
  my3IMU.zeroGyro();
  BTLEserial.begin();
  concat_last_millis = millis();
}


void loop() {
  
  btleLoop();
  if (status == ACI_EVT_CONNECTED) {

    currentTime = millis();
    concat_last_millis = currentTime;
    
    my3IMU.getValues(valf); // Get 9-Axis values (as floats)

    for (int i = 0; i < 6; i++) { // translate Gyro/Accel. to 16Bit int
      val[i] = (int) (valf[i] * 32768);
    }
    for (int i = 6; i < 9; i++) { // translate Magno. to 12Bit int
      val[i] = (int) (valf[i] * 2048);
    }
      
    // 3 Bytes for TimeStamp
    sendbuffer[0] = (currentTime)       & 0xFF;
    sendbuffer[1] = (currentTime >>  8) & 0xFF;
    sendbuffer[2] = (currentTime >> 16) & 0xFF;
      
    // 2 Bytes for Accel. x
    sendbuffer[3] = (byte)  (val[0] & 0xFF);
    sendbuffer[4] = (byte) ((val[0] >> 8)& 0xFF);

    // 2 Bytes for Accel. y
    sendbuffer[5] = (byte)  (val[1] & 0xFF);
    sendbuffer[6] = (byte) ((val[1] >> 8)& 0xFF);

    // 2 Bytes for Accel. z
    sendbuffer[7] = (byte)  (val[2] & 0xFF);
    sendbuffer[8] = (byte) ((val[2] >> 8)& 0xFF);

    // 2 Bytes for Gyro. x
    sendbuffer[9] = (byte)  (val[3] & 0xFF);
    sendbuffer[10] = (byte) ((val[3] >> 8)& 0xFF);
      
    // 2 Bytes for Gyro. y
    sendbuffer[11] = (byte)  (val[4] & 0xFF);
    sendbuffer[12] = (byte) ((val[4] >> 8)& 0xFF);

    // 2 Bytes for Gyro. z
    sendbuffer[13] = (byte)  (val[5] & 0xFF);
    sendbuffer[14] = (byte) ((val[5] >> 8)& 0xFF);


    // Trying to put 3x12-Bit inside 5 Bits [15-19]

    // 12 Bits for Magno. x
    sendbuffer[15] = (byte)  (val[6] & 0xFF);
    sendbuffer[16] = (byte) ((val[6] >> 8)& 0x0F);  // Saving only 4 remaining bits to sendbuffer[16] lsb;

    // 12 Bits for Magno. y
    sendbuffer[16] = (byte)  ((val[7] & 0x0F) << 4) | sendbuffer[16]; // Saving first 4 bits to sendbuffer[16] msb;
    sendbuffer[17] = (byte)  (val[7] >> 4);
      

    // 12 Bits for Magno. z
    sendbuffer[18] = (byte)  (val[8] & 0xFF);
    sendbuffer[19] = (byte) ((val[8] >> 8)& 0xFF);
      
      
    Serial.print(currentTime); Serial.print(" | ");
    Serial.print(val[0]); Serial.print(",");Serial.print(val[1]); Serial.print(",");Serial.print(val[2]); Serial.print(" : ");
    Serial.print(val[3]); Serial.print(",");Serial.print(val[4]); Serial.print(",");Serial.print(val[5]); Serial.print(" : ");
    Serial.print(val[6]); Serial.print(",");Serial.print(val[7]); Serial.print(",");Serial.print(val[8]);Serial.println();
    //    Serial.print(sendbuffer[15]); Serial.print("|");
    //    Serial.print(sendbuffer[16]); Serial.print("|");
    //    Serial.print(sendbuffer[17]); Serial.print("|");
    //    Serial.print(sendbuffer[18]); Serial.print("|");
    //    Serial.print(sendbuffer[19]); Serial.print("|");
    //    Serial.println();
    btleWrite();
    delay(500); 
  }
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/

void btleLoop() {
  // Tell the nRF8001 to do whatever it should be working on.
  //Serial.println("before = " + millis());
  BTLEserial.pollACI();
  //Serial.println("after = " + millis());

  // Ask what is our current status
  status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Adverting"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Discon or advert timeout"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    //if (BTLEserial.available()) {
    //  Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    //}
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      //Serial.print(c);
    }
  }
}

void btleWrite() {
  BTLEserial.write(sendbuffer, 20);  // write the data
}


