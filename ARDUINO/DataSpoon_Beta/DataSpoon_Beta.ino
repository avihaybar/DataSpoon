// DataSpoon V0.3 BTLE Communication 20.02.2016
// ORIGINAL FreeIMU Author: Fabio Varesano [R.I.P]
// ORIGINAL nRF8001 libary Author: Kevin Townsend/KTOWN for Adafruit Industries
// MODIFIED by Ronit Slyper (rslyper@umich.edu) Nov 2014
// UPDATED by Avihay Bar (avihay.bar@post.idc.ac.il) 21.02.2015
//

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
#include "DebugUtils.h"

// FreeIMU library
#include "FreeIMU.h"
#define MARG 0                // Choose filter method [see FreeIMU.h line 33]  
FreeIMU my3IMU = FreeIMU();

// BTLE library - Might switch to default BLE_UART later
#include "Adafruit_BLE_UART_noTXdelay.h"
#define BaudRate 115200

// Connect CLK/MISO/MOSI to hardware SPI, On Leonardo: CLK = 15, MISO = 14, MOSI = 16
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 7
#define ADAFRUITBLE_RST 9

// Initiate BTLE Device
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
aci_evt_opcode_t status = laststatus;


float valf[11];               // 9-axis float (calibrated) values from sensors
int16_t val[11];              // 9-axis int (calibrated) values - Accel/Gyro 16-Bit, Mango. 12-Bit 
uint8_t sendbuffer[20];       // Data packet to transfer
int concat_last_millis = 0;   // Used for time stamp
int32_t currentTime;          // Trunced to 24-bit due to BT Bandwidth limitations


void setup() {
  
  Serial.begin(BaudRate);
  Wire.begin();
  
  Serial.println(F("DataSpoon V0.3.2, Updated 21.02.2016"));
  my3IMU.init(true);
  pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);
  delay(500);
  //digitalWrite(13, LOW);
  
  BTLEserial.begin();
  concat_last_millis = millis();  // save current time as baseline
}


void loop() {
  
  btleLoop();
  if (status == ACI_EVT_CONNECTED) {

      currentTime = millis();
      concat_last_millis = currentTime;
    
      my3IMU.getValues(valf); // Get 9-Axis values (as floats)

      // translate Gyro/Accel. to 16Bit int
        val[0] = (int) (valf[0] * acc_scale_x);
        val[1] = (int) (valf[1] * acc_scale_y);
        val[2] = (int) (valf[2] * acc_scale_z);

        val[3] = (int) (valf[3] * 2048);
        val[4] = (int) (valf[4] * 2048);
        val[5] = (int) (valf[5] * 2048);

        val[6] = (int) (valf[6] * magn_scale_x);
        val[7] = (int) (valf[7] * magn_scale_y);
        val[8] = (int) (valf[8] * magn_scale_z);
        
      
      
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
      
      // Debug info (print to serial)
      Serial.print(currentTime); Serial.print(",");
      Serial.print(val[0]); Serial.print(",");Serial.print(val[1]); Serial.print(",");Serial.print(val[2]); Serial.print(",");
      Serial.print(val[3]); Serial.print(",");Serial.print(val[4]); Serial.print(",");Serial.print(val[5]); Serial.print(",");
      Serial.print(val[6]); Serial.print(",");Serial.print(val[7]); Serial.print(",");Serial.print(val[8]);Serial.println();
            
      // Transmit readings
      btleWrite();
      delay(40); 
  }
  
}


//    Constantly checks for new events on the nRF8001
void btleLoop() {
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  
  // Ask what is our current status
  status = BTLEserial.getState();
  
  // Print if status is changed 
  if (status != laststatus) {
    
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Adverting"));
    }
    else if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    else if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Discon or advert timeout"));
    }  
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {

    // waiting for incoming data [never used]
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      //Serial.print(c);
    }
  }
}

void btleWrite() {
  // Send the 20-Byte data packet to client
  BTLEserial.write(sendbuffer, 20);
}
