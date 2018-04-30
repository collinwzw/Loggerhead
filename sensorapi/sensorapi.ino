#include <SPI.h>
#include <Wire.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <string.h>
/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground

   History
   =======
   2014/JULY/25  - First version (KTOWN)
*/
   
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000


/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/
char tbs[600];
#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  Serial.begin(9600);
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);

}

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


void setup(void) 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  //Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    //Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  //Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  //Serial.println("");
}


void loop(void){
  IMU_DATA();
  char* input;
  while(Serial.available()>0){
     input = Serial.read();
     if (input == 'm'){
     Serial.println('m');
     delay(2000);
     }
     }
}


void IMU_DATA () 
{  
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  // print out accelleration data
  //Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  //Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  //Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");

  // print out magnetometer data
  //Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  //Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
  //Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");
  
  
  // print out gyroscopic data
  //Serial.print(gyro.gyro.x);Serial.print("     ");
  //Serial.print(gyro.gyro.y);Serial.print("     ");
  //Serial.print(gyro.gyro.z);Serial.print("     ");
  //Serial.print(accel.acceleration.x);Serial.print("     ");
  //Serial.print(accel.acceleration.y);Serial.print("     ");
  //Serial.println(accel.acceleration.z);
  sprintf(tbs,"%7d%7d%7d%7d%7d%7d",(int)(gyro.gyro.x*100),(int)(gyro.gyro.y*100),(int)(gyro.gyro.z*100),(int)(accel.acceleration.x*100),(int)(accel.acceleration.y*100),(int)(accel.acceleration.z*100));
  Serial.println(tbs);
  //Serial.print(gyro.gyro.x); Serial.print(" ");
  //Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
  //Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");

  // print out temperature data
  //.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");

  //Serial.println("**********************\n");
 
  delay(1000);
}
