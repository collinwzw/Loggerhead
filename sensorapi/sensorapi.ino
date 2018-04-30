#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <string.h>

void IMU_DATA ();
void configureSensor(void);
void displaySensorDetails(void);
void motor_function(int motorid, int targetangle, int speeds,int currentpos[]);
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000


char tbs[600];
#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);


#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11



Servo Motors[6];
Servo myservomiddleL;  // create servo object to control a servo
Servo myservorearR;
Servo myservorearL;
Servo myservofrontL;
Servo myservofrontR;
Servo myservomiddleR;


int pos;    
int n;
int speeds;
int originalpos=100;
int currentpos[6];
char read_data[10];
int targetpos;
int motorid;
int rd;
int ndx = 0;
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

  myservomiddleL.attach(9);  //connect servo to pins
  myservorearR.attach(8);
  myservorearL.attach(7);
  myservofrontL.attach(6);
  myservofrontR.attach(5);
  myservomiddleR.attach(4);
  
  Motors[0] = myservomiddleL;
  Motors[1] = myservorearR;
  Motors[2] = myservorearL;
  Motors[3] = myservofrontL;
  Motors[4] = myservofrontR;
  Motors[5] = myservomiddleR;
  for (int i = 0; i < 6; i++){
  currentpos[i] = originalpos;
  }
}


void loop(void){
  IMU_DATA();
  char* input;
  while(Serial.available()>0){
     input = Serial.read();
     if (input == 'm'){
      //Serial.println("success");
      boolean newData = false;
        if (Serial.available() > 0){
           while(Serial.available() > 0 && newData == false){
            
              rd= Serial.read();
              if (rd != '\n'){
                read_data[ndx] = rd;
               // Serial.print("the char is: ");Serial.println(read_data[ndx]);
                ndx++;
              }
              else{
                newData = true;
              }
            }
        }
      }
        motorid = ((int) read_data[0]) -48;
        //Serial.print("Motor ID is:");Serial.println(motorid);
        targetpos = ((int)read_data[1])*100+(int)read_data[2]*10+(int)read_data[3];
        speeds = ((int)read_data[4])*100+(int)read_data[5]*10+(int)read_data[6];
        motor_function(motorid,targetpos,speeds,currentpos);
        
     
     }
}


void IMU_DATA () 
{  
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  sprintf(tbs,"%7d%7d%7d%7d%7d%7d",(int)(gyro.gyro.x*100),(int)(gyro.gyro.y*100),(int)(gyro.gyro.z*100),(int)(accel.acceleration.x*100),(int)(accel.acceleration.y*100),(int)(accel.acceleration.z*100));
  Serial.println(tbs);

 
  delay(10);
}

void motor_function(int motorid, int targetangle, int speeds,int currentpos[]) {
  n=speeds; // delay time period for movement of every degree
  //Serial.print("in motor function");
  if (currentpos[motorid] < targetangle){
    for (pos = currentpos[motorid]; pos >= targetangle; pos ++) { 
      // in steps of 1 degree
      Motors[motorid].write(pos);              
      delay(n);             
    }
  }
  else{
      for (pos = currentpos[motorid]; pos <= targetangle; pos --) { 
      Motors[motorid].write(pos);              
      delay(n);                      
  }
  }
  delay(1000);
  //Serial.print("end of motor function ");   
}

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
