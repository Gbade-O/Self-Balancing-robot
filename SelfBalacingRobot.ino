#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Define PIN OUTS 
#define LED_PIN 13
#define Step 3
#define Dir 4
#define M0 5
#define M1 7
#define M2 8

//Define Constants to be used below 
#define Kp 8
#define Ki 0
#define Kd 0
#define Rad2deg 57.2958


//Define functions 
GyroSetUp()

//Define variables 
MPU6050 mpu;


void setup() {
//Define DRV8825 Pins as OUTPUT 
pinMode(Step,OUTPUT);
pinMode(Dir,OUTPUT);
pinMode(M0,OUTPUT);
pinMode(M1,OUTPUT);
pinMode(M2,OUTPUT);

//This chooses a micro-step value on DRV8825
digitalWrite(M0,HIGH);
digitalWrite(M1,LOW);
digitalWrite(M2,HIGH);

//PIN2 will be used as a digital interrup pin 
pinMode(2,INPUT);

//Initialize Serial communication 
Serial.begin(115200);
while(!Serial); //Wait until Serial is initialized completely 

//Initialize Gyro
Serial.println(F("Initializing Gyro device..."));
mpu.initialize();

}

void loop() {
  // put your main code here, to run repeatedly:

}
