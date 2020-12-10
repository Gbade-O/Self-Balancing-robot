/* SelfBalancing Robot Firmware/////////
//MPU6050 DMP control was adapted from a post by JeffRowberg . Control system was built around DMP control
https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino */


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

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//Define functions 
bool GyroSetUp();
void dmpDataReady();

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

//Intialize Gyro communication and DMP unit
dmpReady  = GyroSetup();

//Configure LED as output 
pinMode(LED_PIN,OUTPUT)


//Configure Timer2 to control Stepper Motor 
TCNT2 &= 0; 
TIMSK2 |= (1<<OCIE2A); //Enable timer interrupts 
sei(); //Emable  global interrupts 





}

void loop() {
  // put your main code here, to run repeatedly:
  
  // if Gyro calibration failed, don't try to do anything
  if (!dmpReady) return;
  
  if ( mpuinterrupt ) { //New data has been received 
        
      mpuInterrupt = false;
      mpuIntStatus = mpu.getStatus();  //Informs whether there is an overflow condition or usable data 
      
      fifocount = mpu.getFIFOCount(); 
      
      //check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount== 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
        
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
          mpu.getFIFOBytes(fifoBuffer, packetSize);
        
          //Display Euler angles in degrees 
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

          /* Add in PID and Motor Control */
            
            
            
            }
  
  
  }
            
    
    



}


bool GyroSetup(){
  
  
  uint8_t Gyro_status 
  bool dmpstatus = false; 
  //Functions to initialize Gryo and print status for troubleshooting 
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR =24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  //Initialize Serial communication 
  Serial.begin(115200);
  while(!Serial); //Wait until Serial is initialized completely 
  
  //Initialize Gyro
  Serial.println(F("Initializing Gyro device..."));
  mpu.initialize();
  
  //Confirm Gyro is connected 
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("Gyro connection successful") : F("Gyro connection failed"));
  
  //Configure Digital Motion Processor ( DMP) , which is onboard on Gyro
  Serial.println(F("Initializing DMP..."));
  Gyro_status =  mpu.dmpInitialize();
  
  //Calibrate Gyro offsets 
  mpu.setXGyroOffset(-19);//(220);
  mpu.setYGyroOffset(-6);//(76);
  mpu.setZGyroOffset(-46);//(-85);
  mpu.setXAccelOffset(-1717);//(1788); // 1688 factory default for my test chip
  mpu.setYAccelOffset(-1747);
  mpu.setZAccelOffset(1473);
  
  
  if (Gyro_status == 0) {  // Successful initialization will return 0 
  
  //Turn on DMP
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  // Attach digital interrupt to Arduino pin 
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
  attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpstatus  = true;  //
  
  // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  
  
  } else {
    
   // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(Gyro_status);
    Serial.println(F(")")); }
    
  
  return dmpstatus;

}


void dmpDataReady(){
  
  //Digital interrupt triggered , set flag 
  mpuInterrupt = true;
}
  
