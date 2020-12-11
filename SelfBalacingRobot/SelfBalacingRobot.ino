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
#define CW 1 
#define CCW 0
#define DRIVE_MIN 240  //This is the longest period , that will cause the driver to spin stepper motor. ( Based on current set-up)
#define DRIVE_MAX 3  //Shortest period that DRV8825 can handle ( Fastest stepper motor speed ) 

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuInterrupt;
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

//PID variables 
unsigned long currtime=0;
 unsigned long prevtime=0;
double elapsedtime;
double PID_error,Error_Sum;
double prevError;
float setPoint;
double rateError;

//Motor Variables 
volatile uint8_t direction; // 0 is clockwise , wh

//Define functions 
bool GyroSetUp();
void dmpDataReady();
void ComputePID_Control(float angle );




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
pinMode(LED_PIN,OUTPUT);


//Configure Timer2 to control Stepper Motor 
TCNT2 &= 0; 
TIMSK2 |= (1<<OCIE2A); //Enable timer interrupts 
sei(); //Emable  global interrupts 





}

void loop() {
  // put your main code here, to run repeatedly:
  
  // if Gyro calibration failed, don't try to do anything
  if (!dmpReady) return;
  
  if ( mpuInterrupt ) { //New data has been received 
        
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();  //Informs whether there is an overflow condition or usable data 
      
      fifoCount = mpu.getFIFOCount(); 
      
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

          ComputePID_Control( ypr[2] * Rad2deg); }
  
  
  }
            
    
    



}


bool GyroSetup(){
  
  
  uint8_t Gyro_status ;
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
  

void ComputePID_Control( float angle){
  
  //Compute PID calculations 
  currtime = millis();
  elapsedtime = (double)(currtime - prevtime);
  
  
  PID_error = setPoint - angle; //Compute instant PID Error , Kp
  
  
  Error_Sum += PID_error * elapsedtime;  //Compute inntegral Ki 
  rateError  = (PID_error - prevError) / elapsedtime; //Compute derivative , Kd 
  
  double PID_Value  = Kp*PID_error + Ki*Error_Sum + Kd*rateError; 
  
  prevError = PID_error;
  prevtime = currtime;
  
  //determine and send Motor commands 
  direction = (PID_Value < 0 ) ? ( CCW ) : ( CW ) ;//Based on orientatiation of fall , determine what direction to move motor 
  
  if( direction ) { //Set Direction pin accordingly , based on direction 
    
    digitalWrite(Dir,HIGH); //Moves motor in CW direction based on set-up. Prepare Pin for motor to move 
    direction = -1; //Forces direction to be determined fresh each time motor will be actuated 
  } else if( direction == 0) {
    digitalWrite(Dir,LOW); //Moves motor in CCW direction based on set-up. Prepare Pin for motor to move 
    direction = -1 ; //Forces direction to be determined fresh each time motor will be actuated 
  }
  
  //Map PID output to value Motor driver understands 
  double abs_PID = abs(PID_Value); //Needed incase there is a negative PID output 
  double Stepper_delay = map(abs_PID,0,360,DRIVE_MIN,DRIVE_MAX);
  
  //Actuate Motor using timer2 only if the Error is greater than +/5 degrees . Else stop Motor 

  if( abs( PID_error ) >=5 )
  {
      TCNT2 = 0; //Start timer from 0 
      OCR2A = Stepper_delay; //Compare match set to stepper period 
      TCCR2A = ( 1<< WGM21); //Set Timer Mode 
      TCCR2B = (1<<CS22)|(1<<CS21); //Set Clock divider and Start clock 
  } else {

      //Stop Motor from spinning using Timer2 
      TCCR2B &= 0;// same for TCCR2B
      TCNT2 = 0;
      
  }

  
}
  
  
  
ISR(TIMER2_COMPA_vect)
{
  TCNT2 = 0;
  digitalWrite(Step,HIGH);
  delayMicroseconds(2);
  digitalWrite(Step,LOW);
}






  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
