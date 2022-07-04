#define USE_USBCON  
#include <Arduino.h>
#include <Servo.h>
#include <DueTimer.h>
#include <ros.h>
#include <std_msgs/Int64MultiArray.h> 
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

//Initialize motors
Servo FL; //Front Left Wheel
Servo BL; //Back Left Wheel
Servo FR; //Front Right Wheel 
Servo BR; //Back Right Wheel
Servo Auger; //Auger Motor

//Encoder pin setup
#define ENCODER_FL_PINA 26 
#define ENCODER_FL_PINB 25
#define ENCODER_BL_PINA 28 
#define ENCODER_BL_PINB 27
#define ENCODER_FR_PINA 34 
#define ENCODER_FR_PINB 33
#define ENCODER_BR_PINA 36 
#define ENCODER_BR_PINB 35

//Encoder and wheel specifications via measurements
double TICKS_PER_M = 18767;
#define TICKS_PER_REV 10967 //256*2*2*10.71=10967.04 not used atm
#define RADIUS 0.09525 //7.5" Diameter Wheels in radius meters 

//Timer interupt frequency
//#define dt 0.01 //10ms

// timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0; //may also have to be unsigned 
double changeMillis = 0;
float loopTime = 10; //10ms

//Speed Control PID Initialize 
//Setpoint is target m/s
//Output is PWM for motor
//prevError is difference between setpoint and current speed
double Setpoint_FL = 0, Output_FL = 0, prevError_FL = 0, ErrorSum_FL = 0, Speed_FL = 0;
double Setpoint_BL = 0, Output_BL = 0, prevError_BL = 0, ErrorSum_BL = 0, Speed_BL = 0;
double Setpoint_FR = 0, Output_FR = 0, prevError_FR = 0, ErrorSum_FR = 0, Speed_FR = 0;
double Setpoint_BR = 0, Output_BR = 0, prevError_BR = 0, ErrorSum_BR = 0, Speed_BR = 0;

//Create local variables for PID function, consider to be global
double Error_FL = 0, changeInError_FL = 0, PID_FL = 0;
double Error_BL = 0, changeInError_BL = 0, PID_BL = 0;
double Error_FR = 0, changeInError_FR = 0, PID_FR = 0;
double Error_BR = 0, changeInError_BR = 0, PID_BR = 0;

//PID gains
const double Kp_FL=35, Ki_FL=15, Kd_FL=0.2;
const double Kp_BL=30, Ki_BL=15, Kd_BL=0.2; 
const double Kp_FR=30, Ki_FR=8, Kd_FR=0.2; 
const double Kp_BR=30, Ki_BR=10, Kd_BR=0.2; 

//Initialize encoder tick values to 0 and track change
volatile long FLTicks = 0, BLTicks = 0, FRTicks = 0, BRTicks = 0;
volatile long prevFLTicks = 0, prevBLTicks = 0, prevFRTicks = 0, prevBRTicks = 0;
double changeFLTicks = 0, changeBLTicks = 0, changeFRTicks = 0, changeBRTicks = 0;

//Track change in distance traveled per wheel 
double changeFLMeters = 0, changeBLMeters = 0, changeFRMeters = 0, changeBRMeters = 0;

//Left motors quadrature encoder pulse 
void FLpulse() {
  static int8_t lookup_table[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  static uint8_t enc_val1 = 0;

  enc_val1 = enc_val1 << 2;
  enc_val1 = enc_val1 | ((REG_PIOD_PDSR & 0b00000011));

  FLTicks = FLTicks + lookup_table[enc_val1 & 0b00001111];
}

void BLpulse() {
  static int8_t lookup_table[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  static uint8_t enc_val2 = 0;

  enc_val2 = enc_val2 << 2;
  enc_val2 = enc_val2 | ((REG_PIOD_PDSR & 0b00001100) >> 2);

  BLTicks = BLTicks + lookup_table[enc_val2 & 0b00001111];
}

//Right motor quadrature encoder pulse
void FRpulse() {
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_val3 = 0;

  enc_val3 = enc_val3 << 2;
  enc_val3 = enc_val3 | ((REG_PIOC_PDSR & 0b00000110) >> 1);
  
  FRTicks = FRTicks + lookup_table[enc_val3 & 0b00001111]; 
}

void BRpulse() {
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_val4 = 0;

  enc_val4 = enc_val4 << 2;
  enc_val4 = enc_val4 | ((REG_PIOC_PDSR & 0b00011000) >> 3);
  
  BRTicks = BRTicks + lookup_table[enc_val4 & 0b00001111]; 
}

//Convert request cmd_vel setpoint to PWM
double FLspeedToPWM(double setpoint){  
  return 0.0509*sq(setpoint) + 18.6*(setpoint) + 93.3;
}

double BLspeedToPWM(double setpoint){  
  return -0.0138*sq(setpoint) + 18.4*(setpoint) + 92.6;
}

double FRspeedToPWM(double setpoint){  
  return -0.0371*sq(setpoint) - 18.2*(setpoint) + 92.4;
}

double BRspeedToPWM(double setpoint){  
  return -0.0371*sq(setpoint) - 18.2*(setpoint) + 92.4;
}

void SpeedPID(float dt){
  //Change in encoder ticks since last function call to find distance travelled          
  changeFLTicks = FLTicks - prevFLTicks; //change in ticks
  changeFLMeters = changeFLTicks/TICKS_PER_M; //distance travelled
  
  changeBLTicks = BLTicks - prevBLTicks;
  changeBLMeters = changeBLTicks/TICKS_PER_M;

  changeFRTicks = FRTicks - prevFRTicks;
  changeFRMeters = changeFRTicks/TICKS_PER_M;
  
  changeBRTicks = BRTicks - prevBRTicks; //Can have its own PID as long as RH setpoint is equal
  changeBRMeters = changeBRTicks/TICKS_PER_M;
  
  dt = dt*0.001;
  //Lower resolution means less accurate approximation 
  //changeFLMeters = (2 * PI * RADIUS * changeFLTicks) / TICKS_PER_REV;

  // Calculates speed for left and right wheel in m/s
  Speed_FL = changeFLMeters/dt; 
  Speed_BL = changeBLMeters/dt;
  Speed_FR = changeFRMeters/dt;
  Speed_BR = changeBRMeters/dt;

  // sets old count to new count for next calculation
  prevFLTicks = FLTicks;
  prevBLTicks = BLTicks;
  prevFRTicks = FRTicks;
  prevBRTicks = BRTicks;
  
  //calculating Error terms for PID controller
  Error_FL = Setpoint_FL - Speed_FL;
  Error_BL = Setpoint_BL - Speed_BL;
  Error_FR = Setpoint_FR - Speed_FR;
  Error_BR = Setpoint_BR - Speed_BR;

  changeInError_FL = Error_FL - prevError_FL; 
  changeInError_BL = Error_BL - prevError_BL;
  changeInError_FR = Error_FR - prevError_FR;
  changeInError_BR = Error_BR - prevError_BR;

  prevError_FL = Error_FL;
  prevError_BL = Error_BL;
  prevError_FR = Error_FR;
  prevError_BR = Error_BR;

  ErrorSum_FL += Error_FL;
  ErrorSum_BL += Error_BL;
  ErrorSum_FR += Error_FR;
  ErrorSum_BR += Error_BR;

  //Total PID calculation using set KP, KI, KD values
  PID_FL = (Kp_FL*Error_FL) + (Ki_FL*ErrorSum_FL*dt) + ((Kd_FL*changeInError_FL)/dt);   //dt is your time step or time spent for each cycle
  PID_BL = (Kp_BL*Error_BL) + (Ki_BL*ErrorSum_BL*dt) + ((Kd_BL*changeInError_BL)/dt); 
  PID_FR = (Kp_FR*Error_FR) + (Ki_FR*ErrorSum_FR*dt) + ((Kd_FR*changeInError_FR)/dt); 
  PID_BR = (Kp_BR*Error_BR) + (Ki_BR*ErrorSum_BR*dt) + ((Kd_BR*changeInError_BR)/dt); 

  //send setpoints to be converted to their respective PWM values     
  Output_FL = FLspeedToPWM(Setpoint_FL) + PID_FL;
  Output_BL = BLspeedToPWM(Setpoint_BL) + PID_BL;
  Output_FR = FRspeedToPWM(Setpoint_FR) - PID_FR;
  Output_BR = BRspeedToPWM(Setpoint_BR) - PID_BR;
   
  //constrain outputs to not exceed max/min PWM values   
  Output_FL = constrain(Output_FL, 0, 180);  
  Output_BL = constrain(Output_BL, 0, 180);  
  Output_FR = constrain(Output_FR, 0, 180);  
  Output_BR = constrain(Output_BR, 0, 180);   
   
  //Sends speed to motor control to be outputed to the motors
  FL.write(Output_FL); 
  BL.write(Output_BL);
  FR.write(Output_FR);
  BR.write(Output_BR);
} 

//Set up hardware interrupts for channel A and B of drive motors
void attachInterrupts() {           
  attachInterrupt(digitalPinToInterrupt(ENCODER_FL_PINA), FLpulse, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_FL_PINB), FLpulse, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(ENCODER_BL_PINA), BLpulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BL_PINB), BLpulse, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(ENCODER_FR_PINA), FRpulse, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_FR_PINB), FRpulse, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(ENCODER_BR_PINA), BRpulse, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_BR_PINB), BRpulse, CHANGE); 

  //Better to compute within loop 
  //Timer1.attachInterrupt(SpeedPID);
  //Timer1.start(10000); // 0.01s (10ms) frequency timer interrupt. convert sec > microsecond dt*1000000
}

//ROS Parameters
ros::NodeHandle nh;

//ROS msg and pub topic declaration 
std_msgs::Int64MultiArray enc_ticks;
ros::Publisher enc_ticks_pub("encoder_ticks", &enc_ticks);

std_msgs::Float64MultiArray vel_wheels;
ros::Publisher vel_pub("velocity_wheels", &vel_wheels);

//diff drive controller callback
void velCallback(const std_msgs::Float32MultiArray& msg){
  float left_speed = msg.data[0]; //check units, needs m/s 
  float right_speed = msg.data[1];
  Setpoint_FL = left_speed;
  Setpoint_BL = left_speed;
  Setpoint_FR = right_speed;
  Setpoint_BR = right_speed;

  //we could constrain the speed here, maybe even like a bounded condition 
}

ros::Subscriber<std_msgs::Float32MultiArray> cmd_sub("set_vel", &velCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  pinMode(5, OUTPUT);
  FL.attach(5);

  pinMode(6, OUTPUT);
  BL.attach(6);

  pinMode(7, OUTPUT);
  FR.attach(7);

  pinMode(8, OUTPUT);
  BR.attach(8);

  pinMode(51, OUTPUT);
  Auger.attach(51);

  pinMode(ENCODER_FL_PINA, INPUT_PULLUP);
  pinMode(ENCODER_FL_PINB, INPUT_PULLUP);

  pinMode(ENCODER_BL_PINA, INPUT_PULLUP);
  pinMode(ENCODER_BL_PINB, INPUT_PULLUP);

  pinMode(ENCODER_FR_PINA, INPUT_PULLUP);
  pinMode(ENCODER_FR_PINB, INPUT_PULLUP);

  pinMode(ENCODER_BR_PINA, INPUT_PULLUP);
  pinMode(ENCODER_BR_PINB, INPUT_PULLUP);

  attachInterrupts();

  //encoder ticks array initialization 
  char dim0_label[] = "encoder_ticks";
  enc_ticks.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2); 
  enc_ticks.layout.dim[0].label = dim0_label;
  enc_ticks.layout.dim[0].size = 4;
  enc_ticks.layout.dim[0].stride = 1*4;
  enc_ticks.data = (long long int *)malloc(sizeof(long long int)*4);
  enc_ticks.layout.dim_length = 0;
  enc_ticks.data_length = 4;

  //vel array initialization
  char dim1_label[] = "velocity_wheels";
  vel_wheels.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  vel_wheels.layout.dim[0].label = dim1_label;
  vel_wheels.layout.dim[0].size = 4;
  vel_wheels.layout.dim[0].stride = 1*4;
  vel_wheels.data = (float *)malloc(sizeof(float)*4);
  vel_wheels.layout.dim_length = 0;
  vel_wheels.data_length = 4;

  nh.advertise(enc_ticks_pub);
  nh.advertise(vel_pub);
  nh.subscribe(cmd_sub);

  Auger.write(140);
}

void loop() {
  nh.spinOnce(); //spin the ros node

  //Goal is to keep loop time consistent, delay as req or ommp method
  currentMillis = millis();
  changeMillis = currentMillis - previousMillis;

  if (changeMillis >= loopTime) {  // run a loop every 10ms          
    previousMillis = currentMillis; // reset the clock to time it

    SpeedPID(changeMillis); //ROS node spun so if setpoint set, should run motors
    enc_ticks.data[0]=FLTicks; //Feedback encoder data, for state estimatation
    enc_ticks.data[1]=BLTicks;
    enc_ticks.data[2]=FRTicks;
    enc_ticks.data[3]=BRTicks;
    enc_ticks_pub.publish(&enc_ticks); //pub int64 multi-array

    vel_wheels.data[0] = Speed_FL;
    vel_wheels.data[1] = Speed_BL;
    vel_wheels.data[2] = Speed_FR;
    vel_wheels.data[3] = Speed_BR;
    vel_pub.publish(&vel_wheels);

    // speed_confirm.data[0]=Speed_FL; //Feedback encoder data, for state estimatation
    // speed_confirm.data[1]=Speed_BL;
    // speed_confirm.data[2]=Speed_FR;
    // speed_confirm.data[3]=Speed_BR;
    // speed_pub.publish(&speed_confirm);
  }
} 