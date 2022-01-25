// Flex balancing robot using MPU6050 DMP (MotionApps v2.0)
// 26/03/2021 by Ferréol Gagey <ferreol.gagey@ens-paris-saclay.fr>

// Changelog:
// 26/03/2021 - simple pid without encoder

//===============================================


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "mbed.h"
#include <math.h>
#include "MC33926.h"
#include "Servo.h"
#include "QEI.h"
//DigitalOut leds[] = {(LED1), (LED2),(LED3),(LED4)};



#include "MPU6050_6Axis_MotionApps20.h" // works

MPU6050 mpu;


#ifndef M_PI
#define M_PI 3.1415
#endif


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


bool blinkState = false;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
int gyr[3];
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[15] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n',0 };

// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

/* Motors*/

MC33926 motorLeft(PA_11,D4,A3);
MC33926 motorRight(D8,PC_8,A2);
float kimot=0.09;
double imotright=0.0;
double imotleft=0.0;
double motorconsuption=0.0;
int compteuri=0;
double motorleftcurrent=0.0;
double motorrightcurrent=0.0;
double biaisright=1;
double biaisleft=0.99;
/* Encoders*/

QEI rightEncoder(D3,D2,NC,64*30); //tickPerRevolution=64tick*30gearratio;
QEI leftEncoder(A4,A5,NC,64.30);
int pulseleftcoder;
int pulserightcoder;
double timeinterval=100000.0; //5*100000(0.5s) 
double timercoder=0.0;
Timer t;

//int pulseleftcoderprev=0;
//int pulserightcoderprev=0;

static float wheelVelocity;
static double wheelPosition, lastWheelPosition, rotationAngle;
float AngleOffset=0.0;
static double PosCmd, rotationCmd;
#define wheelPosGain 0.0015f
#define wheelRateGain 0.003f


/* bluetooth module*/
Serial blue(D1, D0);  // Bluetooth TX, RX
double timerprint=0.0;
double timeprint=1000000.0; //5*100000(0.5s) 
Timer tprint;

/*servo*/

Servo servogauche(A1);
Servo servodroit(A0);

float servodroitposhaute=0.65;
float servogaucheposhaute=0.106;

float servodroitposbas=0.35;
float servogaucheposbas=0.4;

/* Variable declarations */
float pitchAngle = 0;
float rollAngle = 0;
bool dir;           // direction of movement
double Kp = 0.04; //0.5 (entre 0.15 et 0.2)
double Ki = 0; //0.00001
double Kd = 0;//0.01;//0.05;
AnalogIn potkp(PC_5);
AnalogIn potkd(PC_3);
AnalogIn potki(PC_2);
float set_point = -1.8;  //-1.8 -1.1 -1.7      // Angle for staying upright
float errorPID = 0;             // Proportional term (P) in PID
float last_errorPID =0;         // Previous error value
float integral = 0;             // Integral (I)
float derivative = 0;           // Derivative (D)
float outputPID = 0;            // Output of PID calculations
char phone_char;                 //char returned by th bluetooth module

/* LQR commande parameters */

//variable d'états
float pitchVelocity=0.0;
float rollVelocity=0.0;
double linearleftVelocity=0.0;
double linearrightVelocity=0.0;
double linearleftPosition=0.0;
double linearrightPosition=0.0;
float x;
float x_dot;
float theta;
float theta_dot;
float K1lqr=0;
float K2lqr=0;
float K3lqr=0;
float K4lqr=0;
float Rroue=0.072;
float couplelqr= 0.0;

//Variables pour le PID de la boucle en courant//
double Kpcurrentr = 0.13; //0.5 (entre 0.15 et 0.2)
double Kicurrentr = 0; //0.00001
double Kdcurrentr = 0.0;//0.01;//0.05;
float rightcurrenterrorPID = 0;             // Proportional term (P) in PID
float last_rightcurrenterrorPID =0;         // Previous error value
float rightcurrentintegral = 0;             // Integral (I)
float rightcurrentderivative = 0;           // Derivative (D)
float outputrightcurrentPID = 0;            // Output of PID calculations

double Kpcurrentl = 0.13; //0.5 (entre 0.15 et 0.2)
double Kicurrentl = 0; //0.00001
double Kdcurrentl = 0.0;//0.01;//0.05;
float leftcurrenterrorPID = 0;             // Proportional term (P) in PID
float last_leftcurrenterrorPID =0;         // Previous error value
float leftcurrentintegral = 0;             // Integral (I)
float leftcurrentderivative = 0;           // Derivative (D)
float outputleftcurrentPID = 0;            // Output of PID calculations


Ticker toggler1;               // Ticker for led toggling
Ticker gyro;                 // Ticker for periodic call to compFilter funcçs 
Ticker balance;                 // Periodic routine for PID control of balance system
Ticker speed;                  // Periodic routine for speed control
Ticker bluetooth;              // Ticker for navigation

void toggle_led1();
void toggle_led2();
void balancePID();
void wheel_position_velocity();
void balanceControl();
void Forward(float);
void Reverse(float);
void Stop(void);
void Parameters();
void TurnRight(float);
void TurnLeft(float);
float map(float, float, float, float, float);
float constrain(float,float,float);

float valuePotkp;
float valuePotkd;
float valuePotki;
// ================================================================
// === INITIAL SETUP ===
// ================================================================


int main()
{
    
    //Pin Defines for I2C Bus
#define D_SDA                  D14
#define D_SCL                  D15
//#define D_SDA                  p28
//#define D_SCL                  p27
I2C i2c(D_SDA, D_SCL);


// mbed Interface Hardware definitions
    //DigitalOut myled1(LED1);
    //DigitalOut myled2(LED2);
    //DigitalOut myled3(LED3);
    //DigitalOut heartbeatLED(LED4);

// initialize serial communication
// (115200 chosen because it is required for Teapot Demo output, but it's
// really up to you depending on your project)
//Host PC Baudrate (Virtual Com Port on USB)
    #define D_BAUDRATE            9600

// Host PC Communication channels
    Serial pc(USBTX, USBRX); // tx, rx
    

    pc.baud(D_BAUDRATE);
    blue.baud(9600);
    // initialize device
    pc.printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    pc.printf("Testing device connections...\n");
    
    bool mpu6050TestResult = mpu.testConnection();
    if(mpu6050TestResult){
        pc.printf("MPU6050 test passed \n");
    } else{
        pc.printf("MPU6050 test failed \n");
    }  

    // wait for ready
    pc.printf("\nSend any character to begin DMP programming and demo: ");

    //while(!pc.readable());
            //pc.getc();
    //pc.printf("\n");
     
    // load and configure the DMP
    pc.printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(2);
    mpu.setYGyroOffset(62);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(52); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        pc.printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        pc.printf("Enabling interrupt detection (Arduino external interrupt 0)...\n");
//        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        pc.printf("DMP ready! Waiting for first interrupt...\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        pc.printf("DMP Initialization failed (code ");
        pc.printf("%u",devStatus);
        pc.printf(")\n");
    }
    //gyro.attach(&getAngle, 0.005);         // Call the complementaryFilter func. every 5 ms (200 Hz sampling period) 
    balance.attach(&balancePID,  0.01);       // Same period with balancePID
    speed.attach(&balanceControl, 0.01);
    bluetooth.attach(&Parameters, 0.02);
    servogauche.calibrate(0.001, 90); 
    servodroit.calibrate(0.001, 90);
    servogauche.write(servogaucheposhaute);
    servodroit.write(servodroitposhaute);
    t.start();
    tprint.start();


// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================

while(1)
{
    if(pc.readable()) 
        {       
        phone_char = pc.getc();
        //Parameters(phone_char);
        //pc.putc(phone_char);
        //phone_char='R';        
        }
    valuePotkp = potkp.read(); // the first way of reading from analog input        
    valuePotki = potki.read(); // the first way of reading from analog input
    valuePotkd = potkd.read(); // the first way of reading from analog input
    
    Kp=map(valuePotkp,0,1,0.02,0.12);
    Ki=map(valuePotki,0,1,0,0.2);
    Kd=map(valuePotkd,0,1,0,0.05);
    
    timerprint = tprint.read_us();
    
    if (timerprint>timeprint) {
            //pc.printf("Kp : %f ",Kp);
            //pc.printf("Kd : %f ",Kd);
            //pc.printf("Ki : %f ",Ki);
            //pc.printf("setpoint = %f   ",set_point);
            //pc.printf("Bluetooth Start\r\n");
            //pc.printf("blue = %c    ",phone_char);
            pc.printf("comptright = %d    ",pulserightcoder);
            pc.printf("postion roue = %f    ",wheelPosition);
            pc.printf("comptleft = %d    ",pulseleftcoder);
            pc.printf("vitesse roue = %f    ",wheelVelocity);
            //pc.printf("time %f    ",timercoder);
            //pc.printf("timeinterva %f    ",timeinterval);  
            pc.printf("Angle rotation : %.1f    ",rotationAngle);   
            //pc.printf("courrant droit %f    ",motorrightcurrent*1000/(0.525)-biaisright);
            //pc.printf("courrant gauche %f\r\n    ",motorleftcurrent*1000/(0.525)-biaisleft);
            //pc.printf("Error = %f\r\n",outputPID);            
            //pc.printf("integral = %f\r\n",integral);
            pc.printf("Pitch Angle: %.1f\r\n    ",pitchAngle);
            tprint.reset();
            }    
    timercoder = t.read_us();
    
    if (timercoder>timeinterval){
        wheel_position_velocity();
        t.reset();
        }
    
    imotright=imotright + motorRight.ReadCurrentFeedback();
    imotleft=imotleft + motorLeft.ReadCurrentFeedback();
    compteuri=compteuri+1;
    if (compteuri>=2) {
        motorrightcurrent=imotright/compteuri;
        motorleftcurrent=imotleft/compteuri;
        imotright=0.0;
        imotleft=0.0;
        compteuri=0;
        } 
    
    // if programming failed, don't try to do anything
    if (!dmpReady) continue;
        //myled2=0;
        
    // wait for MPU interrupt or extra packet(s) available
//    while (!mpuInterrupt && fifoCount < packetSize) {
//      while (!mpuIntStatus && fifoCount < packetSize) 
      {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
//        fifoCount= mpu.getFIFOCount();
//        mpuIntStatus = mpu.getIntStatus();
    }
    wait_us(500);
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //pc.printf("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetGyro(gyr,fifoBuffer);
            rollAngle=ypr[1] * 180/M_PI;
            pitchAngle=ypr[2] * 180/M_PI - set_point;
            pitchVelocity=gyr[1] * 180/M_PI;
            rollVelocity=gyr[2] * 180/M_PI;
            //pc.printf("ypr\t");
//            pc.printf("%f3.2",ypr[0] * 180/M_PI);
//            pc.printf("\t");
//            pc.printf("%f3.2",ypr[1] * 180/M_PI);
//            pc.printf("\t");
//            pc.printf("%f3.2\n",ypr[2] * 180/M_PI);
#endif
    }
  }
}

void balancePID()
{
    AngleOffset= (wheelPosition - PosCmd) * wheelPosGain  + wheelVelocity * wheelRateGain; //PD controller
    AngleOffset=constrain(AngleOffset,-5.5f, 5.5f);
    
    errorPID = AngleOffset - pitchAngle;          //Proportional (P)
    integral += errorPID;                       //Integral (I)
    derivative = errorPID - last_errorPID;      //Derivative (D)
    
    last_errorPID = errorPID;                   //Save current value for next iteration
    integral=constrain(integral,-70,70);
    
    outputPID = (Kp * errorPID) + (Ki * integral) + (Kd * derivative);  //PID calculation
    
    outputPID=constrain(outputPID,-0.8f,0.8f);
}

void balanceControl() 
{    
    int direction=0;                                            // Variable to hold direction we want to drive
    if (outputPID>0)direction=1;                                 // Positive speed indicates forward
    if (outputPID<0)direction=2;                                 // Negative speed indicates backwards
    if((abs(pitchAngle)>30.00)){
        Stop();
        integral=0;
        direction=0;
        wheelPosition = 0;
        lastWheelPosition = 0;
        wheelVelocity = 0;
        leftEncoder.reset();
        rightEncoder.reset();
        PosCmd = 0;
        rotationCmd = 0; 
        t.reset();       
        }
  
    switch( direction) {                                        // Depending on what direction was passed
        case 0:                                                // Stop case
            Stop();
            break;
        case 1:                                                 // Forward case
            Forward(abs(outputPID));
            break;
        case 2:                                                 // Backwards
            Reverse(abs(outputPID));
            break;
        default:                                                // Catch-all (Stop)
            Stop();
            break;
    }    
}

void Stop(void)
{
    motorRight.SetPWMPulsewidth(2,0);
    motorLeft.SetPWMPulsewidth(2,0); 
}
 
void Forward(float s)
 {
    
    motorLeft.SetPWMPulsewidth(1,s);
    motorRight.SetPWMPulsewidth(1,s);
 }

void TurnRight(float s)
 {
     motorRight.SetPWMPulsewidth(0,s);
     motorLeft.SetPWMPulsewidth(1,s);
 }
 
void TurnLeft(float s)
 {
     motorRight.SetPWMPulsewidth(1,s);
     motorLeft.SetPWMPulsewidth(0,s);
 }

void Reverse(float s)
 {
    motorRight.SetPWMPulsewidth(0,s);
    motorLeft.SetPWMPulsewidth(0,s); 
 }
 
 float constrain(float x,float min,float max)
 {
 if (x<min){
     return min;
     }
 else if (x>max){
     return max;
     }
 else 
    return x; 
 }
 
 void Parameters()
 {
     switch (phone_char)
            {   case 'E':
                Kp=Kp+0.001;
                phone_char=' ';
                break;
                
                case 'R':
                Kp=Kp+0.01;
                phone_char=' ';
                break;
                
                case 'A':
                Kp=Kp-0.01;
                phone_char=' ';
                break;
                
                case 'Z':
                Kp=Kp-0.001;
                phone_char=' ';
                break;
                
                case 'P':
                Ki=Ki+0.01;
                phone_char=' ';
                break;
                
                case 'O':
                Ki=Ki+0.001;
                phone_char=' ';
                break;
                
                case 'I':
                Ki=Ki-0.001;
                phone_char=' ';
                break;
                
                case 'U':
                Ki=Ki-0.01;
                phone_char=' ';
                break;
                
                case 'F':
                Kd=Kd+0.01;
                phone_char=' ';
                break;
                
                case 'Q':
                Kd=Kd-0.01;
                phone_char=' ';
                break;
                
                case 'S':
                Kd=Kd-0.001;
                phone_char=' ';
                break;
                
                case 'D':
                Kd=Kd+0.001;
                phone_char=' ';
                break;
                
                case 'X':
                set_point=set_point-0.01;
                phone_char=' ';
                break;
                case 'W':
                set_point=set_point-0.1;
                phone_char=' ';
                break;
                
                case 'C':
                set_point=set_point+0.01;
                phone_char=' ';
                break;
                case 'V':
                set_point=set_point+0.1;
                phone_char=' ';
                break;
                
                default:
                break;
            }
}

void Navigate()
{
        switch (phone_char)
            {
                case 'f':
                Forward(0.7);
                break;
                
                case 'F':
                Forward(0.7);
                break;
                
                case 'b': 
                Reverse(0.7);
                break;
                
                case 'B': 
                Reverse(0.7);
                break;
                
                case 'r':
                TurnRight(0.7);
                break;
                
                case 'R':
                TurnRight(0.7);
                break;
                
                case 'l':
                TurnLeft(0.7);
                break;
                
                case 'L':
                TurnLeft(0.7);
                break;
                
                default:
                Stop();
            }
 }
 
 void wheel_position_velocity() {
    pulseleftcoder=-1*leftEncoder.getPulses();
    pulserightcoder=rightEncoder.getPulses();
    wheelPosition = 0.5f*(float(pulseleftcoder) + float(pulserightcoder))*1/180*2*3.14*Rroue;
    wheelVelocity = 10.0f * (wheelPosition - lastWheelPosition); //calcul tous les 10hz
    rotationAngle = 0.5f * (pulseleftcoder - pulserightcoder) *1/180*2*3.14*Rroue * 2*Rroue/280; // Rotation angle in deg.  Note: wheel diam = 2*Rroue mm  and dist between wheels = 280 mm, so rotation angle =  2*Rroue/ 280 = 0.406 * wheel angle 
    lastWheelPosition = wheelPosition;
//    linearleftPosition=-float(pulseleftcoder);
//    linearrightPosition=float(pulserightcoder)*1/180*2*3.14*Rroue;
//    pulseleftVelocity=(pulseleftcoder-pulseleftcoderprev)*1000000/timercoder; //vitesse = deltapulse/deltat * conversion secondes
//    pulserightVelocity=(pulserightcoder-pulserightcoderprev)*1000000/timercoder;
//    linearleftVelocity=-pulseleftVelocity/180*2*3.14*Rroue;  //360*2*pi*Rayon pour avoir la vitesse en m/s
//    linearrightVelocity=pulserightVelocity/180*2*3.14*Rroue;  //360*2*pi*Rayon pour avoir la vitesse en m/s
//    pulseleftcoderprev=pulseleftcoder;
//    pulserightcoderprev=pulserightcoder;
     }

float map(float in, float inMin, float inMax, float outMin, float outMax) {
  // check it's within the range
  if (inMin<inMax) { 
    if (in <= inMin) 
      return outMin;
    if (in >= inMax)
      return outMax;
  } else {  // cope with input range being backwards.
    if (in >= inMin) 
      return outMin;
    if (in <= inMax)
      return outMax;
  }
  // calculate how far into the range we are
  float scale = (in-inMin)/(inMax-inMin);
  // calculate the output.
  return outMin + scale*(outMax-outMin);
}