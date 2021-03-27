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
//MC33926 motorRight(D8,D9,A2);
MC33926 motorLeft(D4,D6,A3);

/* Encoders*/

QEI rightEncoder(D3,D2,NC,360);
QEI leftEncoder(A4,A5,NC,360);
int pulseleftcoder;
int pulserightcoder;

/* bluetooth module*/
Serial blue(D1, D0);  // Bluetooth TX, RX


/* Variable declarations */
float pitchAngle = 0;
float rollAngle = 0;
bool dir;           // direction of movement
float Kp = 0.5;
float Ki = 0.00001;
float Kd = 0.01;//0.01;//0.05;
float set_point = -0.8;         // Angle for staying upright
float errorPID = 0;             // Proportional term (P) in PID
float last_errorPID =0;         // Previous error value
float integral = 0;             // Integral (I)
float derivative = 0;           // Derivative (D)
float outputPID = 0;            // Output of PID calculations
int phone_char;                 //char returned by th bluetooth module



Ticker toggler1;               // Ticker for led toggling
Ticker gyro;                 // Ticker for periodic call to compFilter funcçs 
Ticker balance;                 // Periodic routine for PID control of balance system
Ticker speed;                  // Periodic routine for speed control
Ticker bluetooth;              // Ticker for navigation

void toggle_led1();
void toggle_led2();
void getAngle();
void balancePID();
void balanceControl();
void Forward(float);
void Reverse(float);
void Stop(void);
void Navigate();
void TurnRight(float);
void TurnLeft(float);

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
    #define D_BAUDRATE            115200

// Host PC Communication channels
    Serial pc(USBTX, USBRX); // tx, rx

    pc.baud(D_BAUDRATE);
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
    mpu.setXGyroOffset(62);
    mpu.setYGyroOffset(1);
    mpu.setZGyroOffset(63);
    mpu.setZAccelOffset(16282); // 1688 factory default for my test chip

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
    balance.attach(&balancePID,  0.010);       // Same period with balancePID
    speed.attach(&balanceControl, 0.01);
    bluetooth.attach(&Navigate, 0.05);


// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================

while(1)
{
    if(blue.readable()) 
        {       
        phone_char = blue.getc();
        pc.putc(phone_char);
        pc.printf("Bluetooth Start\r\n");
        //myled = !myled;
        }
      pulseleftcoder=leftEncoder.getPulses();
    pc.printf("compteur = %d    ",pulseleftcoder);    
    pc.printf("Roll Angle: %.1f, Pitch Angle: %.1f    ",rollAngle,pitchAngle);
    pc.printf("Error = %f\r\n",outputPID);
  
    
    // if programming failed, don't try to do anything
    if (!dmpReady) ;//continue;
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
            rollAngle=ypr[1] * 180/M_PI;
            pitchAngle=ypr[2] * 180/M_PI;
            //pc.printf("ypr\t");
//            pc.printf("%f3.2",ypr[0] * 180/M_PI);
//            pc.printf("\t");
//            pc.printf("%f3.2",ypr[1] * 180/M_PI);
//            pc.printf("\t");
//            pc.printf("%f3.2\n",ypr[2] * 180/M_PI);
#endif
        // blink LED to indicate activity
        //blinkState = !blinkState;
        //myled1 = blinkState;
    }
  }
}

void getAngle(){
    
}

void balancePID()
{
    errorPID = set_point - pitchAngle;          //Proportional (P) 
    integral += errorPID;                       //Integral (I)
    derivative = errorPID - last_errorPID;      //Derivative (D)
    
    last_errorPID = errorPID;                   //Save current value for next iteration
    
    outputPID = (Kp * errorPID) + (Ki * integral) + (Kd * derivative);  //PID calculation
    
    /* errorPID is restricted between -1 and 1 */ 
    if(outputPID > 0.8)
        outputPID = 0.8;
    else if(outputPID < -0.8)
        outputPID = -0.8;   
}

void balanceControl() 
{    
    int direction=0;                                            // Variable to hold direction we want to drive
    if (outputPID>0)direction=1;                                 // Positive speed indicates forward
    if (outputPID<0)direction=2;                                 // Negative speed indicates backwards
    if((abs(pitchAngle)>10.00))direction=0; 
  
    switch( direction) {                                        // Depending on what direction was passed
        case 0:                                                 // Stop case
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
    //motorRight.SetPWMPulsewidth(2,0);
    motorLeft.SetPWMPulsewidth(2,0); 
}
 
void Forward(float s)
 {
    //motorRight.SetPWMPulsewidth(1,s);
    motorLeft.SetPWMPulsewidth(1,s); 
 }

void TurnRight(float s)
 {
     //motorRight.SetPWMPulsewidth(0,s);
     motorLeft.SetPWMPulsewidth(1,s);
 }
 
void TurnLeft(float s)
 {
     //motorRight.SetPWMPulsewidth(1,s);
     motorLeft.SetPWMPulsewidth(0,s);
 }

void Reverse(float s)
 {
    //motorRight.SetPWMPulsewidth(0,s);
    motorLeft.SetPWMPulsewidth(0,s); 
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