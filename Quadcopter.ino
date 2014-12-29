/*  Quadcopter

By James Dryden & Ryan Barbaccia

For Penn State ME 445 - Microcomputer Interfacing



*/



#include <Wire.h>
#include <PinChangeInt.h>
#include <QuadPID.h>
#include "Config.h"
#include "Def.h"

#ifdef USE_SERVO_LIB
  #include <Servo.h>
#endif


// ============================================================================
// ===                        GLOBAL VARIABLES                              ===
// ============================================================================

// ----------Rx GLOBALS----------------------
// Declare variables shared b/w interrupts & program as volatile
volatile byte updateFlagsShared;
volatile int rxValShared[NUMBER_CHANNELS];
// define timer variables for interrupts (not volatile b/c not shared)
unsigned long rxStart[NUMBER_CHANNELS];
// define global Rx value variable
int rxVal[NUMBER_CHANNELS];
// variables for Rx desired setpoints
float rxThro, rxRoll, rxPitch, rxYaw;
// variable for rate mode switch state
boolean rateModeSwitch = false;

// ----------MOTOR GLOBALS-------------------
#ifdef USE_SERVO_LIB
// create servo objects for all servo outputs
Servo motors[4];
#endif

// Motor setpoints
int motorVal[4];

// global arming variable for motors
boolean motorsArmed = false;

// ----------Sensor Globals------------------

// new sensor reading flag variable
byte newSensorFlag;

// raw accelerometer and gyro reading vars
int accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// sensor reading offset variables 
int accXOffset, accYOffset, accZOffset;
float gyroXOffset, gyroYOffset, gyroZOffset;

// estimated pitch and roll angles 
float pitch, roll;

#ifdef MAG
  // if using magnetometer, new mag flag, raw mag readings, and calculated heading 
  int magX, magY, magZ;
  int heading;
#endif

//// ============================================================================
//// ===                           PID Class                                  ===
//// ============================================================================
//
//class PID {
//  // General purpose PID controller
//    unsigned long now, dt, lastTime;
//    float error, lastErr, errSum;
//    float P, I, D;
//    float kp, ki, kd;
//    
//  public:
//  PID(float set_kp, float set_ki, float set_kd){
//    kp = set_kp;
//    ki = set_ki;
//    kd = set_kd;
//  }
//  
//  float compute(float setPoint, float input){
//    now = micros();
//    dt = now - lastTime;
//    
//    if(dt >= PID_SMPL_TIME){
//      error = setPoint - input;
//      errSum += error;
//      
//      P = kp * error;
//      
//      /*
//      When elapsed time is greater than 10 times the sample rate,
//      omit the D term which could cause a large output spike. Also 
//      reset errSum to zero the I term. This prevents strange behavior
//      when the throttle is lowerer below cutoff and then raised again. 
//      */
//      if(dt > 10*PID_SMPL_TIME){
//        D = 0;
//        errSum = 0;
//      }
//      else{
//      D = kd * (error - lastErr);
//      }
//      
//      I = constrain(ki*errSum, -I_MAX, I_MAX);
//      
//      lastErr = error;
//      lastTime = now;
//    }
//    
//    return (P + I + D);
//  }
//};

// create each PID object
QuadPID yawPID(YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, PID_SMPL_TIME, I_MAX);
QuadPID rollPID(ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, PID_SMPL_TIME, I_MAX);
QuadPID pitchPID(PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, PID_SMPL_TIME, I_MAX);

// ============================================================================
// ===                              SETUP                                   ===
// ============================================================================

void setup() {
  
  // start Serial
  Serial.begin(115200);
  Serial.println("Quadcopter");
  
  // initialize LEDs
  ledInit();
  
  // initialize sensors. Includes gyro only if ARMED not defined
  sensorInit();
  
  #ifdef MAG
  magInit();
  #endif
  
  // initialize Rx interface
  rxInit();
  // delay to allow Rx readings to begin
  delay(100);
  
  #ifdef ARMED
    // initialize the motors. This begins arming sequence
    motorInit();
  #endif
  
  #ifndef USE_SERIAL
    Serial.end();
  #endif
}

// ============================================================================
// ===                               LOOP                                   ===
// ============================================================================

void loop() {
  
  // update sensor readings and state estimation
  updateSensors();
  updateStateEst();

  
  #ifdef ARMED
  // update Rx control and motors
  flightControl();
  #endif

  
  #ifdef DEBUG_ANGLES
    Serial.print(millis());  Serial.print('\t');
    Serial.print(roll);      Serial.print('\t');
    Serial.println(pitch);
  #endif
  
  #ifdef DEBUG_SENSOR_RAW
    Serial.print(accX);
    Serial.print("\t");
    Serial.print(accY);
    Serial.print("\t");
    Serial.print(accZ);
    Serial.print("\t");
    Serial.print(gyroX);
    Serial.print("\t");
    Serial.print(gyroY);
    Serial.print("\t");
    Serial.println(gyroZ);
  #endif
  
  #ifdef DEBUG_MAG
    Serial.print(magX);
    Serial.print(" ");
    Serial.print(magY);
    Serial.print(" ");
    Serial.println(magZ);
  #endif
  
  #ifdef DEBUG_MOTOR_SETPOINTS
    Serial.print(motorVal[0]); Serial.print('\t');
    Serial.print(motorVal[1]); Serial.print('\t');
    Serial.print(motorVal[2]); Serial.print('\t');
    Serial.println(motorVal[3]);
  #endif
  
  #ifdef LOOP_TIMER
  //static int longestLoopTime;
  static unsigned long startLoopTime;
  unsigned long timeLoopNow = micros();
  if(timeLoopNow > 5000000){
    int elapsed = timeLoopNow - startLoopTime;
    Serial.println(elapsed);
  }
  startLoopTime = timeLoopNow;
  #endif
  
}

