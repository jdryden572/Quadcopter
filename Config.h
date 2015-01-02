
// ============================================================================
// ===                          CONFIGURATION                               ===
// ============================================================================


#define ARMED    // Quad safe if not defined.


// ============================================================================
// ===                            DEBUGGING                                 ===
// ============================================================================

// *****************Debug Printing********************

//#define DEBUG_ANGLES          // print roll and pitch
#define DEBUG_SENSOR_RAW      // print raw sensor vals
//#define DEBUG_MAG             // print mag readings
//#define DEBUG_RX_RAW          // print raw Rx channel values
//#define DEBUG_RX_ANGLES       // print mapped Rx setpoints
//#define DEBUG_PID_ROLL        // print roll PID output
//#define DEBUG_PID_PITCH       // print pitch PID output
//#define DEBUG_PID_YAW         // print yaw PID output
//#define DEBUG_MOTOR_SETPOINTS // print values to be sent to motors
//#define TEST_ROLL             // print roll setpoint and measured roll angle
//#define LOOP_TIMER            // print loop times

//#define USE_SERIAL            // enable Serial even if no debugging selected

// *************Alternate Motor Control***************

//#define RX_MIXING_ONLY        // skip PIDs, directly mix 4 Rx channels into motor output
//#define FORWARD_INDIV 0       // forward throttle to one motor only, selected by number



// ============================================================================
// ===                          PID CONFIG                                  ===
// ============================================================================

#define RATE_KP 1.2
#define RATE_KD 0
#define RATE_KI 0

#define ANG_KP 1.5
#define ANG_KD 0.3
#define ANG_KI 0

#define YAW_KP 3
#define YAW_KD 0
#define YAW_KI 0
  
#define PID_SMPLRT 500    // PID update rate in Hz

#define I_MAX 100         // Max output if integral control term

#define ROLL_A_MAX 20.0   // Max roll angle setpoint in deg. 
#define PITCH_A_MAX 20.0  // Max pitch angle setpoint in deg. 
#define YAW_A_MAX 60.0    // Max yaw rate setpoint in deg/s.

#define RATE_RX_SCALE 4.5   // Multiplier for angle setpoint, for use in rate mode


// ============================================================================
// ===                          SENSOR CONFIG                               ===
// ============================================================================

/* 
The ADXL345, ITG3200, and HMC5883L are all capable of operating at a 400kHz
clock speed. At 400kHz, reading a sensor takes approx 550us. At 100kHz (the
default clock speed) a reading takes approx 1200us.
*/

#define I2C_FAST_MODE    // enable to run I2C at 400kHz clock speed


// **************State Estimation*******************
//#define COMP_FILTER
#define COMP_SPLIT 0.99 // gyro bias for complimentary filter

#define ACCEL_GAIN 1    // accelerometer gain for state estimation

// *************ADXL435 Accelerometer***************
//#define CALIBRATE_ACC
#define ACC_CAL_SMPL_NUM 100

#define ACC_OFFSET_X -12
#define ACC_OFFSET_Y -15
#define ACC_OFFSET_Z -32

// **************ITG-3200 Gyro**********************
#define GYRO_CAL_SMPL_NUM 200  // average 100 readings for offset

#define GYRO_OFFSET_X 0.42
#define GYRO_OFFSET_Y 2.65
#define GYRO_OFFSET_Z 0.21

// **************HMC5883L Compass*******************

//#define MAG



// ============================================================================
// ===                          MOTOR CONFIG                                ===
// ============================================================================

//#define USE_SERVO_LIB
#define USE_MOTOR_PWM


#define MOTOR_ARM_START 1000    // minimum motor level for motors to spin
#define MOTOR_MAX_LEVEL 1800    // max motor power from throttle only



// ============================================================================
// ===                            RX CONFIG                                 ===
// ============================================================================

#define THROTTLE_CUTOFF 1050  // threshold for throttle input. Set to just above bottom stick.
#define THROTTLE_RMAX 1860    // maximum throttle value

#define RX_DEADZONE 8  // +/- deadzone in micros.

// change values to -1 to reverse a channel
#define ROLL_DIR -1
#define PITCH_DIR -1
#define YAW_DIR 1

// pins to be used to read reciever signals for each channel
#define RX_PIN_CH0 6
#define RX_PIN_CH1 7
#define RX_PIN_CH2 2
#define RX_PIN_CH3 4
#define RX_PIN_CH4 5

// which channel corresponds to which input
#define THRO 0
#define ROLL 1
#define PITCH 2
#define YAW 3
#define AUX 4

// ============================================================================
// ===                           LED CONFIG                                 ===
// ============================================================================

#define R_LED  13
#define G_LED  12
#define B_LED  8

