
// ============================================================================
// ===                        STATE ESTIMATION                              ===
// ============================================================================

#ifndef COMP_FILTER
void updateStateEst(){
  /*
  -----------------------Attitude Estimation Function--------------------------
  
  Combines accelerometer and gyro data to estimate the roll and pitch.
  Accelerometer gain adjusted with ACCEL_GAIN. 
 
  Roll Directions:
      Positive roll: craft rolled to the right
      Negative roll: craft rolled to the left
      
  Pitch Directions:
      Positive pitch: craft pitched with nose up
      Negative pitch: craft pitched with nose down
  */
  
  static unsigned long lastStateEstTime; 
  static float pitch_acc, roll_acc;
  float errorPitch, errorRoll;
  
  if(newSensorFlag){
    // only run if new sensor reading has been taken
    
    unsigned long t = micros();
    float dt = (float)(t-lastStateEstTime)/1000000.0;
    lastStateEstTime = t;
    
    if(newSensorFlag & NEW_ACC_FLAG){
      // calculate attitude from accelerometer
      pitch_acc = -atan2(accY, accZ) * 180.0 / PI;
      roll_acc  = -atan2(accX, accZ) * 180.0 / PI;
    }
    
    // compare accel attitude to last estimate to get error
    errorPitch = pitch - pitch_acc;
    errorRoll = roll - roll_acc;
    
    if(newSensorFlag & NEW_GYRO_FLAG){
      // combine with gyro reading and integrate
      pitch = pitch + (-gyroX - errorPitch*ACCEL_GAIN)*dt;
      roll = roll + (-gyroY - errorRoll*ACCEL_GAIN)*dt;
    }
    
    #ifdef MAG
    if(newSensorFlag & NEW_MAG_FLAG){
      heading = (int)((atan2(magX, magY) * 180 / PI)+0.5);
      if(heading<0){
        heading += 360;
      }
    }
    #endif
    
    newSensorFlag = 0;
  }
}
#endif

#ifdef COMP_FILTER
void updateStateEst(){
  // EXPERIMENTAL. Uses simple complementary to estimate state.
  float accRoll, accPitch;
  float errorRoll, errorPitch;
  static unsigned long tm;
  
  unsigned long t = micros();
  float dt = (float)(t-tm)/1000000.0;
  tm = t;
  
  accRoll  = -atan2(accX, accZ) * 180 / PI;
  accPitch = -atan2(accY, accZ) * 180 / PI;
  
  roll  = COMP_SPLIT * (-gyroY*dt +  roll) + (1-COMP_SPLIT) * accRoll;
  pitch = COMP_SPLIT * (-gyroX*dt + pitch) + (1-COMP_SPLIT) * accPitch; 
}
#endif
