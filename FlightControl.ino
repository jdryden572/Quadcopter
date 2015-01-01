/*
  Quad motor configuration

   ->     <- 
    M0   M1
      \ /
      / \
    M3   M2
   ->     <-

*/

// ============================================================================
// ===                          FLIGHT CONTROL                              ===
// ============================================================================

void flightControl() {
  /*
  -------------------------FLIGHT CONTROL SEQUENCE-----------------------------
  
  Calls functions to update Rx values, calculate motor control values, and 
  sends values to the motors.
  */
  
  // Check if any new inputs have been received
  if (updateFlagsShared){
    // get any new RC inputs
    rxGetNewVals();
  }
  
  if(!motorsArmed){ setLED(RED); }
  else{
    if(rateModeSwitch){ setLED(BLUE); }
    else              { setLED(GREEN); }
  }  
  
  
  #ifdef DEBUG_RX_RAW
  Serial.print(rxVal[CHANNEL_THROTTLE]); Serial.print('\t'); 
  Serial.print(rxVal[CHANNEL_ROLL]);     Serial.print('\t');
  Serial.print(rxVal[CHANNEL_PITCH]);    Serial.print('\t');
  Serial.print(rxVal[CHANNEL_YAW]);      Serial.print('\t');
  Serial.println(rxVal[CHANNEL_AUX]);
  #endif


  // motor setpoints for each axis of motion
  int setRoll, setPitch, setYaw;
  
    // flag for whether arm or disarm command was recieved last loop
  static boolean disarmRequested = false;
  static boolean armRequested = false;
  // time for when arm or disarm command was recieved
  static unsigned long armRequestTime, disarmRequestTime;
  // flags for re-arming motors
  static boolean throDown = false; 
  static boolean throUp   = false;
  
  if(motorsArmed && rxVal[CHANNEL_THROTTLE] > THROTTLE_CUTOFF){  // only spin motors if armed and throttle is above cutoff
    
    disarmRequested = false;  // reset disarm request flag
    
    #ifdef DEBUG_RX_ANGLES
      Serial.print(rxThro); Serial.print('\t'); 
      Serial.print(rxRoll); Serial.print('\t');
      Serial.print(rxPitch); Serial.print('\t');
      Serial.println(rxYaw);
    #endif
    
    #if defined(PID_CONTROL)
      // compute controller output values
      if(rateModeSwitch){
        setRoll = (int)rollRatePID.compute((float)rxRoll*RATE_RX_SCALE, -gyroY);
        setPitch = (int)pitchRatePID.compute((float)rxPitch*RATE_RX_SCALE, -gyroX);
      }
      else{
        setRoll =  (int)rollAnglePID.compute((float)rxRoll, roll);
        setPitch = (int)pitchAnglePID.compute((float)rxPitch, pitch);
      }
      setYaw =   (int)yawRatePID.compute((float)rxYaw, gyroZ);
      
      #ifdef DEBUG_PID_ROLL
        Serial.print(rollAnglePID.P); Serial.print('\t');
        Serial.print(rollAnglePID.I); Serial.print('\t');
        Serial.println(rollAnglePID.D);
      #endif
    
    #elif defined(RX_MIXING_ONLY)
      // Mixes Rx outputs only, no PID control
      setRoll  = rxRoll;
      setPitch = rxPitch;
      setYaw   = rxYaw;
    #endif
    
    motorVal[0] = rxThro + setRoll + setPitch + setYaw;
    motorVal[1] = rxThro - setRoll + setPitch - setYaw;
    motorVal[2] = rxThro - setRoll - setPitch + setYaw;
    motorVal[3] = rxThro + setRoll - setPitch - setYaw;
    
    #ifdef FORWARD_INDIV
      // TESTING ONLY. Forward throttle to single motor.
      for(byte i=0; i<4; i++){
        if(i == FORWARD_INDIV){ motorVal[i] = rxThro; }
        else{ motorVal[i] = MOTOR_ARM_START - 50; }
      }          
    #endif
    
  }
  
  else if(motorsArmed){  
    // if motors armed but throttle not above cutoff, don't spin any motors.
    motorVal[0] = MOTOR_ARM_START - 50;
    motorVal[1] = MOTOR_ARM_START - 50;
    motorVal[2] = MOTOR_ARM_START - 50;
    motorVal[3] = MOTOR_ARM_START - 50;
    
    /*  MOTOR DISARM CHECK
    
    Motors may be disarmed mid-program by holding throttle down and yaw full left for 1 second.
    
    When throttle down, check for full left yaw. If it has been held for 1 second, disarm the motors.
    */
    
    if(rxVal[CHANNEL_YAW] > 1850){  // if yaw stick is fully left while motors armed and throttle low
      if(disarmRequested){  // if a disarm request was received in the prev. loop
        if((millis() - disarmRequestTime) > 1000){  // if 1 second have elapsed since first disarm request
          disarmMotors();           // disarm the motors
          motorsArmed = false;
          disarmRequested = false;  // reset disarm request flag
        }
      }
      else{  // this is the first disarm request
        disarmRequested = true;  // set the flag
        disarmRequestTime = millis();  // save the time
      }
    }
    else{  // yaw stick is not in disarm position (fully left yaw, throttle down)
      disarmRequested = false;
    }
  }
  
  else{  // if motors not armed
    /* MOTOR RE-ARM CHECK
    
    Once disarmed mid-program, motors may be re-armed by holding throttle down and yaw full 
    right for one second.
    
    */
    
    if(rxVal[CHANNEL_YAW] < 1150){  // if yaw stick fully right while throttle is fully down
      if(armRequested){  // if a arm request was received last loop
        if((millis() - armRequestTime) > 1000){  // if one second has elapsed since first arm request
          rearmMotors();        // rearm motors
          motorsArmed = true;  
          armRequested = false; // reset arm request flag
        }
      }
      else{  // this is first arm request
        armRequested = true;  // set flag
        armRequestTime = millis();  // record time
      }
    }
    else{  // yaw stick not in arm position, reset flag
      armRequested = false;
    }
  }
  
  // write values to motors
  setMotors();
}
