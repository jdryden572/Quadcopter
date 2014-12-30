// ============================================================================
// ===                          RX FUNCTIONS                                ===
// ============================================================================

byte rxPins[] = {RX_PIN_CH0, RX_PIN_CH1, RX_PIN_CH2, RX_PIN_CH3, RX_PIN_CH4};

void rxInit(){
  
  noInterrupts();
  
  PCICR = 1<<2;  // enable pin change interrupt for PCINT2 (pins 0 to 7)
  
  for(byte i=0; i<NUMBER_CHANNELS; i++){
    DDRD   |= 0<<rxPins[i];    // set pin to INPUT
    PCMSK2 |= 1<<rxPins[i];    // enable pin change interrupt on pin
  }
  
  interrupts();
}

void rxGetNewVals(){
  // --------------------Raw Rx Value Updater Function-------------------------
  // updates values in rxVal with any new RX signals that have been recieved
  
  noInterrupts();  // turn off interrupts while interacting with volatile vars
  
  byte updateFlags = updateFlagsShared;  // take local copy of update flags
   
  // copy the value of each channel into the buffer if it has been updated
  for (int i=0; i<NUMBER_CHANNELS; i++){
    // use bitshift to check if the flag has been raised
    if (updateFlagsShared & 1<<i){
      // check if value is reasonable. 
      // Reject values greater than 2200 micros, these mean an interrupt was missed
      if(600 < rxValShared[i] && rxValShared[i] < 2200){
        rxVal[i] = rxValShared[i];
      }
    }
  }
    
  // now we have a copy of each changed variable. Reset the flag variable and turn interrupts on
  updateFlagsShared = 0;
  interrupts();
  
  rxMapping(updateFlags);
}

void rxMapping(byte updateFlags){
  // -----------------------Rx Value Mapper Function--------------------------
  /* 
  Converts any new Rx values into angle setpoints to be used by the PIDs.
  Only calculates for values that have been updated since last time, indicated
  by the updateFlags parameter. 
  
  For roll/pitch/yaw, uses floating point math instead of the map() function
  because it runs faster.
  */
  
  if(updateFlags & 1<<CHANNEL_THROTTLE){  // New throttle value
    // map throttle values directly 
    rxThro = map(rxVal[CHANNEL_THROTTLE], THROTTLE_CUTOFF, THROTTLE_RMAX, THROTTLE_WMIN, THROTTLE_WMAX);
  }
  if(updateFlags & 1<<CHANNEL_ROLL){  // New roll
    rxRoll  = (float)rxVal[CHANNEL_ROLL]*ROLL_RX_MULT - ROLL_RX_SUB;
    rxRoll  = constrain(rxRoll, -ROLL_A_MAX, ROLL_A_MAX);
  }
  if(updateFlags & 1<<CHANNEL_PITCH){  // New pitch
    rxPitch = (float)rxVal[CHANNEL_PITCH]*PITCH_RX_MULT - PITCH_RX_SUB;
    rxPitch = constrain(rxPitch, -PITCH_A_MAX, PITCH_A_MAX);
  }
  if(updateFlags & 1<<CHANNEL_YAW){  // New yaw
    rxYaw   = (float)rxVal[CHANNEL_YAW]*YAW_RX_MULT - YAW_RX_SUB;
    rxYaw   = constrain(rxYaw, -YAW_A_MAX, YAW_A_MAX);
  }
  if(updateFlags & 1<<CHANNEL_AUX){  // New aux
    if(rxVal[CHANNEL_AUX] > 1500) { 
      rateModeSwitch = true; 
    }
    else {
      rateModeSwitch = false; 
    }
  }
}

// Preprocessor function for pin checking
// Records pulse length for each Rx pin
#define RX_PIN_CHECK(chan_num)                          \
  if(changedPins & 1<<rxPins[chan_num]){                \
    if(pins & 1<<rxPins[chan_num]){                     \
      rxStart[chan_num] = now;                          \
    }                                                   \
    else{                                               \
      rxValShared[chan_num] = now - rxStart[chan_num];  \
      rxStart[chan_num] = 0;                            \
      updateFlagsShared |= 1<<chan_num;                 \
    }                                                   \
  }    


ISR(PCINT2_vect) {
  // Pin change interrupt handler for digital pins 0 to 7
  unsigned long now;
  byte pins, changedPins;
  static byte lastPins;
  
  pins = PIND;  // capture current pin state
  changedPins = pins ^ lastPins;  // toggle last pin state with current state to find changes
  now = micros();
  lastPins = pins;    // remember pin state for next time
  
  for(byte i=0; i<NUMBER_CHANNELS; i++){  // check pins for changes
    RX_PIN_CHECK(i);
  }
  
}

