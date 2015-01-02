// ============================================================================
// ===                         LED  FUNCTIONS                               ===
// ============================================================================


void ledInit(){
  // initialize the LED pins for output
  // all LED pins on PORTB, pins 8 to 13
  DDRB |= ( 1<<(R_LED) | 1<<(G_LED) | 1<<(B_LED) );
  
  // set the LED to red
  setLED(RED);
}

void setLED(byte color){
  // --------Set the LED color---------
  
  switch(color){
    case RED:
      PORTB = SET_R;
      break;
    case GREEN:
      PORTB = SET_G;
      break;
    case BLUE:
      PORTB = SET_B;
      break;
    case TEAL:
      PORTB = SET_T;
      break;
  }
}
