#include "flashyLed.h"

void init(__uint8_t ledMask, unsigned long period_ms){
  mask = ledMask;
  period = period_ms;

    for (int i = 0 ; i < NUM_LEDS; i++){
        pinMode(ledPins[i], OUTPUT);
    }
}

(PinMode)(mask & (__uint8_t)pow(2, i)


void setLedMask(__uint8_t ledMask){
  init(ledMask, period);
  
}

void toggleLeds(void){
  
}

void TimerHandler(void){
  toggleLeds();
}



void loop() {
    for (int i = 0; i < NUM_LEDS; i++) {
        digitalWrite(ledPins[i], HIGH);
        delay(100);
        digitalWrite(ledPins[i], LOW);
    }
}
