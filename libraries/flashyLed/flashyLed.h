/*! \file */
/*!
 * buttons.h
 *
 * Description:
 *
 *  Created on:
 *      Author:
 */

 #ifndef FLASHYLED_H_
 #define FLASHYLED_H_

 #ifdef __cplusplus
extern "C" {
#endif

#include "RPi_Pico_TimerInterrupt.h"      //https://github.com/khoih-prog/RPI_PICO_TimerInterrupt


#define NUM_LEDS 8  // Number of LEDs
#define ALL_LEDS_MASK 255;
int ledPins[NUM_LEDS] = {2, 3, 6, 7, 10, 11, 14, 15};  // LED pin array

class FlashyLed{
    public:
    void init(__uint8_t ledMask, unsigned long period_ms);
    void setLedMask(__uint8_t ledMask);
    void setFlashingPeriod(unsigned long period_ms);
    void startFlashing(void);
    void updateLed(void);
    void stopFlashing(void);
    private:
    bool checkExpired(void);
    void toggleLEDs(void);
    void TimerHandler(void);
    __uint8_t mask = 0;
    unsigned long period = 1000;
};


#ifdef __cplusplus
}
#endif

#endif /* FLASHYLED_H_ */
