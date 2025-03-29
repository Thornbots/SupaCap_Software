#define NUM_LEDS 10  // Number of LEDs
int ledPins[NUM_LEDS] = {2, 3, 6, 7, 10, 11, 14, 15};  // LED pin array

void setup() {
    // Set all LED pins as outputs
    for (int i = 0; i < NUM_LEDS; i++) {
        pinMode(ledPins[i], OUTPUT);
    }
}

void loop() {
    for (int i = 0; i < NUM_LEDS; i++) {
        digitalWrite(ledPins[i], HIGH);
        delay(100);
        digitalWrite(ledPins[i], LOW);
    }
}
