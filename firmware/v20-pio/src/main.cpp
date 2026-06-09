#include <Arduino.h>
#include "config.h"
#include "led.h"

// v20 stub: alleen LED-feedback aanwezig. Modules komen er incrementeel bij.
// Zie docs/architecture.md voor het volledige module-overzicht.

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("v20-pio stub — LED only, see docs/architecture.md");

    led::init();
    led::set(led::BLINK_1HZ, CRGB::Green);
}

void loop() {
    led::update();
    delay(20);
}
