#include <Arduino.h>
#include <avr/wdt.h>

#define POWER_PIN 2
unsigned long prev_power_time;
void PowerUp(void) {
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW);
    prev_power_time = millis();
}

void PowerTask(void) {
    unsigned long cur = millis();
    if (cur - prev_power_time >= 20000) {
        PowerUp();
    } else if (cur - prev_power_time >= 1000) {
        pinMode(POWER_PIN, INPUT);
    }
}

void Reboot(void) {
    wdt_enable(WDTO_15MS);
    while(true);
}
