#include "SupplyPump.h"

SupplyPump::SupplyPump(int relay_pin) {
    this->relay_pin = relay_pin;
    this->is_pumping = false;
    this->last_on = 0;
}

/**
 * Activates pump relay to start the flow
 */
void SupplyPump::activateRelay() {
    digitalWrite(relay_pin, HIGH);
    is_pumping = true;
    last_on = millis();
}

/**
 * Deactivates pump relay to stop the flow
 */
void SupplyPump::deactivateRelay() {
    digitalWrite(relay_pin, LOW);
    is_pumping = false;
    last_on = millis();
}