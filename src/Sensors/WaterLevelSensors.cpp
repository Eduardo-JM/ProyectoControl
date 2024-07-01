//
// Created by eduar on 09/01/2024.
//

#include "WaterLevelSensors.h"

WaterLevelSensors::WaterLevelSensors(int signal_pin) {
    this->signal_pin = signal_pin;
}

bool WaterLevelSensors::readLevelSensor() {
    return digitalRead(signal_pin) == HIGH;
}