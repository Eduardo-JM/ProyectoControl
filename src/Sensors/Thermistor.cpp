//
// Created by eduar on 09/01/2024.
//

#include "Thermistor.h"

Thermistor::Thermistor(int signal_pin) {
    this->signal_pin = signal_pin;
}

float Thermistor::getTemperature() {
    float Vo = analogRead(signal_pin);
    float R = THERMISTOR_FIXED_RESISTANCE * ((1023.0 / (float) Vo) - (float) 1.0);
    float log_r = log(R);
    return (1 / (STEINHART_HART_CONSTANT_A + log_r * STEINHART_HART_CONSTANT_B + STEINHART_HART_CONSTANT_C * pow(log_r,3)));
}