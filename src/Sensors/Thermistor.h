//
// Created by eduar on 09/01/2024.
//

#ifndef DESALINATOR_THERMISTOR_H
#define DESALINATOR_THERMISTOR_H

#include "TemperatureSensor.h"

#define THERMISTOR_FIXED_RESISTANCE 10000
#define STEINHART_HART_CONSTANT_A 0.00112866
#define STEINHART_HART_CONSTANT_B 0.00023422
#define STEINHART_HART_CONSTANT_C (8.7159 * pow(10,-8))

class Thermistor : public TemperatureSensor {
public:
    Thermistor(int signal_pin);
    float getTemperature() override;
private:
    float signal_pin;
};


#endif //DESALINATOR_THERMISTOR_H
