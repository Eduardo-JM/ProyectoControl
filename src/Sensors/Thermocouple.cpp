//
// Created by eduar on 09/01/2024.
//

#include "Thermocouple.h"

Thermocouple::Thermocouple(int sck, int cs, int so):sensor(sck, cs, so){
    pinMode(cs, OUTPUT);
    pinMode(so, INPUT);
    pinMode(sck, OUTPUT);
}

float Thermocouple::getTemperature() {
    return sensor.readCelsius();
}