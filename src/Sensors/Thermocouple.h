//
// Created by eduar on 09/01/2024.
//

#ifndef DESALINATOR_THERMOCOUPLE_H
#define DESALINATOR_THERMOCOUPLE_H

#include "TemperatureSensor.h"
#include <Wire.h>
#include "max6675.h"

class Thermocouple : public TemperatureSensor {
public:
    Thermocouple(int sck, int cs, int so);
    float getTemperature() override;
private:
    MAX6675 sensor;
};


#endif //DESALINATOR_THERMOCOUPLE_H
