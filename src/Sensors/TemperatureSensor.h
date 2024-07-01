//
// Created by eduar on 09/01/2024.
//

#ifndef DESALINATOR_TEMPERATURESENSOR_H
#define DESALINATOR_TEMPERATURESENSOR_H

#include <Arduino.h>

class TemperatureSensor {
public:
    virtual float getTemperature() = 0;
    virtual ~TemperatureSensor(){};
};


#endif //DESALINATOR_TEMPERATURESENSOR_H
