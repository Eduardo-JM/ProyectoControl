//
// Created by eduar on 09/01/2024.
//

#ifndef DESALINATOR_WATERLEVELSENSORS_H
#define DESALINATOR_WATERLEVELSENSORS_H

#include <Arduino.h>

class WaterLevelSensors {
public:
    WaterLevelSensors(int signal_pin);
    bool readLevelSensor();
private:
    int signal_pin;
};


#endif //DESALINATOR_WATERLEVELSENSORS_H
