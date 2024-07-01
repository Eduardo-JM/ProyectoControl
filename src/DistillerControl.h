#ifndef DESALINATOR_DISTILLERCONTROL_H
#define DESALINATOR_DISTILLERCONTROL_H

#include <Arduino.h>
#include "FuzzyControl/FuzzyController.h"
#include "FuzzyControl/EzoPmp.h"
#include "Sensors/Thermistor.h"
#include "Sensors/Thermocouple.h"

#include "PumpControl/SeawaterPump.h"
#include "PumpControl/DistilledWaterPump.h"
#include "PumpControl/HeatedWaterPump.h"

/********************
 * Temperature Sensors
********************/

#define SEA_WATER_TEMPERATURE_SENSOR_PIN A0
#define HEATED_WATER_TEMPERATURE_SENSOR_PIN A1

#define SEA_WATER_LEVEL_SENSOR_PIN 5
#define DISTILLED_WATER_LEVEL_SENSOR_PIN 6

#define SEA_WATER_RELAY_PIN 7
#define DISTILLED_WATER_RELAY_PIN 8
#define HEATED_WATER_RELAY_PIN 4

#define SOLAR_RECEIVER_TEMPERATURE_SENSOR_PIN_SO 9
#define SOLAR_RECEIVER_TEMPERATURE_SENSOR_PIN_CS 10
#define SOLAR_RECEIVER_TEMPERATURE_SENSOR_PIN_SCK 11

class DistillerControl{
public:
    DistillerControl(
            EzoPmp ezoPmp
    ) : seawaterTemperatureSensor(SEA_WATER_TEMPERATURE_SENSOR_PIN),
        heatedWaterTemperatureSensor(HEATED_WATER_TEMPERATURE_SENSOR_PIN),
        receiverTemperatureSensor(
                SOLAR_RECEIVER_TEMPERATURE_SENSOR_PIN_SCK,
                SOLAR_RECEIVER_TEMPERATURE_SENSOR_PIN_CS,
                SOLAR_RECEIVER_TEMPERATURE_SENSOR_PIN_SO),
        fuzzyController(ezoPmp),
        s_pump(SEA_WATER_LEVEL_SENSOR_PIN, SEA_WATER_RELAY_PIN),
        d_pump(
                DISTILLED_WATER_LEVEL_SENSOR_PIN,
                DISTILLED_WATER_RELAY_PIN,
                ezoPmp),
        h_pump(HEATED_WATER_TEMPERATURE_SENSOR_PIN, HEATED_WATER_RELAY_PIN, ezoPmp)
    {
        this->ezoPmp = ezoPmp;
    }
    void initFuzzyControl();
    void initPumpControl();
    void printData();
    float getHeatedWaterTemperature();
private:
    EzoPmp ezoPmp;
    SeawaterPump s_pump;
    DistilledWaterPump d_pump;
    HeatedWaterPump h_pump;
    Thermistor seawaterTemperatureSensor;
    Thermistor heatedWaterTemperatureSensor;
    Thermocouple receiverTemperatureSensor;
    FuzzyController fuzzyController;
};


#endif //DESALINATOR_DISTILLERCONTROL_H
