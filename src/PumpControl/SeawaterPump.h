#ifndef DESALINATOR_SEAWATERPUMP_H
#define DESALINATOR_SEAWATERPUMP_H

#include "SupplyPump.h"
#include "../Sensors/WaterLevelSensors.h"

class SeawaterPump : public SupplyPump {
public:
    SeawaterPump(int level_sensor_pin, int relay_pin)
        : SupplyPump(relay_pin),
          levelSensor(level_sensor_pin){}
    void startFlow() override;
private:
    WaterLevelSensors levelSensor;
};

#endif //DESALINATOR_SEAWATERPUMP_H
