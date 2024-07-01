#ifndef DESALINATOR_SUPPLYPUMP_H
#define DESALINATOR_SUPPLYPUMP_H

#include <Arduino.h>

class SupplyPump {
public:
    SupplyPump(int relay_pin);
    virtual void startFlow() = 0;
protected:
    int relay_pin;
    void activateRelay();
    void deactivateRelay();
    bool is_pumping;
    unsigned long long last_on;
};

#endif //DESALINATOR_SUPPLYPUMP_H
