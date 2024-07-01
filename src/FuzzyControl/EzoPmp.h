#ifndef DESALINATOR_EZOPMP_H
#define DESALINATOR_EZOPMP_H

#include <Arduino.h>
#include "../utils.h"
#include <math.h>

#define CARRIAGE_RETURN '\r'
#define MIN_FLOW_RATE 0.5
#define MAX_FLOW_RATE 105

typedef struct PumpResponse {
    float numeric;
    bool success;
} PumpResponse;

class EzoPmp {
public:
    EzoPmp();
    void stopDispensing();
    void dispenseAtFlowRate(float flow_rate);
    void setResponse(String r);
    float getMaxFlowRate();
    float getVolumeDispensed();
    void clearVolumeDispensed();
    void setMaxFlowRate();
    bool getCurrentStatus();
private:
    void executeCommand(String command);
    PumpResponse response;
    float max_flow_rate;
    bool current_status;
    bool initialized;
};


#endif //DESALINATOR_EZOPMP_H


