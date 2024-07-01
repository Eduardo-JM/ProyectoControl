#include <Arduino.h>
#include <math.h>
#include "src/FuzzyControl/EzoPmp.h"
#include "src/utils.h"
#include "src/DistillerControl.h"
#include <PID_v1.h>

#define TIME 2

EzoPmp ezo_pmp;
DistillerControl control(ezo_pmp);

double set_point, integral, signal, temp_signal, error;
float kp = 0, ki = 0.025 , kd = 0.0;
int flag = 0;

float a, b;

void setup() {
    Serial.begin(9600);
    Serial3.begin(9600);

    set_point = 323.15; //273.15 + 40.0;
    integral = 0;
    ezo_pmp.setMaxFlowRate();
}

/**
 * This method executes everytime serial port_0
 * receives a char. This method will read the inputString
 * until a CR (delimiter) is received.
 */
void serialEvent3() {
    String str = Serial3.readStringUntil(CARRIAGE_RETURN);
    ezo_pmp.setResponse(str);
}

void loop() {
  /*error = getCurrentError();
  signal = error * ki * integrateError();

  temp_signal = saturateControlSignal(signal);

  if (signal != temp_signal) {
    integral = 0;
  }

  ezo_pmp.dispenseAtFlowRate((float) temp_signal);
  Serial.print("Flujo: ");
  Serial.println(temp_signal);
  control.printData();*/

  ezo_pmp.stopDispensing();
  /*if (!flag){
    flag = !flag;
    ezo_pmp.dispenseAtFlowRate(40);
  }
  control.printData();*/
  //delay(500);
  /*Serial.println(control.getHeatedWaterTemperature());
  delay(500);*/
}

float getCurrentError(){
  return set_point - control.getHeatedWaterTemperature();
}

float integrateError() {
  Serial.print("Error: ");
  Serial.println(a);
  a = getCurrentError();
  delay(TIME * 1000);
  b = getCurrentError();
  integral = integral + TIME * (a + b) / 2;
  return integral;
}

double saturateControlSignal(double signal){
  if (signal > 50)
    return 50.0;
  if (signal < 3)
    return 3.0;
  return signal;
}