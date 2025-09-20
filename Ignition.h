/*/
#ifndef IGNITION_H
#define IGNITION_H

#include "Arduino.h"
#include "PCF8574.h"

#define I2C_ADDRESS_IGNITION_PORT PCF8574_ADDRESS_2
#define IGNITION_PIN_ACC 0
#define IGNITION_PIN_ST  1
#define IGNITION_PIN_BAT 2
#define IGNITION_PIN_IG  3
#define IGNITION_PIN_BTN 4

typedef struct {
    bool acc_bat;
    bool acc_ig;
    bool st_bat;
    bool st_ig;
    bool button;
    uint8_t keyPos;
} KeyStateData;

// PCF8574 ignitionKeyPort(I2C_ADDRESS_IGNITION_PORT)
#endif
/**/