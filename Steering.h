#ifndef STEERING_H
#define STEERING_H

#include "Arduino.h"
#include "Joystick.h"
#include "Common.h"
#include "MovingAverage.h"
#include "Wire.h"

#define I2C_ADDRESS_STEERING_BLOCK 0x09
#define I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS1 0x6a
#define I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS2 0x6b

namespace S418 {
    namespace Steering {
        class Steering {
        public:
            Joystick_* joystick;

//            uint16_t steeringValue;

            Steering();
            Steering(Joystick_& joystick);
            void setup();
            void processInputOperations();

        private:
            int16_t readSteeringAngle();

            MovingAverage steeringAverageFilter;
        };
    }
}

#endif