/*/
#ifndef STEERING_H
#define STEERING_H

#include "MovingAverage.h"

#define I2C_ADDRESS_STEERING_BLOCK 0x09
#define I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS1 0x6a
#define I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS2 0x6b


namespace S418 {
    namespace Steering {
        class Steering {
        public:
            uint32_t steeringValue;
            Steering(Joystick_ *joystick);
        private:
            uint32_t steeringActualValue;
            uint32_t steeringScaledValue;

            MovingAverage steeringAverageFilter;
        };
    }
}

#endif
 /**/