#ifndef HANDBRAKE_H
#define HANDBRAKE_H

#include "Arduino.h"
#include "Joystick.h"
#include "Common.h"
#include "MovingAverage.h"

#define HANDBRAKE_MIN_VALUE 492
#define HANDBRAKE_MAX_VALUE 780
#define ANALOG_INPUT_HANDBRAKE A9 // Rx

namespace S418 {
    namespace Steering {
        class Handbrake {
        public:
            Joystick_* joystick;
//            uint32_t handbrakeValue;

            Handbrake();
            Handbrake(Joystick_ *joystick);
            void setup();

        private:
//            uint32_t handbrakeActualValue;
//            uint32_t handbrakeScaledValue;

//            MovingAverage handbrakeAverageFilter;
        };
    }
}

#endif