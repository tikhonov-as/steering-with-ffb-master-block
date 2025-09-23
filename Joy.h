#ifndef JOY_H
#define JOY_H

#include "Arduino.h"
#include "Joystick.h"
#include "Common.h"

namespace S418 {
    namespace Steering {
        class Joy {
            public:
                Joystick_* joystick;

                Joy();
                Joy(Joystick_& joystick);

                void setup();
                void refreshForceValues();
                int32_t* getForces();
            private:
                Gains gains[2];
                EffectParams effectParams[2];
                int32_t forces[2] = {0};

                void setupTimerInterrupt();
        };
    }
}

#endif