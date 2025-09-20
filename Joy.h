
#ifndef JOY_H
#define JOY_H
#include "Joystick.h"
#include "Common.h"

namespace S418 {
    namespace Steering {
        class Joy {
            public:
                Joystick_* joystick;

                Joy();
                Joy(Joystick_* joystick);
                void setup();
                void processForces();
            private:
                void setupTimerInterrupt();
        };
    }
}

#endif
