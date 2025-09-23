
#ifndef GEARBOX_H
#define GEARBOX_H

#include "Arduino.h"
#include "Joystick.h"
#include "Common.h"

#define I2C_ADDRESS_GEARBOX 0x63

namespace S418 {
    namespace Steering {
        class Gearbox {
        public:
            Joystick_* joystick;

            Gearbox();
            Gearbox(Joystick_ *joystick);
            void setup();

        private:

        };
    }
}

#endif