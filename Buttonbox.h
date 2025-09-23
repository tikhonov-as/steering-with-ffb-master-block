#ifndef BUTTONBOX_H
#define BUTTONBOX_H

#include "Arduino.h"
#include "Joystick.h"
#include "Common.h"
#include "PCF8574.h"

#define I2C_ADDRESS_BUTTON_BOX PCF8574_ADDRESS_0
// PCF8574 buttonBoxPort(I2C_ADDRESS_BUTTON_BOX);

namespace S418 {
    namespace Steering {
        class Buttonbox {
        public:
            Joystick_* joystick;

            Buttonbox();
            Buttonbox(Joystick_ *joystick);
            void setup();

        private:
        };
    }
}
#endif
