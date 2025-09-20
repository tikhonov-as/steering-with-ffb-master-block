#include "Gearbox.h"

/*/

void sendGearboxButtons(uint32_t states) {
    // todo
    Joystick.setButton(15, isBitSet(states, 0));    // btn1
    Joystick.setButton(16, isBitSet(states, 1));    // 1
    Joystick.setButton(17, isBitSet(states, 2));    // 2
    Joystick.setButton(18, isBitSet(states, 3));    // 3
    Joystick.setButton(19, isBitSet(states, 4));    // 4
    Joystick.setButton(20, isBitSet(states, 5));    // 5
    Joystick.setButton(21, isBitSet(states, 6));    // 6
    Joystick.setButton(22, isBitSet(states, 7));    // R
    Joystick.setButton(23, isBitSet(states, 7));    // btn2
}

/**/