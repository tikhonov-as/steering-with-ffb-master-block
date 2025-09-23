#include "Steering.h"

namespace S418 {
    namespace Steering {
        Steering::Steering() {

        }

        Steering::Steering(Joystick_& joystick) {
            this->joystick = &joystick;
        }

        void Steering::setup() {

        }

        /**
         * Операции чтения
         */
        void Steering::processInputOperations() {

            int16_t steeringActualValue = readSteeringAngle();
            //Serial.println(steeringActualValue);
            if (steeringActualValue == -1) {
                return;
            }
            uint16_t steeringFilteredValue = steeringAverageFilter.update(steeringActualValue).getAverage();

            joystick->setXAxis(steeringFilteredValue);
        }

        int16_t Steering::readSteeringAngle() {
            Wire.beginTransmission(I2C_ADDRESS_STEERING_BLOCK);
            Wire.endTransmission();

            Wire.requestFrom(I2C_ADDRESS_STEERING_BLOCK, 2);  // Запрашиваем 2 байта (uint16_t)
//            Serial.print("b ");
//            Serial.println(Wire.available());
            while (Wire.available() < 2);
            if (Wire.available() == 2) {
                uint16_t angle = Wire.read() | (Wire.read() << 8);

//                Serial.print("a ");
//                Serial.println(angle);

                return angle;
            }
            return -1;
        }
    }
}

// steeringAverageFilter = new MovingAverage(MOVING_AVERAGE_SIZE);
/*/

int32_t forces[2] = {0};
Gains gains[2];
EffectParams effectparams[2];


 // -255 .. 255
void sendForce(int32_t ffbValue) {
    Wire.beginTransmission(I2C_ADDRESS_STEERING_BLOCK);
    Wire.write(ffbValue & 0xFF);        // младший байт
    Wire.write((ffbValue >> 8) & 0xFF); // старший байт
    Wire.endTransmission();
}

 void processSteeringButtons() {
    sendSteeringButtons(readSteeringButons());
}

uint32_t readSteeringButons() {
    uint32_t result = 0;
    byte byte1, byte2, byte3;

    Wire.beginTransmission(I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS1);
    Wire.write(0x00);

    if (Wire.endTransmission() != 0) {
        return result;
    }

    Wire.requestFrom(I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS1, 3);

    if (Wire.available() < 3) {
        return result;
    }

    byte1 = Wire.read();
    byte2 = Wire.read();
    byte3 = Wire.read();

    // Объединяем биты из трех байтов в одно 20-битное значение
    result =
        // Байты читаются слева направо, биты объединяются от младших к старшим
        // byte1 занимает биты 0-7
        (static_cast<uint32_t>(byte1) & 0xFF) |
        // byte2 занимает биты 8-12 (берем только 5 бит)
        ((static_cast<uint32_t>(byte2 & 0x1F) << 8)) |
        // byte3 занимает биты 13-19 (берем 7 бит)
        ((static_cast<uint32_t>(byte3 & 0x7F) << 13));

    return result;
}

void sendSteeringButtons(uint32_t states) {
    byte hatSwitch = 0;

    bool dpadUp = isBitSet(states, 13);     // (states & (1 << 13)) != 0;
    bool dpadRight = isBitSet(states, 14);  // (states & (1 << 14)) != 0;
    bool dpadDown = isBitSet(states, 15);   // (states & (1 << 15)) != 0;
    bool dpadLeft = isBitSet(states, 16);   // (states & (1 << 16)) != 0;

    if (!(dpadUp || dpadRight || dpadDown || dpadLeft)) {
        Joystick.setHatSwitch(hatSwitch, JOYSTICK_HATSWITCH_RELEASE);
    } else if (dpadUp && dpadRight) {
        Joystick.setHatSwitch(hatSwitch, 45);
    } else if (dpadRight && dpadDown) {
        Joystick.setHatSwitch(hatSwitch, 135);
    } else if (dpadDown && dpadLeft) {
        Joystick.setHatSwitch(hatSwitch, 225);
    } else if (dpadLeft && dpadUp) {
        Joystick.setHatSwitch(hatSwitch, 315);
    } else if (dpadUp) {
        Joystick.setHatSwitch(hatSwitch, 0);
    } else if (dpadRight) {
        Joystick.setHatSwitch(hatSwitch, 90);
    } else if (dpadDown) {
        Joystick.setHatSwitch(hatSwitch, 180);
    } else if (dpadLeft) {
        Joystick.setHatSwitch(hatSwitch, 270);
    }
    Joystick.setButton(0, isBitSet(states, 0));     // 1
    Joystick.setButton(1, isBitSet(states, 1));     // 2
    Joystick.setButton(2, isBitSet(states, 2));     // 3
    Joystick.setButton(3, isBitSet(states, 3));     // 4
    Joystick.setButton(4, isBitSet(states, 4));     // L1
    Joystick.setButton(5, isBitSet(states, 5));     // R1
    Joystick.setButton(6, isBitSet(states, 6));     // L2
    Joystick.setButton(7, isBitSet(states, 7));     // R2
    Joystick.setButton(8, isBitSet(states, 8));     // share
    Joystick.setButton(9, isBitSet(states, 9));     // options
    Joystick.setButton(10, isBitSet(states, 10));   // L3
    Joystick.setButton(11, isBitSet(states, 11));   // R3
    Joystick.setButton(12, isBitSet(states, 12));   // mode
    Joystick.setButton(13, isBitSet(states, 17));   // gear -
    Joystick.setButton(14, isBitSet(states, 18));   // gear +
}


 void Steering::process()
 {
    steeringActualValue = readSteeringValue();
    steeringScaledValue = steeringActualValue;
    steeringFilteredValue = steeringAverageFilter.update(steeringScaledValue).getAverage();
    joystick.setXAxis(steeringFilteredValue);
}

 void setForce()
 {

 }

/**/