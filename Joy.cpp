#include "Joy.h"

namespace S418 {
    namespace Steering {
        Joy::Joy() {

        }

        Joy::Joy(Joystick_& joystick) {
            this->joystick = &joystick;
        }

        void Joy::setup() {
            setupTimerInterrupt();

            this->joystick->setXAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
            this->joystick->setYAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
            this->joystick->setZAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
            this->joystick->setRxAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
//            this->joystick->setRyAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
            this->joystick->setRzAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);

            // 0 - 100
            gains[0].totalGain = 100;
            gains[0].springGain = 100;
            gains[1].totalGain = 100;
            gains[1].springGain = 100;
            effectParams[0].springMaxPosition = 512;
            effectParams[1].springMaxPosition = 1023;

            this->joystick->setGains(gains);

            this->joystick->begin(true);
        }

//        void Joy::processForces() {
//            effectParams[0].springPosition = steeringFilteredValue - 512;
//            effectParams[1].springPosition = brakeFilteredValue;
//
//            joystick.setEffectParams(effectparams);
//            joystick.getForce(forces);
//
//            sendForce(forces[0]);
//        }

        void Joy::setupTimerInterrupt() {
            cli();
            TCCR3A = 0;              //set TCCR1A 0
            TCCR3B = 0;              //set TCCR1B 0
            TCNT3 = 0;               //counter init
            OCR3A = 399;
            TCCR3B |= (1 << WGM32);  //open CTC mode
            TCCR3B |= (1 << CS31);   //set CS11 1(8-fold Prescaler)
            TIMSK3 |= (1 << OCIE3A);
            sei();
        }

        void Joy::refreshForceValues() {
//            effectParams[0].springPosition = steeringFilteredValue - 512;
//            effectParams[1].springPosition = brakeFilteredValue;
            effectParams[0].springPosition = 0;
            effectParams[1].springPosition = 0;

            this->joystick->setEffectParams(effectParams);
            this->joystick->getForce(forces);
        }
    }
}
