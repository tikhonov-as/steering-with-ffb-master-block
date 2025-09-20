#include "Handbrake.h"

// handbrakeAverageFilter = new MovingAverage(MOVING_AVERAGE_SIZE);
/*/
setup
 //    pinMode(ANALOG_INPUT_HANDBRAKE, INPUT);

 Handbrake ::process()
 {
     handbrakeActualValue = analogRead(ANALOG_INPUT_HANDBRAKE);
    handbrakeScaledValue = constrain(map(handbrakeActualValue, HANDBRAKE_MIN_VALUE, HANDBRAKE_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    handbrakeFilteredValue = handbrakeAverageFilter.update(handbrakeScaledValue).getAverage();
    joystick.setRxAxis(handbrakeFilteredValue);

 }

/**/