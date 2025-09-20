//#include "Pedals.h"

/*/

 pedals.process();

     // int* pedalsValues = readAds1115Channels(adcPedals);
    /*
    clutchActualValue = pedalsValues[0];
    clutchScaledValue = constrain(map(clutchActualValue, CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    clutchFilteredValue = clutchAverageFilter.update(clutchScaledValue).getAverage();
    joystick.setZAxis(clutchFilteredValue);

    brakeActualValue = pedalsValues[1];
    brakeScaledValue = constrain(map(brakeActualValue, BRAKE_MIN_VALUE, BRAKE_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    brakeFilteredValue = brakeAverageFilter.update(brakeScaledValue).getAverage();
    joystick.setRzAxis(brakeFilteredValue);

    accelActualValue = pedalsValues[2];
    accelScaledValue = constrain(map(accelActualValue, ACCEL_MIN_VALUE, ACCEL_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    accelFilteredValue = accelAverageFilter.update(accelScaledValue).getAverage();
    joystick.setYAxis(accelFilteredValue);


/**/


/*/
namespace S418 {
    namespace Steering {
        Pedals::Pedals() {
            ads = ADS1115_WE(I2C_ADDRESS_PEDALS);

            clutchAverageFilter = new MovingAverage(MOVING_AVERAGE_SIZE);
            brakeAverageFilter = new MovingAverage(MOVING_AVERAGE_SIZE);
            accelAverageFilter = new MovingAverage(MOVING_AVERAGE_SIZE);
        }

        void Pedals::setup() {
            clutchActualValue = 0;
            clutchScaledValue = 0;
            clutchFilteredValue = 0;
            brakeActualValue = 0;
            brakeScaledValue = 0;
            brakeFilteredValue = 0;
            accelActualValue = 0;
            accelScaledValue = 0;
            accelFilteredValue = 0;
            if (ads.init()) {
                ads.setVoltageRange_mV(ADS1115_RANGE_6144);
                ads.setCompareChannels(ADS1115_COMP_0_GND);
                ads.setMeasureMode(ADS1115_CONTINUOUS);
                ads.setConvRate(ADS1115_250_SPS); // def 128
            }
        }

        void Pedals::process() {
          read();
        }

        void Pedals::read() {
            // clutchValue = readAds1115ChannelRaw(ads, ADS1115_COMP_0_GND) >> ADS1115_NONSIGNIFICANT_BITS;
            // brakeValue = readAds1115ChannelRaw(ads, ADS1115_COMP_1_GND) >> ADS1115_NONSIGNIFICANT_BITS;
            // accelerationValue = readAds1115ChannelRaw(ads, ADS1115_COMP_2_GND) >> ADS1115_NONSIGNIFICANT_BITS;

            int* pedalsValues = readAds1115Channels(adcPedals);    
            clutchActualValue = pedalsValues[0];
            clutchScaledValue = constrain(map(clutchActualValue, CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
            clutchFilteredValue = clutchAverageFilter.update(clutchScaledValue).getAverage();
            Joystick.setZAxis(clutchFilteredValue);

            brakeActualValue = pedalsValues[1];
            brakeScaledValue = constrain(map(brakeActualValue, BRAKE_MIN_VALUE, BRAKE_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
            brakeFilteredValue = brakeAverageFilter.update(brakeScaledValue).getAverage();
            Joystick.setRzAxis(brakeFilteredValue);

            accelActualValue = pedalsValues[2];
            accelScaledValue = constrain(map(accelActualValue, ACCEL_MIN_VALUE, ACCEL_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
            accelFilteredValue = accelAverageFilter.update(accelScaledValue).getAverage();
            Joystick.setYAxis(accelFilteredValue);

            handbrakeActualValue = analogRead(ANALOG_INPUT_HANDBRAKE);
            handbrakeScaledValue = constrain(map(handbrakeActualValue, HANDBRAKE_MIN_VALUE, HANDBRAKE_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
            handbrakeFilteredValue = handbrakeAverageFilter.update(handbrakeScaledValue).getAverage();
            Joystick.setRxAxis(handbrakeFilteredValue);

        }

        uint16_t Pedals::getClutchValue() {
            return clutchValue;
        }

        uint16_t Pedals::getBrakeValue() {
            return brakeValue;
        }

        uint16_t Pedals::getAccelerationValue() {
            return accelerationValue;
        }

        uint16_t* Pedals::readAds1115Channels() {
            static uint16_t values[3];

            values[0] = readAds1115ChannelRaw(ADS1115_COMP_0_GND) >> ADS1115_NONSIGNIFICANT_BITS;
            values[1] = readAds1115ChannelRaw(ADS1115_COMP_1_GND) >> ADS1115_NONSIGNIFICANT_BITS;
            values[2] = readAds1115ChannelRaw(ADS1115_COMP_2_GND) >> ADS1115_NONSIGNIFICANT_BITS;
            
            return values;
        }

        uint16_t Pedals::readAds1115ChannelRaw(ADS1115_MUX channel) {
            ads.setCompareChannels(channel);
            return ads.getRawResult();
        }
    }
}
/**/