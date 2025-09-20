/*/
#ifndef PEDALS_H
#define PEDALS_H

#include "Arduino.h"
#include "ADS1115_WE.h"
#include "MovingAverage.h"

#define I2C_ADDRESS_PEDALS ADS1115_ADDRESS_VDD

#define CLUTCH_MIN_VALUE    840     // 570 // старый блок педалей - 309
#define CLUTCH_MAX_VALUE    112     // 112 // старый блок педалей - 511
#define BRAKE_MIN_VALUE     832     // 578 // старый блок педалей - 160
#define BRAKE_MAX_VALUE     40      // 25  // старый блок педалей - 395
#define ACCEL_MIN_VALUE     981     // 680 // старый блок педалей - 207
#define ACCEL_MAX_VALUE     560     // 339 // старый блок педалей - 511

namespace S418 {
    namespace Steering {
        class Pedals {
        public:
            Pedals();
            void setup();
            void process();
            uint16_t getClutchValue();
            uint16_t getBrakeValue();
            uint16_t getAccelerationValue();
            
        private:
            ADS1115_WE ads;

            uint16_t clutchActualValue;
            uint16_t clutchScaledValue;

            uint16_t brakeActualValue;
            uint16_t brakeScaledValue;

            uint16_t accelActualValue;
            uint16_t accelScaledValue;
            uint16_t accelFilteredValue;

            uint16_t clutchValue;
            uint16_t brakeValue;
            uint16_t accelerationValue;

            MovingAverage clutchAverageFilter;
            MovingAverage brakeAverageFilter;
            MovingAverage accelAverageFilter;

            void read();
            void send();

            uint16_t* readAds1115Channels();
            uint16_t readAds1115ChannelRaw(ADS1115_MUX channel);
    }
}

#endif
/**/