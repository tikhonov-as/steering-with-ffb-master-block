#include "Arduino.h"
#include "Wire.h"
#include "Joystick.h"
#include "Common.h"
//#include "Joy.h"
#include "Steering.h"
//#include "Pedals.h"
//#include "Gearbox.h"
//#include "Handbrake.h"
//#include "Ignition.h"
//#include "Buttonbox.h"

// кнопки 1-15 - стоковые на руле. DPad (Hat) - отдельно. Кнопка Prog не считается номерной кнопкой контроллера (т.е. служебная)
// кнопки 16-24 - кнопки КПП
// кнопки 25-40 - кнопки ButtonBox1
// кнопки 41-45 - кнопки Ignition

Joystick_ joystick(
        JOYSTICK_DEFAULT_REPORT_ID,
        JOYSTICK_TYPE_JOYSTICK, // JOYSTICK_TYPE_MULTI_AXIS
        45, 1,                   // Button Count, Hat Switch Count
        true, true, true,        // X Y Z // STEERING CLUTCH BRAKE
        true, false,  true,      // Rx Ry Rz // ACCEL ? HANDBRAKE
        false, false,            // rudder throttle
        false, false, false      // accelerator brake steering
);

int32_t forces[2] = {0};
Gains gains[2];
EffectParams effectParams[2];

//S418::Steering::Joy joy(joystick);
S418::Steering::Steering steering(joystick);
//S418::Steering::Pedals* pedals;
//S418::Steering::Gearbox* gearbox; 
//S418::Steering::Handbrake* handbrake;
//S418::Steering::Ignition* ignition;
//S418::Steering::Buttonbox* buttonbox;

ISR(TIMER3_COMPA_vect){
    joystick.getUSBPID();
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    // steering
        // forces
        // buttons
    // pedals
    // gearbox
    // handbrake
    // ignition
    // buttonbox

//    joystick = Joystick_(
//            JOYSTICK_DEFAULT_REPORT_ID,
//            JOYSTICK_TYPE_JOYSTICK, // JOYSTICK_TYPE_MULTI_AXIS
//            45, 1,                   // Button Count, Hat Switch Count
//            true, true, true,        // X Y Z // STEERING CLUTCH BRAKE
//            true, false,  true,      // Rx Ry Rz // ACCEL ? HANDBRAKE
//            false, false,            // rudder throttle
//            false, false, false      // accelerator brake steering
//    );

//---------------------------------------------------------
// лишнее
//    joy = S418::Steering::Joy(&joystick);
//    steering = new S418::Steering::Steering(&joystick);
//    pedals = new S418::Steering::Pedals(joystick);
//    gearbox = new S418::Steering::Gearbox(joystick);
//    handbrake = new S418::Steering::Handbrake(joystick);
//    ignition = new S418::Steering::Ignition(joystick);
//    buttonbox = new S418::Steering::Buttonbox(joystick);
//---------------------------------------------------------


    joystick.setXAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    joystick.setYAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    joystick.setZAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    joystick.setRxAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    joystick.setRyAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    joystick.setRzAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);

    // 0 - 100
    gains[0].totalGain = 100;
    gains[0].springGain = 100;
    gains[1].totalGain = 100;
    gains[1].springGain = 100;
    effectParams[0].springMaxPosition = 512;
    effectParams[1].springMaxPosition = 1023;

    joystick.setGains(gains);

    joystick.begin(true);

    setupTimerInterrupt();

//    joy.setup();
    steering.setup();
//    pedals->setup();
//    gearbox->setup();
//    handbrake->setup();
//    ignition->setup();
//    buttonbox->setup();
}

void setupTimerInterrupt() {
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

void loop() {
    // 1. обработка операций ввода
    // 2. вычисление forces
    // 3. обработка операций вывода

    steering.processInputOperations();
//    pedals->processInputOperations();
//    gearbox->processInputOperations();
//    handbrake->processInputOperations();
//    ignition->processInputOperations();
//    buttonbox->processInputOperations();

//    joy.refreshForceValues();

//    steering->sendForces(joy->getForces());

    effectParams[0].springMaxPosition = 512;
//    effectParams[0].springPosition = steeringFilteredValue - 512;
    effectParams[0].springPosition = 512 - 512;
    effectParams[1].springMaxPosition = 1023;
//    effectParams[1].springPosition = brakeFilteredValue;
    effectParams[1].springPosition = 0;

    joystick.setEffectParams(effectParams);
    joystick.getForce(forces);

    sendForce(forces[0]);
}

void sendForce(int32_t ffbValue) {
    Wire.beginTransmission(I2C_ADDRESS_STEERING_BLOCK);
    Wire.write(ffbValue & 0xFF);        // младший байт
    Wire.write((ffbValue >> 8) & 0xFF); // старший байт
    Wire.endTransmission();
}
