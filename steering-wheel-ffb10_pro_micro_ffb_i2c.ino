#include "Arduino.h"
#include "Wire.h"
#include "Common.h"
#include "Joy.h"
//#include "Steering.h"
//#include "Steering.h"
//#include "Pedals.h"
//#include "Gearbox.h"
//#include "Handbrake.h"
//#include "Ignition.h"
//#include "Buttonbox.h"

// кнопки 1-15 - стоковые на руле. DPad (Hat) - отдельно. Кнопка Prog не считается номерной кнопкой контроллера (т.е. служебная)
// кнопки 16-24 - кнопки КПП
// кнопки 25-40 - кнопки ButtonBox1
// кнопки 41-45 - кнопки Ignition

Joystick_ joystick;
S418::Steering::Joy joy;
//S418::Steering::Pedals pedals;
//S418::Steering::Gearbox gearbox;
//S418::Steering::Handbrake handbrake;
//S418::Steering::Ignition ignition;
//S418::Steering::Buttonbox buttonbox;

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

    joystick = new Joystick_(
            JOYSTICK_DEFAULT_REPORT_ID,
            JOYSTICK_TYPE_JOYSTICK, // JOYSTICK_TYPE_MULTI_AXIS
            45, 1,                   // Button Count, Hat Switch Count
            true, true, true,        // X Y Z // STEERING CLUTCH BRAKE
            true, false,  true,      // Rx Ry Rz // ACCEL ? HANDBRAKE
            false, false,            // rudder throttle
            false, false, false      // accelerator brake steering
    );
    joy = new Joy(joystick);
//    steering = new Steering(joystick);
//    pedals = new Pedals(joystick);
//    gearbox = new Gearbox(joystick);
//    handbrake = new Handbrake(joystick);
//    ignition = new Ignition(joystick);
//    buttonbox = new Buttonbox(joystick);

//    joy.setup();
//    steering.setup();
//    pedals.setup();
//    gearbox.setup();
//    handbrake.setup();
//    ignition.setup();
//    buttonbox.setup();
}

void loop() {
//    steering.process();
//    pedals.process();
//    gearbox.process();
//    handbrake.process();
//    ignition.process();
//    buttonbox.process();
//    joy.processForces();
//    processSteeringButtons();
//    processButtonBox();
//    processIgnitionKeyState();
}