//#include "Joystick.h"
#include "JoystickS418.h"
#include "PCF8574.h"

bool debug = false;

int32_t forces[2] = {0};
Gains gains[2];
EffectParams effectparams[2];

#define ANALOG_OUT_MIN_VALUE 0
#define ANALOG_OUT_MAX_VALUE 1023

#define I2C_ADDRESS_STEERING_BLOCK  0x09
#define I2C_ADDRESS_GEARBOX_CUSTOM  0x10
#define I2C_ADDRESS_PEDALS_CUSTOM   0X11
#define I2C_ADDRESS_STICK           0X12
#define I2C_ADDRESS_HANDBRAKE       0X13

#define I2C_ADDRESS_BUTTON_BOX      0x20
#define I2C_ADDRESS_IGNITION_PORT   0x22
#define I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS1 0x6a
#define I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS2 0x6b

#define IGNITION_PIN_ACC    0
#define IGNITION_PIN_ST     1
#define IGNITION_PIN_BAT    2
#define IGNITION_PIN_IG     3
#define IGNITION_PIN_BTN    4
#define IGNITION_PIN_MODE   5

#define AXIS_STEERING       Axis::A_X
// #define AXIS_STEERING       Axis::S_STEERING // не работает FFb
#define AXIS_CLUTCH         Axis::A_Z
#define AXIS_BRAKE          Axis::A_RZ  //Axis::S_BRAKE
#define AXIS_ACCELERATOR    Axis::A_Y   //Axis::S_ACCELERATOR
#define AXIS_HANDBRAKE      Axis::A_RX
#define AXIS_STICK1_X       Axis::A_DIAL
#define AXIS_STICK1_Y       Axis::A_SLIDER

int32_t steeringValue = 0;

// кнопки 1-15 - стоковые на руле. DPad (Hat) - отдельно. Кнопка Prog не считается номерной кнопкой контроллера (т.е. служебная)
// кнопки 16-24 - кнопки КПП

S418::JoystickFfb::Joystick_ Joystick{};

bool ledState = false;

PCF8574 buttonBoxPort(I2C_ADDRESS_BUTTON_BOX);
PCF8574 ignitionKeyPort(I2C_ADDRESS_IGNITION_PORT);

typedef struct {
    bool acc_bat;
    bool acc_ig;
    bool st_bat;
    bool st_ig;
    bool button;
    uint8_t keyPos;
    
    bool igModeBoth;
} KeyStateData;

inline bool isBitSet(uint32_t state, uint8_t bit) {
 if (bit > 31) return false;
 return (state & (static_cast<uint32_t>(1) << bit)) != 0;
}

void setupTimerInterrupt() {
    cli();
    TCCR3A = 0; //set TCCR1A 0
    TCCR3B = 0; //set TCCR1B 0
    TCNT3 = 0; //counter init
    OCR3A = 399;
    TCCR3B |= (1 << WGM32); //open CTC mode
    TCCR3B |= (1 << CS31); //set CS11 1(8-fold Prescaler)
    TIMSK3 |= (1 << OCIE3A);
    sei();
}

ISR(TIMER3_COMPA_vect){
    Joystick.getUSBPID();
}

unsigned int interval = 0;

bool setInterval(unsigned int step = 1000) {
    static unsigned long lastTime = millis();
    unsigned long curTime = millis();

    bool become = (lastTime + step) <= curTime;

    if (become) {
        lastTime = curTime;
    }

    return become;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    setupJoystick();
    setupButtonBox1();
    setupIgnitionKey();
}

void setupJoystick()
{
    Joystick
            .hidReportId(JOYSTICK_DEFAULT_REPORT_ID)
            .joystickType(JOYSTICK_TYPE_JOYSTICK)
            .buttonCount(45)
            .hatSwitchCount(1)
            .includeAxis(AXIS_STEERING)      
            .includeAxis(AXIS_CLUTCH)   
            .includeAxis(Axis::S_BRAKE)     
            .includeAxis(Axis::S_ACCELERATOR)
            .includeAxis(AXIS_HANDBRAKE)
            .includeAxis(AXIS_STICK1_X)
            .includeAxis(AXIS_STICK1_Y)
            .init();

    // 0 - 100
    gains[0].totalGain = 100;
    gains[0].springGain = 100;
    gains[1].totalGain = 100;
    gains[1].springGain = 100;

    effectparams[0].springMaxPosition = 512;
    effectparams[1].springMaxPosition = 1023;

    Joystick.setGains(gains);
    Joystick.begin(true);

    setupTimerInterrupt();
}

void setupButtonBox1() {
    buttonBoxPort.pinMode(0, OUTPUT);
    buttonBoxPort.pinMode(1, OUTPUT);
    buttonBoxPort.pinMode(2, OUTPUT);
    buttonBoxPort.pinMode(3, OUTPUT);
    buttonBoxPort.pinMode(4, INPUT);
    buttonBoxPort.pinMode(5, INPUT);
    buttonBoxPort.pinMode(6, INPUT);
    buttonBoxPort.pinMode(7, INPUT);

    buttonBoxPort.begin();

    buttonBoxPort.digitalWrite(0, HIGH);
    buttonBoxPort.digitalWrite(1, HIGH);
    buttonBoxPort.digitalWrite(2, HIGH);
    buttonBoxPort.digitalWrite(3, HIGH);
}

void setupIgnitionKey() {
    ignitionKeyPort.pinMode(IGNITION_PIN_ACC, OUTPUT);
    ignitionKeyPort.pinMode(IGNITION_PIN_ST, OUTPUT);

    ignitionKeyPort.pinMode(IGNITION_PIN_BAT, INPUT_PULLUP);
    ignitionKeyPort.pinMode(IGNITION_PIN_IG, INPUT_PULLUP);

    ignitionKeyPort.pinMode(IGNITION_PIN_BTN, INPUT_PULLUP);
    ignitionKeyPort.pinMode(IGNITION_PIN_MODE, INPUT_PULLUP);

    ignitionKeyPort.begin();

    ignitionKeyPort.digitalWrite(IGNITION_PIN_ACC, HIGH);
    ignitionKeyPort.digitalWrite(IGNITION_PIN_ST, HIGH);
}

void loop() {
    processSteering();
    processPedals();
    processGearbox();
    processHandbrake();
    processSteeringButtons();
    processButtonBox();
    processIgnitionKeyState();
    processStick();
    processForces();
}

void processSteering()
{
    steeringValue = readSteeringValue();
    Joystick.setAxisValue(AXIS_STEERING, steeringValue);
}

int16_t readSteeringValue() {       
    Wire.requestFrom(I2C_ADDRESS_STEERING_BLOCK, 2);
    
    if (Wire.available() == 2) {
        uint16_t angle = Wire.read() | (Wire.read() << 8);

        return angle;
    }
    return -1;
}

void processPedals()
{
    Wire.requestFrom(I2C_ADDRESS_PEDALS_CUSTOM, 6);
    
    if (Wire.available() == 6) {
        Joystick.setAxisValue(AXIS_CLUTCH, (Wire.read() | (Wire.read() << 8)));
        Joystick.setAxisValue(AXIS_BRAKE, (Wire.read() | (Wire.read() << 8)));
        Joystick.setAxisValue(AXIS_ACCELERATOR, (Wire.read() | (Wire.read() << 8)));
    }
}

void processGearbox()
{
    sendGearboxButtons(readGearbox());
}

uint16_t readGearbox()
{
    Wire.requestFrom(I2C_ADDRESS_GEARBOX_CUSTOM, 1);
    
    if (Wire.available() == 1) {
        uint8_t gear = Wire.read();

        uint16_t result = 0;
        if (gear) {
            result = 1 << (gear - 1);
        }

        return result;
    }

    return 0;
}

void sendGearboxButtons(uint16_t states) {
    Joystick.setButton(16, isBitSet(states, 0));    // 1
    Joystick.setButton(17, isBitSet(states, 1));    // 2
    Joystick.setButton(18, isBitSet(states, 2));    // 3
    Joystick.setButton(19, isBitSet(states, 3));    // 4
    Joystick.setButton(20, isBitSet(states, 4));    // 5
    Joystick.setButton(21, isBitSet(states, 5));    // 6
    Joystick.setButton(22, isBitSet(states, 7));    // R

    // Joystick.setButton(15, isBitSet(states, 0));    // btn1
    // Joystick.setButton(23, isBitSet(states, 8));    // btn2
}

void processHandbrake()
{
    Wire.requestFrom(I2C_ADDRESS_HANDBRAKE, 4);

    if (Wire.available() == 4) {
        uint16_t handbrake = Wire.read() | (Wire.read() << 8);
        uint16_t buttons = Wire.read() | (Wire.read() << 8);
        
       Joystick.setAxisValue(AXIS_HANDBRAKE, handbrake);

        Joystick.setButton(23, isBitSet(buttons, 0)); 
    }
}

void processSteeringButtons() {
    sendSteeringButtons(readSteeringButtons());
}

uint32_t readSteeringButtons() {
    uint32_t result = 0;
    byte byte1, byte2, byte3;

    // Wire.beginTransmission(I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS1);
    // Wire.write(0x00);

    // if (Wire.endTransmission() != 0) {
    //     return result;
    // }

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

    bool dpadUp = isBitSet(states, 13);
    bool dpadRight = isBitSet(states, 14);
    bool dpadDown = isBitSet(states, 15);
    bool dpadLeft = isBitSet(states, 16);

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

uint16_t processButtonBox() {
    sendButtonBoxState(readButtonBox());
}

uint16_t readButtonBox()
{
    uint16_t buttonState = 0xFFFF;  // Инициализируем все биты в 1

    Wire.beginTransmission(I2C_ADDRESS_BUTTON_BOX);
    if (Wire.endTransmission() != 0) {
        return 0x0000;
    }

    byte readData;

    // Внешний цикл по строкам (0-3)
    for (uint8_t row = 0; row < 4; row++) {
        // Активируем текущую строку
        buttonBoxPort.digitalWrite(row, LOW);

        readData = buttonBoxPort.digitalReadAll() >> 4;

        // Инвертируем биты и применяем маску
        buttonState &= ~((unsigned int)((readData & 15) << (row * 4)));

        // Деактивируем строку
        buttonBoxPort.digitalWrite(row, HIGH);
    }

    return buttonState;
}
void sendButtonBoxState(uint16_t buttonStates)
{
    /*/
    for (byte buttonIterator = 0; buttonIterator < 16; buttonIterator++) {

        Joystick.setButton(
                buttonIterator + 24,
                isBitSet(buttonStates, buttonIterator)
        );
    }
    /**/
}

void processIgnitionKeyState() {
    sendIgnitionKeyState(readIgnitionKeyState());
}

KeyStateData readIgnitionKeyState() {
    KeyStateData data;

    ignitionKeyPort.digitalWrite(IGNITION_PIN_ACC, LOW);
    data.acc_bat = ignitionKeyPort.digitalRead(IGNITION_PIN_BAT, true);
    data.acc_ig = ignitionKeyPort.digitalRead(IGNITION_PIN_IG, true);

    ignitionKeyPort.digitalWrite(IGNITION_PIN_ACC, HIGH);
    ignitionKeyPort.digitalWrite(IGNITION_PIN_ST, LOW);
    data.st_bat = ignitionKeyPort.digitalRead(IGNITION_PIN_BAT, true);
    data.st_ig = ignitionKeyPort.digitalRead(IGNITION_PIN_IG, true);

    ignitionKeyPort.digitalWrite(IGNITION_PIN_ST, HIGH);

    uint8_t keyState = (!data.acc_bat << 0) |
                       (!data.acc_ig << 1) |
                       (!data.st_bat << 2) |
                       (!data.st_ig << 3);

    switch (keyState) {
        case 0b0001: data.keyPos = 1; break;
        case 0b0010: data.keyPos = 2; break;
        case 0b0011: data.keyPos = 3; break;
        case 0b1100: data.keyPos = 4; break;
        default: data.keyPos = 0; break;
    }

    data.button = ignitionKeyPort.digitalRead(IGNITION_PIN_BTN, true);
    data.igModeBoth = ignitionKeyPort.digitalRead(IGNITION_PIN_MODE, true);

    return data;
}

void sendIgnitionKeyState(KeyStateData state) {
    Joystick.setButton(24, state.keyPos == 1);
    Joystick.setButton(25, state.keyPos == 2);
    Joystick.setButton(26, state.igModeBoth ? (state.keyPos >= 3) : (state.keyPos == 3));
    Joystick.setButton(27, state.keyPos == 4);
    Joystick.setButton(28, state.button == 0);
}

void processStick()
{
    Wire.requestFrom(I2C_ADDRESS_STICK, 6);

    if (Wire.available() == 6) {
        Joystick.setAxisValue(AXIS_STICK1_X, (Wire.read() | (Wire.read() << 8)));
        Joystick.setAxisValue(AXIS_STICK1_Y, (Wire.read() | (Wire.read() << 8)));

        uint16_t buttons = Wire.read() | (Wire.read() << 8);
    }
}

void processForces()
{
    effectparams[0].springPosition = steeringValue - 512;
    // effectparams[1].springPosition = brakeFilteredValue;

    Joystick.setEffectParams(effectparams);
    Joystick.getForce(forces);

    sendForce(forces[0]);
}

// -255 .. 255
void sendForce(int32_t ffbValue) {
    Wire.beginTransmission(I2C_ADDRESS_STEERING_BLOCK);
    if (debug && setInterval(250)) {
        Serial.print("sending ffb value: ");
        Serial.println(ffbValue);
    }
    Wire.write(ffbValue & 0xFF);        // младший байт
    Wire.write((ffbValue >> 8) & 0xFF); // старший байт
    Wire.endTransmission();
}