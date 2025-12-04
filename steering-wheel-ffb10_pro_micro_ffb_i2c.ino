//#include "Joystick.h"
#include "JoystickS418.h"

#include "PCF8574.h"
#include "ADS1115_WE.h"
#include "MovingAverage.h"

bool debug = false;

int32_t forces[2] = {0};
Gains gains[2];
EffectParams effectparams[2];

//#define ANALOG_INPUT_HANDBRAKE A9 // Rx

// это прочитанные значения с ADC
//#define HANDBRAKE_MIN_VALUE 492
//#define HANDBRAKE_MAX_VALUE 780
// а это значения, на которые мапим - которые будут высылаться на ПК
#define ANALOG_OUT_MIN_VALUE 0
#define ANALOG_OUT_MAX_VALUE 1023

#define PCF8574_ADDRESS_0 0x20
#define PCF8574_ADDRESS_1 0x21
#define PCF8574_ADDRESS_2 0x22
#define PCF8574_ADDRESS_3 0x23
#define PCF8574_ADDRESS_4 0x24
#define PCF8574_ADDRESS_5 0x25
#define PCF8574_ADDRESS_6 0x26
#define PCF8574_ADDRESS_7 0x27

// #define PCA9685_ADDRESS_0  0x40
// #define PCA9685_ADDRESS_1  0x41
// #define PCA9685_ADDRESS_2  0x42
// #define PCA9685_ADDRESS_3  0x43
// #define PCA9685_ADDRESS_4  0x44
// #define PCA9685_ADDRESS_5  0x45
// #define PCA9685_ADDRESS_6  0x46
// #define PCA9685_ADDRESS_7  0x47
// #define PCA9685_ADDRESS_8  0x48
// #define PCA9685_ADDRESS_9  0x49
// #define PCA9685_ADDRESS_10 0x4A
// #define PCA9685_ADDRESS_11 0x4B
// #define PCA9685_ADDRESS_12 0x4C
// #define PCA9685_ADDRESS_13 0x4D
// #define PCA9685_ADDRESS_14 0x4E
// #define PCA9685_ADDRESS_15 0x4F
// #define PCA9685_ADDRESS_ALL_CALL 0x70

#define ADS1115_ADDRESS_GND 0x48
#define ADS1115_ADDRESS_VDD 0x49
#define ADS1115_ADDRESS_SDA 0x4a
#define ADS1115_ADDRESS_SCL 0x4b // при соединении напрямую - подсаживает линию SCL. работает только с короткким проводом, длиннее - виснет
#define ADS1115_NONSIGNIFICANT_BITS 4

#define I2C_ADDRESS_STEERING_BLOCK 0x09
#define I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS1 0x6a
#define I2C_ADDRESS_STEERING_BLOCK_WHEEL_BUTTONS2 0x6b
#define I2C_ADDRESS_GEARBOX_CUSTOM 0x10
#define I2C_ADDRESS_PEDALS_CUSTOM 0X11
#define I2C_ADDRESS_PEDALS ADS1115_ADDRESS_VDD
#define I2C_ADDRESS_BUTTON_BOX PCF8574_ADDRESS_0
#define I2C_ADDRESS_STICK 0X12
#define I2C_ADDRESS_HANDBRAKE 0X13
#define I2C_ADDRESS_IGNITION_PORT PCF8574_ADDRESS_2

#define IGNTION_COMBINED_IGNITION_AND_STARTER_MODE 1

#define IGNITION_PIN_ACC    0
#define IGNITION_PIN_ST     1
#define IGNITION_PIN_BAT    2
#define IGNITION_PIN_IG     3
#define IGNITION_PIN_BTN    4
#define IGNITION_PIN_MODE   5

int32_t steeringActualValue = 0;
int32_t steeringScaledValue = 0;
int32_t steeringFilteredValue = 0;
int32_t handbrakeActualValue = 0;
int32_t handbrakeScaledValue = 0;
int32_t handbrakeFilteredValue = 0;

#define MOVING_AVERAGE_SIZE 5
MovingAverage steeringAverageFilter(MOVING_AVERAGE_SIZE);
MovingAverage handbrakeAverageFilter(MOVING_AVERAGE_SIZE);

ADS1115_WE adcPedals = ADS1115_WE(I2C_ADDRESS_PEDALS);

// кнопки 1-15 - стоковые на руле. DPad (Hat) - отдельно. Кнопка Prog не считается номерной кнопкой контроллера (т.е. служебная)
// кнопки 16-24 - кнопки КПП
// кнопки 25-40 - кнопки ButtonBox1
// кнопки 41-45 - кнопки Ignition

/*/
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, // // JOYSTICK_TYPE_MULTI_AXIS
                   45, 1,                 // Button Count, Hat Switch Count
                   true, true, true,      // X Y Z // STEERING CLUTCH BRAKE
                   true, true,  true,     // Rx Ry Rz // ACCEL ? HANDBRAKE
                   false, false,          // rudder throttle
                   false, false, false);  // accelerator brake steering
/**/

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

int16_t readAds1115ChannelRaw(ADS1115_WE adc, ADS1115_MUX channel) {
    adc.setCompareChannels(channel);
    return adc.getRawResult();
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

int16_t readSteeringValue() {
    Wire.beginTransmission(I2C_ADDRESS_STEERING_BLOCK);
    Wire.endTransmission();
    
    Wire.requestFrom(I2C_ADDRESS_STEERING_BLOCK, 2);  // Запрашиваем 2 байта (uint16_t)
    
    if (Wire.available() == 2) {
        uint16_t angle = Wire.read() | (Wire.read() << 8);
        if (debug && setInterval(1000)) {
            Serial.print("Получен угол: ");
            Serial.println(angle);
        }

        return angle;
    }
    return -1;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    setupJoystick();
    setupGearbox();
    setupHandbrake();
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
// sim controls
            .includeSteering(true)      // Так же определяется как X
            .includeAccelerator(true)   // Так же определяется как Y
            .includeZAxis(true)         // clutch
            .includeBrake(true)         // Так же определяется как Rz
            .includeRxAxis(true)        // handbrake
            

// axes
            .includeVx(false)           // не распознается
            .includeVy(false)           // не распознается

            .includeSlider(true)
            .includeDial(true)


//other axes
            .includeXAxis(false)        // занято как Steering 
            .includeYAxis(false)        // занято как Accelerator
            .includeRzAxis(false)       // занято как Brake

            .includeRyAxis(false)

            .includeClutch(false)       // не распознается
            .includeHandbrake(false)    // не распознается

            .includeWheel(false)
            .includeVz(false)           // не распознается - судя по vx vy
            .includeVbrx(false)
            .includeVbry(false)
            .includeVbrz(false)

            .includeAx(false)            // вешает USB
            .includeAy(false)            // вешает USB
            .includeAz(false)


            .includeAbrrx(false)
            .includeAbrry(false)
            .includeAbrrz(false)
            .includeForcex(false)
            .includeForcey(false)
            .includeForcez(false)
            .includeTorquex(false)
            .includeTorquey(false)
            .includeTorquez(false)
//other sim controls
            .includeYaw(false)          // не распознается
            .includePitch(false)        // не распознается
            
            .includeRoll(false)          // один не определяется
            .includeRudder(false)        // второй как Rz

            .includeThrottle(true)
            .includeTurretx(false)
            .includeTurrety(false)
            .includeTurretz(false)

            .init();

    Joystick.setXAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setYAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setZAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setRxAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setRyAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setRzAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);

    Joystick.setWheelRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setVxRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setVyRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setVzRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setVbrxRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setVbryRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setVbrzRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setAxRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setAyRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setAzRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setAbrrxRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setAbrryRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setAbrrzRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setForcexRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setForceyRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setForcezRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setTorquexRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setTorqueyRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setTorquezRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);

    Joystick.setYawRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setPitchRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setRollRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setRudderRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setThrottleRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setTurretxRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setTurretyRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setTurretzRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);

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

void setupGearbox()
{
}

void setupHandbrake()
{
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
    steeringActualValue = readSteeringValue();
    steeringScaledValue = steeringActualValue;
    steeringFilteredValue = steeringAverageFilter.update(steeringScaledValue).getAverage();
    Joystick.setSteering(steeringFilteredValue);
}

void processPedals()
{
    Wire.beginTransmission(I2C_ADDRESS_PEDALS_CUSTOM);
    Wire.endTransmission();
    
    Wire.requestFrom(I2C_ADDRESS_PEDALS_CUSTOM, 6);
    
    if (Wire.available() == 6) {
        uint16_t clutchValue = Wire.read() | (Wire.read() << 8);
        uint16_t brakeValue = Wire.read() | (Wire.read() << 8);
        uint16_t accelValue = Wire.read() | (Wire.read() << 8);

        Joystick.setZAxis(clutchValue);
        Joystick.setBrake(brakeValue);
        Joystick.setAccelerator(accelValue);
    }
}

void processGearbox()
{
    sendGearboxButtons(readGearbox());
}

uint16_t readGearbox()
{
    Wire.beginTransmission(I2C_ADDRESS_GEARBOX_CUSTOM);
    Wire.endTransmission();
    
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
        
        Joystick.setRxAxis(handbrake);

        Joystick.setButton(23, isBitSet(buttons, 0)); 
    }
}

void processSteeringButtons() {
    sendSteeringButtons(readSteeringButtons());
}

uint32_t readSteeringButtons() {
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
    
    //Serial.println(Wire.available());

    if (Wire.available() == 6) {
        uint16_t vrx = Wire.read() | (Wire.read() << 8);
        uint16_t vry = Wire.read() | (Wire.read() << 8);
        uint16_t buttons = Wire.read() | (Wire.read() << 8);

        Joystick.setSlider(vrx);
        Joystick.setDial(vry);
    }
}

void processForces()
{
    effectparams[0].springPosition = steeringFilteredValue - 512;
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
    Wire.write(ffbValue & 0xFF);     // младший байт
    Wire.write((ffbValue >> 8) & 0xFF); // старший байт
    Wire.endTransmission();
}