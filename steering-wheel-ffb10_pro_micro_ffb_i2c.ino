#include "Joystick.h"
#include "DigitalWriteFast.h"
#include "PCF8574.h"
#include "ADS1115_WE.h"
#include "MovingAverage.h"

bool debug = false;

int32_t forces[2] = {0};
Gains gains[2];
EffectParams effectparams[2];


#define ANALOG_INPUT_HANDBRAKE A9 // Rx

// это прочитанные значения с ADC
// #define STEERING_MIN_VALUE  123
// #define STEERING_MAX_VALUE  914
#define CLUTCH_MIN_VALUE    840     // 570 // старый блок педалей - 309
#define CLUTCH_MAX_VALUE    112     // 112 // старый блок педалей - 511
#define BRAKE_MIN_VALUE     832     // 578 // старый блок педалей - 160
#define BRAKE_MAX_VALUE     40      // 25  // старый блок педалей - 395
#define ACCEL_MIN_VALUE     981     // 680 // старый блок педалей - 207
#define ACCEL_MAX_VALUE     560     // 339 // старый блок педалей - 511
#define HANDBRAKE_MIN_VALUE 492
#define HANDBRAKE_MAX_VALUE 780
// а это значения, на которые мапим - которые будут высылаться на ПК
#define ANALOG_OUT_MIN_VALUE 0
#define ANALOG_OUT_MAX_VALUE 1023
#define ANALOG_OUT_MAX_VALUE_10bit 1023
#define ANALOG_OUT_MAX_VALUE_12bit 4095
#define ANALOG_OUT_MAX_VALUE_14bit 16383
#define ANALOG_OUT_MAX_VALUE_16bit 65535

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
#define I2C_ADDRESS_GEARBOX 0x63
#define I2C_ADDRESS_PEDALS ADS1115_ADDRESS_VDD
#define I2C_ADDRESS_BUTTON_BOX PCF8574_ADDRESS_0
#define I2C_ADDRESS_IGNITION_PORT PCF8574_ADDRESS_2

#define IGNITION_PIN_ACC 0
#define IGNITION_PIN_ST  1
#define IGNITION_PIN_BAT 2
#define IGNITION_PIN_IG  3
#define IGNITION_PIN_BTN 4

int32_t steeringActualValue = 0;
int32_t steeringScaledValue = 0;
int32_t steeringFilteredValue = 0;
int32_t clutchActualValue = 0;
int32_t clutchScaledValue = 0;
int32_t clutchFilteredValue = 0;
int32_t brakeActualValue = 0;
int32_t brakeScaledValue = 0;
int32_t brakeFilteredValue = 0;
int32_t accelActualValue = 0;
int32_t accelScaledValue = 0;
int32_t accelFilteredValue = 0;
int32_t handbrakeActualValue = 0;
int32_t handbrakeScaledValue = 0;
int32_t handbrakeFilteredValue = 0;

#define MOVING_AVERAGE_SIZE 5
MovingAverage steeringAverageFilter(MOVING_AVERAGE_SIZE);
MovingAverage clutchAverageFilter(MOVING_AVERAGE_SIZE);
MovingAverage brakeAverageFilter(MOVING_AVERAGE_SIZE);
MovingAverage accelAverageFilter(MOVING_AVERAGE_SIZE);
MovingAverage handbrakeAverageFilter(MOVING_AVERAGE_SIZE);

ADS1115_WE adcPedals = ADS1115_WE(I2C_ADDRESS_PEDALS);

// кнопки 1-15 - стоковые на руле. DPad (Hat) - отдельно. Кнопка Prog не считается номерной кнопкой контроллера (т.е. служебная)
// кнопки 16-24 - кнопки КПП
// кнопки 25-40 - кнопки ButtonBox1
// кнопки 41-45 - кнопки Ignition

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, // // JOYSTICK_TYPE_MULTI_AXIS
                   45, 1,                 // Button Count, Hat Switch Count
                   true, true, true,      // X Y Z // STEERING CLUTCH BRAKE
                   true, false,  true,     // Rx Ry Rz // ACCEL ? HANDBRAKE
                   false, false,          // rudder throttle
                   false, false, false);  // accelerator brake steering

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
} KeyStateData;


// int32_t  currentPosition = 0;
// volatile int8_t oldState = 0;
// const int8_t KNOBDIR[] = {
//   0, 1, -1, 0,
//   -1, 0, 0, 1,
//   1, 0, 0, -1,
//   0, -1, 1, 0
// };

// void readEncoderValue(void)
// {
//   int sig1 = digitalReadFast(encoderPinA);
//   int sig2 = digitalReadFast(encoderPinB);
//   int8_t thisState = sig1 | (sig2 << 1);

//   if (oldState != thisState) {
//     currentPosition += KNOBDIR[thisState | (oldState<<2)];
//     oldState = thisState;
//   } 
// }

// 0x27
// PCF8575 buttonBoxPort(0x21);
// PCF8575 simpleButtonsPort(0x27);

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
    setupAds1115Pedals();
    setupGearbox();
    setupHandbrake();
    setupButtonBox1();
    setupIgnitionKey();
}

void setupJoystick()
{
    Joystick.setXAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setZAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setRzAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setYAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    Joystick.setRxAxisRange(ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);

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

void setupAds1115Pedals()
{
    if (adcPedals.init()) {
        adcPedals.setVoltageRange_mV(ADS1115_RANGE_6144);
        adcPedals.setCompareChannels(ADS1115_COMP_0_GND);
        adcPedals.setMeasureMode(ADS1115_CONTINUOUS);
        adcPedals.setConvRate(ADS1115_250_SPS); // def 128

        Serial.println("adcPedals.init success");
    } else {
        Serial.println("adcPedals.init error");
    }
}

void setupGearbox()
{

}

void setupHandbrake()
{
    pinMode(ANALOG_INPUT_HANDBRAKE, INPUT);
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
    processForces();

    // debug
    if (0 && debug && setInterval(250)) {
        Serial.println("-------------------");

        Serial.print("adcPedals.isDisconnected: ");
        Serial.println(adcPedals.isDisconnected());

        Serial.println("control\t\taxis\tactual\tscaled\tfiltered");

        Serial.print("steering\tX\t");
        Serial.print(steeringActualValue);
        Serial.print("\t");
        Serial.print(steeringScaledValue);
        Serial.print("\t");
        Serial.println(steeringFilteredValue);

        Serial.print("clutch\t\tZ\t");
        Serial.print(clutchActualValue);
        Serial.print("\t");
        Serial.print(clutchScaledValue);
        Serial.print("\t");
        Serial.println(clutchFilteredValue);

        Serial.print("brake\t\tRz\t");
        Serial.print(brakeActualValue);
        Serial.print("\t");
        Serial.print(brakeScaledValue);
        Serial.print("\t");
        Serial.println(brakeFilteredValue);

        Serial.print("accel\t\tY\t");
        Serial.print(accelActualValue);
        Serial.print("\t");
        Serial.print(accelScaledValue);
        Serial.print("\t");
        Serial.println(accelFilteredValue);

        Serial.print("handbrake\tRx\t");
        Serial.print(handbrakeActualValue);
        Serial.print("\t");
        Serial.print(handbrakeScaledValue);
        Serial.print("\t");
        Serial.println(handbrakeFilteredValue);

        Serial.println("-------------------");
        Serial.print("steeringFilteredValue: ");
        Serial.println(steeringFilteredValue);
        Serial.print("effectparams[0].springPosition: ");
        Serial.println(effectparams[0].springPosition);
        Serial.print("force: ");
        Serial.print(forces[0]);
        Serial.print("; ");
        Serial.print(forces[1]);

    }    
}

void processSteering()
{
    steeringActualValue = readSteeringValue();
    steeringScaledValue = steeringActualValue;
    steeringFilteredValue = steeringAverageFilter.update(steeringScaledValue).getAverage();
    Joystick.setXAxis(steeringFilteredValue);
}

void processPedals()
{
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
}

int16_t* readAds1115Channels(ADS1115_WE adc) {
    static int16_t values[3];

    values[0] = readAds1115ChannelRaw(adc, ADS1115_COMP_0_GND) >> ADS1115_NONSIGNIFICANT_BITS;
    values[1] = readAds1115ChannelRaw(adc, ADS1115_COMP_1_GND) >> ADS1115_NONSIGNIFICANT_BITS;
    values[2] = readAds1115ChannelRaw(adc, ADS1115_COMP_2_GND) >> ADS1115_NONSIGNIFICANT_BITS;

    return values;
}

void processGearbox()
{
    sendGearboxButtons(readGearbox());
}

uint16_t readGearbox()
{
    return 0;
}

void sendGearboxButtons(uint16_t states) {
    Joystick.setButton(15, isBitSet(states, 0));    // btn1
    Joystick.setButton(16, isBitSet(states, 1));    // 1
    Joystick.setButton(17, isBitSet(states, 2));    // 2
    Joystick.setButton(18, isBitSet(states, 3));    // 3
    Joystick.setButton(19, isBitSet(states, 4));    // 4
    Joystick.setButton(20, isBitSet(states, 5));    // 5
    Joystick.setButton(21, isBitSet(states, 6));    // 6
    Joystick.setButton(22, isBitSet(states, 7));    // R
    Joystick.setButton(23, isBitSet(states, 8));    // btn2
}

void processHandbrake()
{
    handbrakeActualValue = analogRead(ANALOG_INPUT_HANDBRAKE);
    handbrakeScaledValue = constrain(map(handbrakeActualValue, HANDBRAKE_MIN_VALUE, HANDBRAKE_MAX_VALUE, ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE), ANALOG_OUT_MIN_VALUE, ANALOG_OUT_MAX_VALUE);
    handbrakeFilteredValue = handbrakeAverageFilter.update(handbrakeScaledValue).getAverage();
    Joystick.setRxAxis(handbrakeFilteredValue);
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

uint16_t processButtonBox() {
    sendButtonBoxState(readButtonBox());
}

uint16_t readButtonBox()
{
    uint16_t buttonState = 0xFFFF;  // Инициализируем все биты в 1

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
    for (byte buttonIterator = 0; buttonIterator < 16; buttonIterator++) {

        Joystick.setButton(
                buttonIterator + 24,
                isBitSet(buttonStates, buttonIterator)
        );
    }
}

void processIgnitionKeyState() {
    sendIgnitionKeyState(readIgnitionKeyState());
}

KeyStateData readIgnitionKeyState() {
    KeyStateData data;

    ignitionKeyPort.digitalWrite(IGNITION_PIN_ACC, LOW);
    // ignitionKeyPort.digitalWrite(IGNITION_PIN_ST, HIGH);
    data.acc_bat = ignitionKeyPort.digitalRead(IGNITION_PIN_BAT);
    data.acc_ig = ignitionKeyPort.digitalRead(IGNITION_PIN_IG);

    ignitionKeyPort.digitalWrite(IGNITION_PIN_ACC, HIGH);
    ignitionKeyPort.digitalWrite(IGNITION_PIN_ST, LOW);
    data.st_bat = ignitionKeyPort.digitalRead(IGNITION_PIN_BAT);
    data.st_ig = ignitionKeyPort.digitalRead(IGNITION_PIN_IG);

    ignitionKeyPort.digitalWrite(IGNITION_PIN_ST, HIGH);

    uint8_t keyState = (data.acc_bat << 0) |
                       (data.acc_ig << 1) |
                       (data.st_bat << 2) |
                       (data.st_ig << 3);

    switch (keyState) {
        case 0b0001: data.keyPos = 1; break;
        case 0b0010: data.keyPos = 2; break;
        case 0b0011: data.keyPos = 3; break;
        case 0b1100: data.keyPos = 4; break;
        default: data.keyPos = 0; break;
    }

    data.button = ignitionKeyPort.digitalRead(IGNITION_PIN_BTN);

    return data;
}

void sendIgnitionKeyState(KeyStateData state) {
    Joystick.setButton(40, state.keyPos == 1);
    Joystick.setButton(41, state.keyPos == 2);
    Joystick.setButton(42, state.keyPos == 3);
    Joystick.setButton(43, state.keyPos == 4);
    Joystick.setButton(44, state.button == 0);
}



void processForces()
{
    effectparams[0].springPosition = steeringFilteredValue - 512;
    effectparams[1].springPosition = brakeFilteredValue;

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