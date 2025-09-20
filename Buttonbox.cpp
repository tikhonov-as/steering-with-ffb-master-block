#include "Buttonbox.h"

/*/
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

    // Инвертируем результат, чтобы активные кнопки были 1
    // return ~buttonState & 0xFFFF;

    // Serial.print("buttonState: ");
    // Serial.println(buttonState);
    return buttonState;
}
void sendButtonBoxState(uint16_t buttonStates)
{
    for (byte buttonIterator = 0; buttonIterator < 16; buttonIterator++) {

        Joystick.setButton(
            buttonIterator + 24,
            buttonStates & bit(buttonIterator)
        );
    }
}

/**/