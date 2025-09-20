/*/

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

/**/