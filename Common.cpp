/*
 todo сделать классом как MovingAverage
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

 inline bool isBitSet(uint32_t state, uint8_t bit) {
 if (bit > 31) return false;
 return (state & (static_cast<uint32_t>(1) << bit)) != 0;
}

*/