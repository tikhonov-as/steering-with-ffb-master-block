#include "MovingAverage.h"

// Реализация конструктора
MovingAverage::MovingAverage(int bufferSize) {
    size = bufferSize;
    buffer = new int[size];
    index = 0;
    sum = 0;
    
    // Инициализация буфера нулями
    for(int i = 0; i < size; i++) {
        buffer[i] = 0;
    }
}

// Реализация деструктора
MovingAverage::~MovingAverage() {
    delete[] buffer;
}

// Реализация метода обновления
MovingAverage& MovingAverage::update(int newValue) {
    sum -= buffer[index];
    buffer[index] = newValue;
    sum += newValue;
    index = (index + 1) % size;
    return *this;  // возвращаем ссылку на текущий объект
}
// Реализация метода получения среднего
int MovingAverage::getAverage() {
    return sum / size;
}