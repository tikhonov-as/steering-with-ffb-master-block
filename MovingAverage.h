/*/
#pragma once

#include "Arduino.h"

#define MOVING_AVERAGE_SIZE 5

class MovingAverage {
private:
    int* buffer;    // динамический буфер
    int index;      // текущий индекс
    int sum;        // сумма значений
    int size;       // размер буфера

public:
    // Конструктор
    MovingAverage(int bufferSize);
    
    // Деструктор
    ~MovingAverage();
    
    // Методы
    MovingAverage& update(int newValue);
    int getAverage();
};
 /**/