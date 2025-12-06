#ifndef CONTROLFLASH_H
#define CONTROLFLASH_H

#include <Arduino.h>

class ControlFlash {
private:
    int pin_flash;
    bool estado;

public:
    ControlFlash(int pin);

    void begin();
    void encender();
    void apagar();
    void toggle();

    bool getEstado() const { return estado; }
};

#endif
