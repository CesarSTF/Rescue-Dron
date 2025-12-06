#include "ControlFlash.h"

ControlFlash::ControlFlash(int pin)
    : pin_flash(pin), estado(false) {}

void ControlFlash::begin() {
    pinMode(pin_flash, OUTPUT);
    digitalWrite(pin_flash, LOW);
}

void ControlFlash::encender() {
    digitalWrite(pin_flash, HIGH);
    estado = true;
}

void ControlFlash::apagar() {
    digitalWrite(pin_flash, LOW);
    estado = false;
}

void ControlFlash::toggle() {
    estado ? apagar() : encender();
}
