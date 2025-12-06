#ifndef CONTROLCAMARA_H
#define CONTROLCAMARA_H

#include <Arduino.h>
#include "esp_camera.h"

class ControlCamara {
public:
    ControlCamara();

    bool begin();                       // Inicializa la cámara
    camera_fb_t* capturarFrame();       // Captura un frame JPEG
    void liberarFrame(camera_fb_t* fb); // Libera el frame
    bool estaInicializada() const;      // Retorna estado

private:
    camera_config_t config; // Ahora es privado
    bool inicializada;
};

#endif
