#include "ControlCamara.h"

ControlCamara::ControlCamara()
    : inicializada(false)
{
    // Configuración para ESP32-CAM AI-THINKER
    config.pin_pwdn = 32;
    config.pin_reset = -1;
    config.pin_xclk = 0;
    config.pin_sscb_sda = 26;
    config.pin_sscb_scl = 27;
    config.pin_d7 = 35;
    config.pin_d6 = 34;
    config.pin_d5 = 39;
    config.pin_d4 = 36;
    config.pin_d3 = 21;
    config.pin_d2 = 19;
    config.pin_d1 = 18;
    config.pin_d0 = 5;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_pclk = 22;

    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_SVGA;  
    config.jpeg_quality = 60;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
}

bool ControlCamara::begin() {
    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK) {
        inicializada = false;
        return false;
    }

    inicializada = true;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 0);
        s->set_gain_ctrl(s, 1);
        s->set_agc_gain(s, 0);
        s->set_gainceiling(s, (gainceiling_t)0);
        s->set_bpc(s, 0);
        s->set_wpc(s, 1);
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        s->set_dcw(s, 1);
        s->set_colorbar(s, 0);
    }

    return true;
}

camera_fb_t* ControlCamara::capturarFrame() {
    if (!inicializada) return nullptr;
    return esp_camera_fb_get();
}

void ControlCamara::liberarFrame(camera_fb_t* fb) {
    if (fb) esp_camera_fb_return(fb);
}

bool ControlCamara::estaInicializada() const {
    return inicializada;
}
