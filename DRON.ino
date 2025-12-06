#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- LIBRERÍAS PROPIAS ---
#include "MotorController.h"
#include "SensorIMU.h"
#include "FlightController.h"
#include "ControlCamara.h"
#include "ControlFlash.h"

// ------------------ CONFIG WiFi ------------------
const char* ssid = "DEIVY";
const char* password = "19082024Dav#Day";

// ------------------ COMPONENTES ------------------
MotorController motors;
SensorIMU imu;
FlightController fc(&motors, &imu); // El controlador que une todo (PID + Motores + Sensor)
ControlCamara camara;
ControlFlash flash(4);

AsyncWebServer server(80);

// Variables compartidas (Volátiles para FreeRTOS)
volatile float web_throttle = 0.0f;
volatile float web_roll = 0.0f;
volatile float web_pitch = 0.0f;
volatile bool web_armed = false;

// Handle para la tarea de vuelo
TaskHandle_t TaskFlightHandle;

// ------------------ HTML (JOYSTICK + VIDEO) ------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>DEIVY DRONE</title>
  <style>
    body { margin: 0; padding: 0; background: #111; overflow: hidden; font-family: sans-serif; touch-action: none; }
    #cam-stream { position: absolute; top: 0; left: 0; width: 100%; height: 100%; object-fit: cover; z-index: -1; transform: rotate(180deg); opacity: 0.8; }
    .hud { position: absolute; top: 0; left: 0; width: 100%; height: 100%; display: flex; justify-content: space-between; align-items: flex-end; padding-bottom: 20px; box-sizing: border-box; pointer-events: none; }
    
    .left-panel { pointer-events: auto; width: 100px; height: 260px; margin-left: 20px; display: flex; flex-direction: column; align-items: center; background: rgba(0,0,0,0.4); border-radius: 15px; padding: 10px; backdrop-filter: blur(2px); }
    input[type=range][orient=vertical] { writing-mode: bt-lr; -webkit-appearance: slider-vertical; width: 40px; height: 200px; margin-bottom: 10px; }
    .label { color: cyan; font-weight: bold; margin-bottom: 5px; text-shadow: 1px 1px 2px black; }

    .right-panel { pointer-events: auto; width: 180px; height: 180px; margin-right: 20px; margin-bottom: 20px; background: rgba(255, 255, 255, 0.1); border-radius: 50%; border: 2px solid rgba(255,255,255,0.3); position: relative; }
    #joystick { width: 100%; height: 100%; }

    .top-bar { position: absolute; top: 10px; width: 100%; text-align: center; pointer-events: auto; display: flex; justify-content: center; gap: 15px; align-items: center; }
    .btn { padding: 10px 20px; font-size: 14px; border-radius: 5px; border: none; font-weight: bold; cursor: pointer; text-transform: uppercase; box-shadow: 0px 4px 6px rgba(0,0,0,0.5); color: white; }
    .btn-arm { background: #e74c3c; }
    .btn-active { background: #2ecc71; color: black; box-shadow: 0 0 15px #2ecc71; }
    .btn-flash { background: #f39c12; }
    .btn-flash-on { background: #f1c40f; color: black; box-shadow: 0 0 15px #f1c40f; }
  </style>
</head>
<body>
  <img id="cam-stream" src="/stream">
  <div class="top-bar">
    <button id="btnArm" class="btn btn-arm" onclick="toggleArm()">DESARMADO</button>
    <button id="btnFlash" class="btn btn-flash" onclick="toggleFlash()">🔦</button>
    <div style="color:white; font-size:12px; text-shadow:1px 1px 2px black;" id="debug">WiFi: DEIVY</div>
  </div>
  <div class="hud">
    <div class="left-panel">
        <div class="label">POTENCIA</div>
        <div class="label" id="throtVal">0%</div>
        <input type="range" orient="vertical" min="0" max="100" value="0" id="sliderT" oninput="sendThrottle(this.value)" onchange="sendThrottle(this.value)">
    </div>
    <div class="right-panel" id="joyContainer">
        <canvas id="joystick"></canvas>
    </div>
  </div>
<script>
    let armed = false;
    let flashOn = false;

    function toggleArm() {
        armed = !armed;
        const btn = document.getElementById("btnArm");
        if(armed) {
            btn.innerText = "¡ARMADO!"; btn.className = "btn btn-active";
            fetch("/action?cmd=arm");
        } else {
            btn.innerText = "DESARMADO"; btn.className = "btn btn-arm";
            fetch("/action?cmd=disarm");
            document.getElementById("sliderT").value = 0; sendThrottle(0);
        }
    }
    function toggleFlash() {
        flashOn = !flashOn;
        const btn = document.getElementById("btnFlash");
        fetch('/flash?state=' + (flashOn ? '1' : '0')).then(() => { btn.className = flashOn ? "btn btn-flash-on" : "btn btn-flash"; });
    }
    let lastThrottle = 0;
    function sendThrottle(val) {
        document.getElementById("throtVal").innerText = val + "%";
        if(Math.abs(val - lastThrottle) > 1) { fetch("/control?var=t&val=" + val); lastThrottle = val; }
    }
    const canvas = document.getElementById("joystick");
    const ctx = canvas.getContext("2d");
    const container = document.getElementById("joyContainer");
    canvas.width = container.offsetWidth; canvas.height = container.offsetHeight;
    let centerX = canvas.width/2, centerY = canvas.height/2, maxDist = (canvas.width/2)-35, isDragging = false;
    let currX = centerX, currY = centerY;

    function drawJoystick() {
        ctx.clearRect(0,0,canvas.width,canvas.height);
        ctx.beginPath(); ctx.arc(centerX,centerY,10,0,Math.PI*2); ctx.fillStyle="rgba(255,255,255,0.3)"; ctx.fill();
        ctx.beginPath(); ctx.arc(currX,currY,35,0,Math.PI*2); ctx.fillStyle="cyan"; ctx.fill();
    }
    function sendJoyData(x, y) {
        let roll = Math.round((x/maxDist)*100);
        let pitch = Math.round((y/maxDist)*100)*-1;
        document.getElementById("debug").innerText="R: "+roll+" P: "+pitch;
        fetch("/control?var=j&r="+roll+"&p="+pitch);
    }
    function move(e) {
        if(!isDragging) return; e.preventDefault();
        let clientX = e.touches ? e.touches[0].clientX : e.clientX;
        let clientY = e.touches ? e.touches[0].clientY : e.clientY;
        const rect = canvas.getBoundingClientRect();
        let dx = clientX - rect.left - centerX, dy = clientY - rect.top - centerY, dist = Math.sqrt(dx*dx+dy*dy);
        if(dist > maxDist) { dx=(dx/dist)*maxDist; dy=(dy/dist)*maxDist; }
        currX=centerX+dx; currY=centerY+dy; drawJoystick(); sendJoyData(dx,dy);
    }
    function start(e) { isDragging=true; move(e); }
    function end() { isDragging=false; currX=centerX; currY=centerY; drawJoystick(); sendJoyData(0,0); }
    canvas.addEventListener('mousedown',start); canvas.addEventListener('mousemove',move);
    canvas.addEventListener('mouseup',end); canvas.addEventListener('mouseleave',end);
    canvas.addEventListener('touchstart',start); canvas.addEventListener('touchmove',move);
    canvas.addEventListener('touchend',end);
    drawJoystick();
</script>
</body>
</html>
)rawliteral";

// ------------------ HANDLERS DEL SERVIDOR ------------------

void handleRoot(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", index_html);
}

void handleAction(AsyncWebServerRequest *request) {
  if (request->hasParam("cmd")) {
    String cmd = request->getParam("cmd")->value();
    if (cmd == "arm") web_armed = true;
    else if (cmd == "disarm") { web_armed = false; web_throttle = 0; }
  }
  request->send(200, "text/plain", "OK");
}

void handleControl(AsyncWebServerRequest *request) {
  if (request->hasParam("var") && request->hasParam("val")) {
    String v = request->getParam("var")->value();
    // t = Throttle (0-100)
    if(v == "t") {
      web_throttle = request->getParam("val")->value().toFloat() / 100.0f;
    }
    // j = Joystick (Roll/Pitch)
    else if (v == "j") {
      if(request->hasParam("r")) web_roll = request->getParam("r")->value().toFloat() / 100.0f;
      if(request->hasParam("p")) web_pitch = request->getParam("p")->value().toFloat() / 100.0f;
    }
  }
  request->send(200, "text/plain", "OK");
}

void handleFlash(AsyncWebServerRequest *request) {
  if (!request->hasParam("state")) {
    request->send(400, "text/plain", "Falta state");
    return;
  }
  bool state = (request->getParam("state")->value() == "1");
  state ? flash.encender() : flash.apagar();
  request->send(200, "text/plain", flash.getEstado() ? "ON" : "OFF");
}

void handleStream(AsyncWebServerRequest *request) {
  if (!camara.estaInicializada()) {
    request->send(503, "text/plain", "Camara Error");
    return;
  }

  AsyncWebServerResponse *response = request->beginChunkedResponse(
    "multipart/x-mixed-replace; boundary=123456789000000000000987654321",
    [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      static camera_fb_t *fb = nullptr;
      static size_t _jpg_buf_len = 0;
      static uint8_t *_jpg_buf = nullptr;
      static size_t bytesSent = 0;
      static bool headerSent = false;
      const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
      const char* _STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
      
      size_t bytesWritten = 0;

      if (!fb) {
        fb = esp_camera_fb_get();
        if (!fb) return 0;
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = nullptr;
          if (!jpeg_converted) return 0;
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
        bytesSent = 0;
        headerSent = false;
      }

      if (!headerSent) {
        char part_buf[64];
        size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
        if (hlen < maxLen) {
          memcpy(buffer, part_buf, hlen);
          bytesWritten += hlen;
          headerSent = true;
        } else return 0;
      }

      if (headerSent && bytesSent < _jpg_buf_len) {
        size_t remainingBuffer = maxLen - bytesWritten;
        size_t remainingImage = _jpg_buf_len - bytesSent;
        size_t toSend = min(remainingBuffer, remainingImage);
        memcpy(buffer + bytesWritten, _jpg_buf + bytesSent, toSend);
        bytesWritten += toSend;
        bytesSent += toSend;
      }

      if (bytesSent >= _jpg_buf_len && bytesWritten < maxLen) {
        size_t remainingBuffer = maxLen - bytesWritten;
        size_t boundaryLen = strlen(_STREAM_BOUNDARY);
        if (remainingBuffer >= boundaryLen) {
          memcpy(buffer + bytesWritten, _STREAM_BOUNDARY, boundaryLen);
          bytesWritten += boundaryLen;
          if (fb) { esp_camera_fb_return(fb); fb = nullptr; }
          else if (_jpg_buf) { free(_jpg_buf); _jpg_buf = nullptr; }
        }
      }
      return bytesWritten;
    }
  );
  response->addHeader("Access-Control-Allow-Origin", "*");
  request->send(response);
}

// ------------------ TAREA DE VUELO (Core 1) ------------------
void FlightTaskCode( void * pvParameters ) {
  // NOTA: Serial NO FUNCIONA aquí porque estará apagado
  unsigned long lastTimeTask = 0;

  for(;;) {
    unsigned long now = micros();
    float dt = (now - lastTimeTask) / 1000000.0f;
    lastTimeTask = now;
    if (dt <= 0 || dt > 0.1) dt = 0.004; 

    // Copia segura de variables Web
    float targetThrottle = web_throttle;
    float targetRoll = web_roll;
    float targetPitch = web_pitch;
    bool systemArmed = web_armed;

    if (systemArmed) {
        if (!fc.motors->isArmed()) fc.arm();
        
        // --- CONTROLADOR DE VUELO REAL (PID) ---
        fc.setThrottle(targetThrottle);
        fc.setRollInput(targetRoll);
        fc.setPitchInput(targetPitch);
        
        // Esta función lee el IMU, calcula PID y mueve motores
        fc.update(dt); 
        
    } else {
        if (fc.motors->isArmed()) fc.disarm();
    }
    
    // Mantenemos 250Hz - 500Hz aprox
    vTaskDelay(2 / portTICK_PERIOD_MS); 
  }
}

// ------------------ SETUP (Core 0) ------------------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  
  // 1. INICIO CONSOLA (SOLO PARA DIAGNÓSTICO INICIAL)
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== DEIVY DRONE: STARTUP ===");

  // 2. CONFIGURACIÓN WIFI ROBUSTA (Estilo Carro)
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  delay(100);
  
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  
  Serial.println("Creando Red WiFi...");
  bool apCreated = WiFi.softAP(ssid, password);
  
  if (apCreated) {
    Serial.println("✓ WiFi Creado: " + String(ssid));
    Serial.print("✓ IP: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("✗ ERROR WiFi: Reiniciando...");
    delay(2000); ESP.restart();
  }

  // 3. INICIO HARDWARE BÁSICO
  motors.begin();
  camara.begin(); 
  flash.begin();

  // 4. TRANSICIÓN CRÍTICA: SERIAL -> SENSOR
  Serial.println("!!! ATENCION !!!");
  Serial.println("Apagando Serial Monitor en 3 segundos...");
  Serial.println("Conecta el Sensor MPU6050 ahora si no lo has hecho.");
  delay(3000);
  Serial.end(); // ADIÓS CONSOLA, HOLA PINES 1 Y 3

  // 5. INICIO SENSOR Y VUELO (Ahora que los pines estan libres)
  // SensorIMU.cpp usa Wire.begin(1, 3) internamente
  imu.begin(); 
  fc.begin();

  // 6. INICIO SERVIDOR
  server.on("/", HTTP_GET, handleRoot);
  server.on("/action", HTTP_GET, handleAction);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/flash", HTTP_GET, handleFlash);
  server.on("/stream", HTTP_GET, handleStream);
  server.begin();

  // 7. LANZAR TAREA DE VUELO
  xTaskCreatePinnedToCore(FlightTaskCode, "FlightTask", 10000, NULL, 1, &TaskFlightHandle, 1);
}

void loop() {
  // Loop vacío, todo corre en Background (Server) y Core 1 (Vuelo)
  delay(1000);
}