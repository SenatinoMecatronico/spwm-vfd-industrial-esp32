#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_timer.h"
#include <math.h>

// ============================================================================
// 1. MAPA DE PINES DE LA TARJETA 1
// ============================================================================
#define PIN_SPWM_H1   25    // J_VFD1 Pin 2 -> 6N137 Superior Rama A
#define PIN_SPWM_L1   26    // J_VFD1 Pin 3 -> 6N137 Inferior Rama A
#define PIN_SPWM_H2   27    // J_VFD2 Pin 1 -> 6N137 Superior Rama B (Puente H)
#define PIN_SPWM_L2   14    // J_VFD2 Pin 2 -> 6N137 Inferior Rama B (Puente H)
#define PIN_VFD_SD    13    // J_VFD2 Pin 3 -> Shutdown (HIGH = Habilitado)

#define PIN_ENC_CLK    4    // J_ENC Pin 1 -> GPIO 4
#define PIN_ENC_DT    19    // J_ENC Pin 2 -> GPIO 19
#define PIN_ENC_SW    15    // J_ENC Pin 3 -> GPIO 15 (Pulsador RUN / STOP)

#define PIN_OLED_SCL  22    // J_OLED Pin 3 -> GPIO 22
#define PIN_OLED_SDA  21    // J_OLED Pin 4 -> GPIO 21

#define PIN_PLC_Q1    32    // U1 Pin 4 -> GPIO 32 (Q1 LOGO!: Marcha / Paro)
#define PIN_PLC_Q2    33    // U2 Pin 4 -> GPIO 33 (Q2 LOGO!: Vel. Bit 0)
#define PIN_PLC_Q3    18    // U3 Pin 4 -> GPIO 18 (Q3 LOGO!: Vel. Bit 1)

// ============================================================================
// 2. PARÁMETROS DEL VARIADOR Y CALIBRACIÓN REAL (167.7 VAC)
// ============================================================================
#define CARRIER_FREQ_HZ   10000     // Portadora SPWM = 10 kHz
#define LUT_SIZE          256       // Tabla senoidal de 256 puntos
#define DEADTIME_TICKS    15        // 1.5 us de Tiempo Muerto por hardware
#define F_MAX_HZ          60.0f     // Frecuencia máxima (60.0 Hz)
#define F_CUTOFF_HZ       1.0f      // Apaga MOSFETs recién al bajar de 1.0 Hz
#define RAMP_HZ_PER_SEC   15.0f     // Rampa de aceleración y frenado (15 Hz/s)
#define V_MAX_CALIBRATED  167.7f    // Voltaje máximo real medido con filtro LC

// Credenciales de la Red Wi-Fi creada por el ESP32
const char* AP_SSID = "MINI_VFD_INDUSTRIAL";
const char* AP_PASS = "12345678";

Adafruit_SSD1306 display(128, 64, &Wire, -1);
WebServer server(80);
float sineLUT[LUT_SIZE];

// Variables de tiempo real (DDS 10 kHz)
volatile bool powerStageEnabled = false;
volatile uint32_t phaseAccumulator = 0;
volatile uint32_t phaseStep = 0;
volatile float modulationIndex = 0.0f;

// Variables del Encoder KY-040
volatile int32_t encoderSteps = 0;
volatile uint32_t lastEncInterruptUs = 0;

// Variables de estado del variador
bool localOrWebRunActive = false;
float targetManualFreqHz = 0.0f;
float setpointFreqHz = 0.0f;
float currentFreqHz = 0.0f;
float vOutRMS = 0.0f;
float vLowRMS = 0.0f;
bool plcQ1_Run = false, plcQ2_Bit0 = false, plcQ3_Bit1 = false;
const char* activeModeStr = "STOP";

esp_timer_handle_t spwm_timer_handle;

// ============================================================================
// 3. INTERFAZ WEB SCADA EMBEBIDA (HTML5 + CSS + JS EN MEMORIA FLASH)
// ============================================================================
const char SCADA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SCADA Mini-VFD SPWM</title>
<style>
  body { font-family: system-ui, sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 16px; text-align: center; }
  .card { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 16px; max-width: 420px; margin: 0 auto 14px; box-shadow: 0 4px 12px rgba(0,0,0,0.4); }
  h2 { margin: 0 0 6px; font-size: 1.2rem; color: #38bdf8; }
  .badge { display: inline-block; padding: 4px 12px; border-radius: 99px; font-weight: 700; font-size: 0.85rem; background: #334155; }
  .run { background: #16a34a; color: #fff; }
  .stop { background: #dc2626; color: #fff; }
  .ramp { background: #d97706; color: #fff; }
  .big-val { font-size: 2.8rem; font-weight: 800; margin: 8px 0; color: #facc15; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 10px; }
  .metric { background: #0f172a; padding: 10px; border-radius: 8px; border: 1px solid #1e293b; }
  .metric span { display: block; font-size: 0.78rem; color: #94a3b8; }
  .metric strong { font-size: 1.25rem; color: #e2e8f0; }
  .btn-row { display: flex; gap: 10px; margin-top: 12px; }
  button { flex: 1; padding: 12px; border: none; border-radius: 8px; font-weight: 700; font-size: 0.95rem; cursor: pointer; color: #fff; }
  .btn-run { background: #16a34a; }
  .btn-stop { background: #dc2626; }
  .btn-preset { background: #2563eb; padding: 8px; font-size: 0.85rem; }
  input[type=range] { width: 100%; margin: 14px 0; accent-color: #38bdf8; height: 8px; }
  .plc-box { display: flex; justify-content: space-around; margin-top: 8px; }
  .led { width: 14px; height: 14px; border-radius: 50%; display: inline-block; background: #334155; vertical-align: middle; margin-right: 5px; }
  .led.on { background: #22c55e; box-shadow: 0 0 8px #22c55e; }
</style>
</head>
<body>
  <div class="card">
    <h2>SCADA VFD SPWM - INDUSTRIA 4.0</h2>
    <div id="modeBadge" class="badge stop">STOP</div>
    <div class="big-val"><span id="freqNow">0.0</span> <small style="font-size:1.2rem">Hz</small></div>
    <div class="grid">
      <div class="metric"><span>Consigna Objetivo</span><strong><span id="freqSet">0.0</span> Hz</strong></div>
      <div class="metric"><span>Relación V/f</span><strong><span id="vfPct">0</span> %</strong></div>
      <div class="metric"><span>Salida Trafo 220V</span><strong><span id="vHigh">0.0</span> VAC</strong></div>
      <div class="metric"><span>Primario 9V</span><strong><span id="vLow">0.0</span> VAC</strong></div>
    </div>
  </div>

  <div class="card">
    <div style="font-size:0.85rem; color:#94a3b8; margin-bottom:6px;">ESTADO ENTRADAS PLC SIEMENS LOGO! (24VDC)</div>
    <div class="plc-box">
      <div><span id="q1" class="led"></span>Q1 (RUN)</div>
      <div><span id="q2" class="led"></span>Q2 (VEL.0)</div>
      <div><span id="q3" class="led"></span>Q3 (VEL.1)</div>
    </div>
  </div>

  <div class="card">
    <div style="font-size:0.9rem; font-weight:600;">Control Remoto Wi-Fi (0.0 a 60.0 Hz)</div>
    <input type="range" id="freqSlider" min="0" max="60" step="0.5" value="0" onchange="setFreq(this.value)" oninput="document.getElementById('sliderVal').innerText=this.value">
    <div style="font-size:0.85rem; color:#38bdf8;">Ajuste manual: <strong id="sliderVal">0.0</strong> Hz</div>
    <div class="btn-row">
      <button class="btn-preset" onclick="setFreq(0)">0 Hz</button>
      <button class="btn-preset" onclick="setFreq(20)">20 Hz</button>
      <button class="btn-preset" onclick="setFreq(40)">40 Hz</button>
      <button class="btn-preset" onclick="setFreq(60)">60 Hz</button>
    </div>
    <div class="btn-row">
      <button class="btn-run" onclick="sendCmd('run', 1)">MARCHA (RUN)</button>
      <button class="btn-stop" onclick="sendCmd('run', 0)">PARO EN RAMPA</button>
    </div>
  </div>

<script>
let userDragging = false;
const slider = document.getElementById('freqSlider');
slider.addEventListener('touchstart', () => userDragging = true);
slider.addEventListener('touchend', () => userDragging = false);
slider.addEventListener('mousedown', () => userDragging = true);
slider.addEventListener('mouseup', () => userDragging = false);

function sendCmd(param, val) {
  fetch(`/cmd?${param}=${val}`).then(r => r.json()).then(updateUI);
}
function setFreq(hz) {
  document.getElementById('sliderVal').innerText = parseFloat(hz).toFixed(1);
  slider.value = hz;
  sendCmd('freq', hz);
}
function updateUI(d) {
  document.getElementById('freqNow').innerText = d.fNow.toFixed(1);
  document.getElementById('freqSet').innerText = d.fSet.toFixed(1);
  document.getElementById('vfPct').innerText = d.vf;
  document.getElementById('vHigh').innerText = d.vH.toFixed(1);
  document.getElementById('vLow').innerText = d.vL.toFixed(1);
  const b = document.getElementById('modeBadge');
  b.innerText = d.mode;
  b.className = 'badge ' + (d.mode === 'STOP' ? 'stop' : (d.mode === 'FRENANDO' ? 'ramp' : 'run'));
  document.getElementById('q1').className = 'led ' + (d.q1 ? 'on' : '');
  document.getElementById('q2').className = 'led ' + (d.q2 ? 'on' : '');
  document.getElementById('q3').className = 'led ' + (d.q3 ? 'on' : '');
  if (!userDragging) {
    slider.value = d.fMan;
    document.getElementById('sliderVal').innerText = d.fMan.toFixed(1);
  }
}
setInterval(() => {
  fetch('/status').then(r => r.json()).then(updateUI).catch(()=>{});
}, 350);
</script>
</body>
</html>
)rawliteral";

// ============================================================================
// 4. INTERRUPCIONES EN TIEMPO REAL (ENCODER Y GENERADOR DDS 10 kHz)
// ============================================================================
void IRAM_ATTR onEncoderClkChange() {
    uint32_t nowUs = micros();
    if (nowUs - lastEncInterruptUs < 2500) return;

    if (digitalRead(PIN_ENC_CLK) == LOW) {
        lastEncInterruptUs = nowUs;
        if (digitalRead(PIN_ENC_DT) == HIGH) {
            if (encoderSteps < 120) encoderSteps++;
        } else {
            if (encoderSteps > 0) encoderSteps--;
        }
    }
}

void IRAM_ATTR onSPWMTimer(void* arg) {
    if (!powerStageEnabled) return;

    phaseAccumulator += phaseStep;
    uint8_t index = (uint8_t)(phaseAccumulator >> 24);
    float sineVal = sineLUT[index];

    float dutyA = 50.0f + (46.0f * modulationIndex * sineVal);
    float dutyB = 50.0f - (46.0f * modulationIndex * sineVal);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyA);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dutyB);
}

// ============================================================================
// 5. CONTROL DE ENCENDIDO Y APAGADO SEGURO DE LA ETAPA DE POTENCIA
// ============================================================================
void disablePowerStageSafe() {
    powerStageEnabled = false;
    digitalWrite(PIN_VFD_SD, LOW);
    delayMicroseconds(200);

    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
    delayMicroseconds(50);

    mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_1);
}

void enablePowerStageWithBootstrap() {
    mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_1);

    // Pre-cargar capacitores Bootstrap encendiendo Q2 por 5 ms
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
    digitalWrite(PIN_VFD_SD, HIGH);
    delay(5);

    phaseAccumulator = 0;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0f);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 50.0f);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0f);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 50.0f);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                          MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                          DEADTIME_TICKS, DEADTIME_TICKS);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,
                          MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                          DEADTIME_TICKS, DEADTIME_TICKS);

    powerStageEnabled = true;
}

// ============================================================================
// 6. ENDPOINTS DEL SERVIDOR WEB SCADA
// ============================================================================
void sendJsonTelemetry() {
    char json[256];
    snprintf(json, sizeof(json),
             "{\"fNow\":%.1f,\"fSet\":%.1f,\"fMan\":%.1f,\"vf\":%d,\"vH\":%.1f,\"vL\":%.1f,\"mode\":\"%s\",\"q1\":%d,\"q2\":%d,\"q3\":%d}",
             currentFreqHz, setpointFreqHz, targetManualFreqHz,
             (int)(modulationIndex * 100.0f), vOutRMS, vLowRMS, activeModeStr,
             plcQ1_Run ? 1 : 0, plcQ2_Bit0 ? 1 : 0, plcQ3_Bit1 ? 1 : 0);
    server.send(200, "application/json", json);
}

void setupWebServer() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    server.on("/", HTTP_GET, []() {
        server.send_P(200, "text/html", SCADA_HTML);
    });

    server.on("/status", HTTP_GET, []() {
        sendJsonTelemetry();
    });

    server.on("/cmd", HTTP_GET, []() {
        if (server.hasArg("run")) {
            localOrWebRunActive = (server.arg("run").toInt() == 1);
        }
        if (server.hasArg("freq")) {
            float f = server.arg("freq").toFloat();
            if (f < 0.0f) f = 0.0f;
            if (f > F_MAX_HZ) f = F_MAX_HZ;
            encoderSteps = (int32_t)roundf(f * 2.0f); // Sincroniza con el Encoder KY-040
        }
        sendJsonTelemetry();
    });

    server.begin();
}

// ============================================================================
// 7. CONFIGURACIÓN INICIAL (SETUP)
// ============================================================================
void setup() {
    pinMode(PIN_SPWM_H1, OUTPUT);
    pinMode(PIN_SPWM_L1, OUTPUT);
    pinMode(PIN_SPWM_H2, OUTPUT);
    pinMode(PIN_SPWM_L2, OUTPUT);
    pinMode(PIN_VFD_SD, OUTPUT);
    digitalWrite(PIN_SPWM_H1, LOW);
    digitalWrite(PIN_SPWM_L1, LOW);
    digitalWrite(PIN_SPWM_H2, LOW);
    digitalWrite(PIN_SPWM_L2, LOW);
    digitalWrite(PIN_VFD_SD, LOW);

    Serial.begin(115200);

    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT, INPUT_PULLUP);
    pinMode(PIN_ENC_SW, INPUT_PULLUP);
    pinMode(PIN_PLC_Q1, INPUT_PULLUP);
    pinMode(PIN_PLC_Q2, INPUT_PULLUP);
    pinMode(PIN_PLC_Q3, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), onEncoderClkChange, CHANGE);

    for (int i = 0; i < LUT_SIZE; i++) {
        sineLUT[i] = sinf((2.0f * M_PI * (float)i) / (float)LUT_SIZE);
    }

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_SPWM_H1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_SPWM_L1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_SPWM_H2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_SPWM_L2);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = CARRIER_FREQ_HZ;
    pwm_config.cmpr_a = 0.0f;
    pwm_config.cmpr_b = 0.0f;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

    disablePowerStageSafe();

    const esp_timer_create_args_t timer_args = {
        .callback = &onSPWMTimer,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "spwm_dds_timer"
    };
    esp_timer_create(&timer_args, &spwm_timer_handle);
    esp_timer_start_periodic(spwm_timer_handle, 100);

    setupWebServer();

    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    Wire.setClock(400000);
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("WIFI: MINI_VFD_IND");
        display.println("IP:   192.168.4.1");
        display.display();
        delay(1200);
    }
}

// ============================================================================
// 8. BUCLE PRINCIPAL: LÓGICA DE CONTROL, RAMPA REAL Y TELEMETRÍA
// ============================================================================
void loop() {
    static bool lastBtnState = HIGH;
    static uint32_t lastDebounceMs = 0;
    static uint32_t lastRampUpdateMs = millis();
    static uint32_t lastOledUpdateMs = 0;

    // Atender clientes del SCADA Web Wi-Fi
    server.handleClient();

    // --- 1. Leer Pulsador del Encoder KY-040 (GPIO 15: RUN / STOP) ---
    bool currentBtnState = digitalRead(PIN_ENC_SW);
    if (currentBtnState == LOW && lastBtnState == HIGH && (millis() - lastDebounceMs > 250)) {
        lastDebounceMs = millis();
        localOrWebRunActive = !localOrWebRunActive;
    }
    lastBtnState = currentBtnState;

    // --- 2. Frecuencia seleccionada por Encoder o Slider Web ---
    targetManualFreqHz = (float)encoderSteps * 0.5f;

    // --- 3. Leer Entradas Aisladas del PLC LOGO! ---
    plcQ1_Run  = (digitalRead(PIN_PLC_Q1) == LOW);
    plcQ2_Bit0 = (digitalRead(PIN_PLC_Q2) == LOW);
    plcQ3_Bit1 = (digitalRead(PIN_PLC_Q3) == LOW);

    bool runRequested = localOrWebRunActive || plcQ1_Run;
    uint8_t plcSpeedCode = (plcQ3_Bit1 ? 2 : 0) | (plcQ2_Bit0 ? 1 : 0);

    if (runRequested) {
        if (plcSpeedCode == 1) {
            setpointFreqHz = 20.0f;
            activeModeStr = "PLC VEL.1";
        } else if (plcSpeedCode == 2) {
            setpointFreqHz = 40.0f;
            activeModeStr = "PLC VEL.2";
        } else if (plcSpeedCode == 3) {
            setpointFreqHz = 60.0f;
            activeModeStr = "PLC VEL.3";
        } else {
            setpointFreqHz = targetManualFreqHz;
            activeModeStr = plcQ1_Run ? "PLC+MAN" : "RUN ACTIVO";
        }
    } else {
        setpointFreqHz = 0.0f;
        activeModeStr = (currentFreqHz >= F_CUTOFF_HZ) ? "FRENANDO" : "STOP";
    }

    // --- 4. Rampa Suave de Aceleración y Frenado (15 Hz/s) ---
    uint32_t nowMs = millis();
    float dtSec = (nowMs - lastRampUpdateMs) / 1000.0f;
    lastRampUpdateMs = nowMs;

    float maxStepHz = RAMP_HZ_PER_SEC * dtSec;
    if (currentFreqHz < setpointFreqHz) {
        currentFreqHz += maxStepHz;
        if (currentFreqHz > setpointFreqHz) currentFreqHz = setpointFreqHz;
    } else if (currentFreqHz > setpointFreqHz) {
        currentFreqHz -= maxStepHz;
        if (currentFreqHz < setpointFreqHz) currentFreqHz = setpointFreqHz;
    }

    // --- 5. CORRECCIÓN CLAVE: Los MOSFETs siguen modulando durante TODA la rampa ---
    // Solo se apagan cuando la frecuencia real baja de 1.0 Hz.
    if (currentFreqHz >= F_CUTOFF_HZ) {
        if (!powerStageEnabled) {
            enablePowerStageWithBootstrap();
        }
    } else {
        if (powerStageEnabled) {
            disablePowerStageSafe();
        }
    }

    // --- 6. Ley V/f Proporcional ---
    float ma = 0.0f;
    if (powerStageEnabled) {
        ma = 0.05f + (0.95f * (currentFreqHz / F_MAX_HZ));
        if (ma > 1.0f) ma = 1.0f;
    }

    phaseStep = (uint32_t)((currentFreqHz * 4294967296.0f) / (float)CARRIER_FREQ_HZ);
    modulationIndex = ma;

    // Telemetría calibrada con tus mediciones reales (7.3V en 9V y 167.7V en 220V)
    vLowRMS = powerStageEnabled ? (ma * 7.3f) : 0.0f;
    vOutRMS = powerStageEnabled ? (ma * V_MAX_CALIBRATED) : 0.0f;

    // --- 7. Actualizar Pantalla OLED ---
    if (millis() - lastOledUpdateMs >= 150) {
        lastOledUpdateMs = millis();

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print(activeModeStr);
        display.setCursor(68, 0);
        display.print("192.168.4.1");
        display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

        display.setTextSize(2);
        display.setCursor(0, 14);
        display.print(currentFreqHz, 1);
        display.print(" Hz");

        display.setTextSize(1);
        display.setCursor(0, 35);
        display.print("Set: ");
        display.print(runRequested ? setpointFreqHz : targetManualFreqHz, 1);
        display.println(" Hz");

        display.setCursor(0, 45);
        display.print("PLC Q1-Q3: ");
        display.print(plcQ1_Run ? "1 " : "0 ");
        display.print(plcQ2_Bit0 ? "1 " : "0 ");
        display.println(plcQ3_Bit1 ? "1" : "0");

        display.setCursor(0, 55);
        display.print("Salida AC: ");
        display.print(vOutRMS, 1);
        display.println(" V");

        display.display();
    }

    delay(5);
}
