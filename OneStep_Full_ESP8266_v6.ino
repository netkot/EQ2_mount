// ============================================================
//  EQ2 Mount Controller  —  v6.0
//  ESP8266 NodeMCU
//
//  Протоколы:
//    • Web UI        http://<IP>/
//    • RTS2 JSON API http://<IP>/api/...   (Stellarium)
//    • LX200/OnStep  TCP <IP>:4030         (KStars, PHD2, Cartes du Ciel)
//    • OTA update    http://<IP>/update
//
//  Изменения v6 vs v5:
//    • LX200/OnStep команды приведены в полное соответствие
//      с lx200driver.h (INDI) и OnStepListCmd.txt
//    • Добавлены: :ACK (0x06), :GX0n#, :GX4n#, :GX9n#,
//      :GX91#, :GXS#, :GXU#, :SXA#, :GXGA#, :GXGO#,
//      :GX98# (rotator none), PEC-заглушки :VH# :VR# :VS# :VW#,
//      Library-заглушки :LI# :LR# :LB# :LN# :LC# :LD# :LL# :Lo#,
//      :Rotator-заглушки :rA# и прочие :r*#
//    • Исправлены форматы ответов :GU# :Gu# :GW# per lx200driver
//    • :GT# → "60.1642#" когда трекинг включён, "0.0000#" иначе
//    • :D#  → "\x7f#" / "#" (per spec "\0x7f#")
//    • DEC мотор не отключается после остановки (держит ось)
// ============================================================

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <AccelStepper.h>
#include <ElegantOTA.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>

// ============================================================
//  WiFi
//  Данные вынесены в secrets.h (не коммитить!). Шаблон: secrets.h.example
// ============================================================
#include "secrets.h"

// ============================================================
//  Пины драйверов TMC2209 (STEP/DIR/EN — как у A4988)
// ============================================================
#define RA_STEP_PIN    4
#define RA_DIR_PIN     5
#define RA_ENABLE_PIN  0

#define DEC_STEP_PIN   12
#define DEC_DIR_PIN    14
#define DEC_ENABLE_PIN 13

// ============================================================
//  Механика
// ============================================================
const int   MICROSTEPS     = 16;
const int   STEPS_PER_REV  = 200 * MICROSTEPS;   // 3200 шаг/об

// DEC: шестерни 2×10 = 1:5
const float DEC_GEAR_RATIO  = 5.0f;
const long  DEC_STEPS_PER_DEG =
    (long)((STEPS_PER_REV * DEC_GEAR_RATIO) / 360.0f);   // 44

// RA: ремень GT2 16→80 (×5) + червяк EQ2 (×144) = 1:720
const float RA_BELT_RATIO  = 80.0f / 16.0f;
const float RA_WORM_RATIO  = 144.0f;
const float RA_TOTAL_RATIO = RA_BELT_RATIO * RA_WORM_RATIO; // 720
const long  RA_STEPS_PER_DEG =
    (long)((STEPS_PER_REV * RA_TOTAL_RATIO) / 360.0f);   // 6400

// Сидерическая скорость
const float SIDEREAL_RATE     = 360.0f / 86164.0f;       // °/сек
const float RA_SIDEREAL_SPEED = SIDEREAL_RATE * RA_STEPS_PER_DEG;

// Инверсия направления (поменяйте знак если крутит не туда)
const int   RA_DIR  =  1;
const int   DEC_DIR =  1;

// ============================================================
//  Объекты
// ============================================================
AccelStepper stepperRA (AccelStepper::DRIVER, RA_STEP_PIN,  RA_DIR_PIN);
AccelStepper stepperDEC(AccelStepper::DRIVER, DEC_STEP_PIN, DEC_DIR_PIN);
ESP8266WebServer server(80);

// ============================================================
//  TMC2209 UART — однопроводная шина на D4 (GPIO2) через 1 кОм
//  к общему узлу PDN_UART. Адреса: RA = 0, DEC = 1 (MS1/MS2).
// ============================================================
#define TMC_UART_PIN   2
const long  TMC_BAUD       = 57600;   // ≤74880 — асинхронный RX-ISR SoftwareSerial
const float TMC_RSENSE     = 0.11f;
const int   RA_CURRENT_MA  = 1190;    // 70% от 1.7 A (JK42HS40-1704)
const int   DEC_CURRENT_MA = 840;     // 70% от 1.2 A (JK42HS60-1206)

// SoftwareSerial в onewire-режиме сам не переключает пин на передачу —
// обёртка включает TX на время записи. Пока TX включён, RX выключен,
// поэтому собственного эха в приёмном буфере нет.
class OneWireUart : public Stream {
public:
    explicit OneWireUart(SoftwareSerial& s) : ss(s) {}
    size_t write(uint8_t b) override {
        ss.enableTx(true);
        size_t n = ss.write(b);
        ss.enableTx(false);
        return n;
    }
    int available() override { return ss.available(); }
    int read()      override { return ss.read(); }
    int peek()      override { return ss.peek(); }
    void flush()    override {}
private:
    SoftwareSerial& ss;
};

SoftwareSerial tmcSerial;
OneWireUart    tmcBus(tmcSerial);
TMC2209Stepper drvRA (&tmcBus, TMC_RSENSE, 0);
TMC2209Stepper drvDEC(&tmcBus, TMC_RSENSE, 1);
bool raDrvOk  = false;
bool decDrvOk = false;
unsigned long lastTmcCheck = 0;

// ============================================================
//  Состояние монтировки
// ============================================================
bool   trackingEnabled = false;
float  raSpeedMult     = 1.0f;    // множитель сидерической скорости
float  raManualSpeed   = 0.0f;    // °/сек, ручное RA
float  decManualSpeed  = 0.0f;    // °/сек, ручное DEC
bool   isParked        = false;
bool   isSlewing       = false;

// Координаты J2000, градусы
double currentRA_deg  = 0.0;     // 0..360
double currentDEC_deg = 0.0;     // -90..+90
double targetRA_deg   = 0.0;
double targetDEC_deg  = 0.0;

// Геопозиция обсерватории
double obsLongitude_deg = 30.5;  // ← ваша долгота (East +)
double obsLatitude_deg  = 50.5;  // ← ваша широта  (North +)
double obsAltitude_m    = 30.0;  // ← высота над уровнем моря, м

// ============================================================
//  Авто-отключение моторов
// ============================================================
const unsigned long MOTOR_OFF_DELAY = 1000;  // мс
unsigned long raStopTime  = 0;
unsigned long decStopTime = 0;
bool raMoving  = false;
bool decMoving = false;

// Удержание оси DEC мотором после остановки.
// true  = мотор DEC остаётся включённым (рекомендуется если ось ослаблена).
// false = мотор DEC отключается через MOTOR_OFF_DELAY мс (тихий режим).
bool decHoldEnabled = true;

// ============================================================
//  Настройки моторов
// ============================================================
float raMaxSpeed  = 6000.0f;
float raAccel     = 1000.0f;
float decMaxSpeed =  500.0f;
float decAccel    =  100.0f;

unsigned long lastTrackUpdate = 0;

// ============================================================
//  LX200 TCP сервер (порт 4030)
// ============================================================
WiFiServer lxTcpServer(4030);
WiFiClient lxTcpClient;
bool       lxClientConnected = false;
String     lxTcpBuf          = "";
// Флаг: мы внутри команды (между ':' и '#').
// БЕЗ него ':' внутри аргумента (:Sr22:01:48#, :Sd+45*30:00#)
// сбрасывал буфер и на обработку попадал только хвост ("48", "00").
bool       lxInCommand       = false;

// LX200 переменные
float lxGuideRate     = SIDEREAL_RATE; // °/сек (1× sidereal по умолчанию)
bool  lxHighPrecision = true;          // true = HH:MM:SS / +DD*MM:SS

// ============================================================
//  Вспомогательные функции
// ============================================================

void updateCurrentCoords() {
    double rawRA = stepperRA.currentPosition()
                   / (double)(RA_STEPS_PER_DEG * RA_DIR);
    rawRA = fmod(rawRA, 360.0);
    if (rawRA < 0.0) rawRA += 360.0;
    currentRA_deg  = rawRA;
    currentDEC_deg = stepperDEC.currentPosition()
                     / (double)(DEC_STEPS_PER_DEG * DEC_DIR);
}

void enableRA (bool on) { digitalWrite(RA_ENABLE_PIN,  on ? LOW : HIGH); }
void enableDEC(bool on) { digitalWrite(DEC_ENABLE_PIN, on ? LOW : HIGH); }

void startGoTo(double ra_deg, double dec_deg) {
    long raSteps  = (long)(ra_deg  * RA_STEPS_PER_DEG  * RA_DIR);
    long decSteps = (long)(dec_deg * DEC_STEPS_PER_DEG * DEC_DIR);
    enableRA(true);
    enableDEC(true);
    stepperRA .moveTo(raSteps);
    stepperDEC.moveTo(decSteps);
    raMoving  = true;
    decMoving = true;
    isSlewing = true;
    isParked  = false;
    Serial.printf("GoTo: RA=%.4f° DEC=%.4f°\n", ra_deg, dec_deg);
}

// ============================================================
//  HTML веб-интерфейс
// ============================================================
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>EQ2 Mount</title>
  <style>
    *{margin:0;padding:0;box-sizing:border-box}
    body{font-family:'Segoe UI',sans-serif;
         background:linear-gradient(135deg,#1a1a2e 0%,#16213e 50%,#0f3460 100%);
         min-height:100vh;padding:12px;color:#e0e0e0}
    .container{max-width:820px;margin:0 auto}

    /* ── Статус-строка ── */
    .statusbar{background:rgba(0,0,0,0.45);border:1px solid rgba(255,255,255,0.1);
               border-radius:10px;padding:10px 14px;margin-bottom:10px;
               font-size:13px;line-height:1.7;text-align:center}
    .statusbar .coords{font-size:1.2em;font-weight:700;color:#e2b96f;
                       letter-spacing:.03em;margin-bottom:4px}

    /* ── Секции ── */
    .card{background:rgba(255,255,255,0.04);border:1px solid rgba(255,255,255,0.08);
          border-radius:10px;margin-bottom:8px;overflow:hidden}

    /* Заголовок секции — кнопка раскрытия */
    .card-hdr{display:flex;align-items:center;justify-content:space-between;
              padding:10px 14px;cursor:pointer;user-select:none;
              border-bottom:1px solid transparent;transition:background .2s}
    .card-hdr:hover{background:rgba(255,255,255,0.05)}
    .card-hdr h2{font-size:1em;font-weight:600;color:#a0c4ff;margin:0}
    .card-hdr .arr{font-size:.75em;color:#666;transition:transform .25s}
    .card-hdr.open{border-bottom-color:rgba(255,255,255,0.08)}
    .card-hdr.open .arr{transform:rotate(180deg)}

    /* Тело секции */
    .card-body{padding:12px 14px;display:none}
    .card-body.open{display:block}

    /* Секции всегда открытые (без заголовка-кнопки) */
    .card-always .card-body{display:block;padding:12px 14px}

    /* ── Сетка ── */
    .g2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
    @media(max-width:500px){.g2{grid-template-columns:1fr}}

    /* ── Координаты ── */
    .coord-box{background:rgba(0,0,0,0.3);border-radius:7px;
               padding:8px;text-align:center}
    .coord-val{font-size:1.25em;color:#e2b96f;font-weight:700}
    .coord-lbl{font-size:.75em;color:#777;margin-top:2px}

    /* ── Ввод ── */
    label{display:block;margin-bottom:4px;font-size:.85em;color:#bbb}
    input[type=range]{width:100%;height:5px;border-radius:3px;
                      background:rgba(255,255,255,0.15);outline:none;margin-top:2px}
    input[type=number]{width:100%;padding:7px 10px;
                       border:1px solid rgba(255,255,255,0.15);border-radius:7px;
                       background:rgba(255,255,255,0.08);color:#fff;font-size:14px}

    /* ── Кнопки ── */
    button{width:100%;padding:10px;margin:4px 0;border:none;
           border-radius:8px;font-size:14px;font-weight:600;
           cursor:pointer;transition:all .2s}
    .btn-track{background:linear-gradient(135deg,#10b981,#059669);color:#fff}
    .btn-stop {background:linear-gradient(135deg,#ef4444,#dc2626);color:#fff}
    .btn-nav  {background:rgba(96,165,250,0.2);color:#93c5fd;
               border:1px solid rgba(96,165,250,0.3)}
    .btn-goto {background:linear-gradient(135deg,#f59e0b,#d97706);color:#fff}
    .btn-muted{background:rgba(255,255,255,0.08);color:#ccc}
    .btn-warn {background:rgba(239,68,68,0.18);color:#fca5a5;
               border:1px solid rgba(239,68,68,0.3)}
    .btn-estop{background:linear-gradient(135deg,#ef4444,#dc2626);color:#fff;
               font-size:16px;padding:13px}
    button:hover{filter:brightness(1.18);box-shadow:0 3px 10px rgba(0,0,0,0.3)}
    button:active{filter:brightness(.9)}

    /* ── D-pad ── */
    .dpad{display:grid;grid-template-columns:repeat(3,60px);
          grid-template-rows:repeat(3,48px);gap:6px;margin:0 auto;width:198px}
    .dpad button{padding:0;font-size:18px;border-radius:7px;margin:0;width:auto;height:auto}
    .cell-N{grid-column:2;grid-row:1}
    .cell-W{grid-column:1;grid-row:2}
    .cell-C{grid-column:2;grid-row:2}
    .cell-E{grid-column:3;grid-row:2}
    .cell-D{grid-column:2;grid-row:3}

    /* ── Info box ── */
    .ibox{background:rgba(0,0,0,0.22);border-radius:7px;
          padding:9px 12px;margin-top:8px;font-size:12px;
          border:1px solid rgba(255,255,255,0.06);line-height:1.65;color:#aaa}
    code{background:rgba(255,255,255,0.09);padding:1px 5px;
         border-radius:3px;font-family:monospace;font-size:11px}
    .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px}
    .dot-g{background:#10b981} .dot-r{background:#ef4444}
    .badge{display:inline-block;padding:2px 8px;margin:2px;
           border-radius:4px;background:rgba(255,255,255,0.09);font-size:11px}

    /* ── Toggle switch ── */
    .tog{position:relative;display:inline-block;width:52px;height:28px;flex-shrink:0}
    .tog input{opacity:0;width:0;height:0}
    .tog-sl{position:absolute;cursor:pointer;inset:0;
            background:rgba(239,68,68,.55);border-radius:28px;transition:.3s}
    .tog-sl:before{content:"";position:absolute;width:20px;height:20px;
                   left:4px;bottom:4px;background:#fff;border-radius:50%;transition:.3s}
    .tog input:checked+.tog-sl{background:#10b981}
    .tog input:checked+.tog-sl:before{transform:translateX(24px)}

    /* ── Заголовок страницы ── */
    .page-title{text-align:center;font-size:1.5em;color:#e2b96f;
                text-shadow:0 0 18px rgba(226,185,111,.35);
                padding:10px 0 8px;font-weight:700}
  </style>
</head>
<body>
<div class="container">

  <div class="page-title">🔭 EQ2 Mount</div>

  <!-- ═══ Статус (всегда виден) ════════════════════════════════════════ -->
  <div class="statusbar">
    <div class="coords">RA <span id="raVal">—</span>°&nbsp;&nbsp;DEC <span id="decVal">—</span>°</div>
    <span id="statusBox">Loading…</span>
  </div>

  <!-- ═══ Трекинг + экстренная остановка (всегда видны) ═══════════════ -->
  <div class="card card-always">
    <div class="card-body">
      <div class="g2" style="align-items:center;gap:8px">
        <button id="trackBtn" class="btn-track" onclick="toggleTracking()">▶ Start Tracking</button>
        <button class="btn-estop" onclick="emergencyStop()">⛔ STOP</button>
      </div>
      <div style="margin-top:10px">
        <label>Tracking speed: <span id="spdVal">1.00×</span> sidereal</label>
        <input type="range" min="0" max="2" step="0.05" value="1" oninput="setTrackSpeed(this.value)">
      </div>
    </div>
  </div>

  <!-- ═══ Ручное управление (всегда видно) ════════════════════════════ -->
  <div class="card card-always">
    <div class="card-body">
      <div class="dpad">
        <button class="btn-nav cell-N"
          onmousedown="jog('dec',1)" onmouseup="jogStop('dec')"
          ontouchstart="jog('dec',1)" ontouchend="jogStop('dec')">▲</button>
        <button class="btn-nav cell-W"
          onmousedown="jog('ra',-1)" onmouseup="jogStop('ra')"
          ontouchstart="jog('ra',-1)" ontouchend="jogStop('ra')">◄</button>
        <button class="btn-stop cell-C" onclick="stopAll()">■</button>
        <button class="btn-nav cell-E"
          onmousedown="jog('ra',1)" onmouseup="jogStop('ra')"
          ontouchstart="jog('ra',1)" ontouchend="jogStop('ra')">►</button>
        <button class="btn-nav cell-D"
          onmousedown="jog('dec',-1)" onmouseup="jogStop('dec')"
          ontouchstart="jog('dec',-1)" ontouchend="jogStop('dec')">▼</button>
      </div>
      <div style="margin-top:10px">
        <label>Jog speed: <span id="jogVal">14</span> °/s</label>
        <input type="range" id="jogSpeed" min="2" max="60" step="2" value="14"
               oninput="document.getElementById('jogVal').innerText=this.value">
      </div>
    </div>
  </div>

  <!-- ═══ GoTo (раскрываемая) ════════════════════════════════════════ -->
  <div class="card">
    <div class="card-hdr" onclick="tog(this)">
      <h2>🎯 GoTo / Sync</h2><span class="arr">▼</span>
    </div>
    <div class="card-body">
      <div class="g2">
        <div>
          <label>RA (degrees 0–360):</label>
          <input type="number" id="gotoRA" value="0" step="0.001" min="0" max="360">
        </div>
        <div>
          <label>DEC (degrees −90..+90):</label>
          <input type="number" id="gotoDEC" value="0" step="0.001" min="-90" max="90">
        </div>
      </div>
      <button class="btn-goto" onclick="doGoTo()" style="margin-top:8px">🚀 Go To</button>
      <div class="g2">
        <button class="btn-muted" onclick="setHome()">⌂ Set Home</button>
        <button class="btn-muted" onclick="doSync()">⟳ Sync</button>
      </div>
    </div>
  </div>

  <!-- ═══ Park / DEC hold (раскрываемая) ════════════════════════════ -->
  <div class="card">
    <div class="card-hdr" onclick="tog(this)">
      <h2>🅿 Park &amp; DEC Hold</h2><span class="arr">▼</span>
    </div>
    <div class="card-body">
      <div class="g2">
        <button class="btn-warn" onclick="doPark()">Park</button>
        <button class="btn-muted" onclick="doUnpark()">Unpark</button>
      </div>
      <div style="display:flex;align-items:center;gap:12px;margin-top:12px;flex-wrap:wrap">
        <label class="tog">
          <input type="checkbox" id="decHoldChk" onchange="setDecHold(this.checked)" checked>
          <span class="tog-sl"></span>
        </label>
        <span id="decHoldLabel" style="font-size:.95em;font-weight:600;color:#10b981">
          DEC Hold ON
        </span>
      </div>
      <div class="ibox">
        DEC Hold: мотор удерживает ось после остановки.<br>
        ⚠ Мотор и драйвер DEC постоянно под током (в покое TMC2209 снижает ток вдвое) — нужен радиатор на драйвере.
      </div>
    </div>
  </div>

  <!-- ═══ Настройки моторов (раскрываемая) ══════════════════════════ -->
  <div class="card">
    <div class="card-hdr" onclick="tog(this)">
      <h2>⚙️ Motor Settings</h2><span class="arr">▼</span>
    </div>
    <div class="card-body">
      <div class="g2">
        <div>
          <label>RA max speed (steps/s):</label>
          <input type="number" id="raSpd" value="6000" min="50" max="6000" step="50">
        </div>
        <div>
          <label>DEC max speed (steps/s):</label>
          <input type="number" id="decSpd" value="500" min="50" max="4000" step="50">
        </div>
        <div>
          <label>RA acceleration (steps/s²):</label>
          <input type="number" id="raAcc" value="1000" min="50" max="4000" step="50">
        </div>
        <div>
          <label>DEC acceleration (steps/s²):</label>
          <input type="number" id="decAcc" value="100" min="50" max="1000" step="50">
        </div>
      </div>
      <div class="g2" style="margin-top:8px">
        <button class="btn-muted" onclick="applyMotor('ra')">Apply RA</button>
        <button class="btn-muted" onclick="applyMotor('dec')">Apply DEC</button>
      </div>
    </div>
  </div>

  <!-- ═══ Протоколы (раскрываемая) ══════════════════════════════════ -->
  <div class="card">
    <div class="card-hdr" onclick="tog(this)">
      <h2>🌐 Protocols &amp; Connections</h2><span class="arr">▼</span>
    </div>
    <div class="card-body">
      <span class="badge">✓ RTS2 HTTP/JSON</span>
      <span class="badge">✓ LX200/OnStep TCP:4030</span>
      <div class="ibox">
        <p><span class="dot" id="rts2dot"></span>RTS2: <code id="rts2url">http://…/</code></p>
        <p style="margin-top:5px">LX200: <code id="lx200addr">…:4030</code></p>
        <p style="margin-top:5px">
          <b>Stellarium:</b> Type=RTS2, URL above, Refresh=500ms<br>
          <b>KStars/PHD2/CdC:</b> Type=LX200/OnStep, port 4030
        </p>
      </div>
    </div>
  </div>

  <!-- ═══ OTA Update (раскрываемая) ════════════════════════════════ -->
  <div class="card">
    <div class="card-hdr" onclick="tog(this)">
      <h2>🔄 Firmware Update (OTA)</h2><span class="arr">▼</span>
    </div>
    <div class="card-body">
      <div class="ibox" style="margin-top:0;margin-bottom:8px">
        Version: <b>6.0</b> &nbsp;|&nbsp; <span class="dot dot-g"></span>Ready
      </div>
      <button class="btn-muted" onclick="window.open('/update','_blank')">Open Update Page</button>
      <p style="font-size:11px;color:#666;margin-top:6px">⚠ Do not disconnect power during update!</p>
    </div>
  </div>

</div><!-- /container -->

<script>
var trackOn=false;
function fmt(v,d){return Number(v).toFixed(d||4);}

/* ── Раскрытие секций ── */
function tog(hdr){
  hdr.classList.toggle('open');
  hdr.nextElementSibling.classList.toggle('open');
}

/* ── Поллинг статуса ── */
function poll(){
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('raVal').innerText=fmt(d.ra_deg);
    document.getElementById('decVal').innerText=fmt(d.dec_deg);
    document.getElementById('statusBox').innerHTML=
      '<b>Tracking:</b> '+(d.tracking?'<span style="color:#10b981">ON</span>':'OFF')+
      ' &nbsp;|&nbsp; <b>Slewing:</b> '+(d.slewing?'<span style="color:#f59e0b">YES</span>':'NO')+
      ' &nbsp;|&nbsp; <b>Parked:</b> '+(d.parked?'<span style="color:#60a5fa">YES</span>':'NO');
    if(d.ip){
      var u='http://'+d.ip+'/';
      document.getElementById('rts2url').innerText=u;
      document.getElementById('lx200addr').innerText=d.ip+':4030';
    }
    document.getElementById('rts2dot').className=
      'dot '+(d.tracking||d.slewing?'dot-g':'dot-r');
    if(typeof d.dec_hold!=='undefined') updateDecHoldUI(d.dec_hold);
  }).catch(()=>{
    document.getElementById('statusBox').innerText='Connection error…';
  });
}

/* ── Трекинг ── */
function toggleTracking(){
  trackOn=!trackOn;
  fetch('/tracking?enable='+(trackOn?'1':'0')).then(poll);
  var b=document.getElementById('trackBtn');
  b.innerText=trackOn?'⏹ Stop Tracking':'▶ Start Tracking';
  b.className=trackOn?'btn-stop':'btn-track';
}
function setTrackSpeed(v){
  document.getElementById('spdVal').innerText=parseFloat(v).toFixed(2)+'×';
  fetch('/ra-speed?speed='+v);
}

/* ── Джойстик ── */
function jog(axis,dir){
  fetch('/jog?axis='+axis+'&speed='+(dir*document.getElementById('jogSpeed').value));
}
function jogStop(axis){fetch('/jog?axis='+axis+'&speed=0');}
function stopAll(){fetch('/stop-all').then(poll);}

/* ── GoTo ── */
function doGoTo(){
  fetch('/goto?ra='+document.getElementById('gotoRA').value+
              '&dec='+document.getElementById('gotoDEC').value).then(poll);
}
function setHome(){fetch('/set-home').then(poll);}
function doSync(){
  fetch('/sync?ra='+document.getElementById('gotoRA').value+
              '&dec='+document.getElementById('gotoDEC').value).then(poll);
}

/* ── Park ── */
function doPark()  {fetch('/park').then(poll);}
function doUnpark(){fetch('/unpark').then(poll);}

/* ── DEC Hold ── */
function setDecHold(on){
  fetch('/dec-hold?enable='+(on?'1':'0'))
    .then(r=>r.json()).then(d=>updateDecHoldUI(d.dec_hold));
}
function updateDecHoldUI(on){
  var l=document.getElementById('decHoldLabel');
  document.getElementById('decHoldChk').checked=on;
  l.innerText=on?'DEC Hold ON':'DEC Hold OFF';
  l.style.color=on?'#10b981':'#f59e0b';
}

/* ── Настройки моторов ── */
function applyMotor(axis){
  var spd=document.getElementById(axis+'Spd').value;
  var acc=document.getElementById(axis+'Acc').value;
  fetch('/motor?axis='+axis+'&speed='+spd+'&accel='+acc)
    .then(r=>r.json()).then(d=>{
      alert(d.ok?axis.toUpperCase()+' OK':'Error: '+d.error);
    });
}

/* ── Emergency stop ── */
function emergencyStop(){
  fetch('/estop').then(poll);
  trackOn=false;
  var b=document.getElementById('trackBtn');
  b.innerText='▶ Start Tracking';
  b.className='btn-track';
}

setInterval(poll,2000);
poll();
</script>
</body>
</html>
)rawliteral";

// ============================================================
//  TMC2209: настройка и контроль по UART
// ============================================================
bool tmcInit(TMC2209Stepper& drv, const char* name, int currentMa) {
    Serial.printf("TMC %s: ", name);
    uint8_t conn = drv.test_connection();
    if (conn != 0) {
        Serial.printf("нет ответа (код %u)\n", conn);
        return false;
    }
    drv.GSTAT(0b111);              // сброс флагов reset/drv_err/uv_cp
    drv.pdn_disable(true);         // PDN_UART — только UART, не power-down
    drv.mstep_reg_select(true);    // микрошаг из регистра, MS1/MS2 — адрес
    drv.I_scale_analog(false);     // ток задаётся только по UART, VREF игнорируется
    drv.toff(4);
    drv.blank_time(24);
    drv.microsteps(MICROSTEPS);
    drv.intpol(true);
    drv.rms_current(currentMa);
    drv.en_spreadCycle(false);     // StealthChop — тихий режим
    drv.pwm_autoscale(true);

    uint32_t ds = drv.DRV_STATUS();
    Serial.printf("OK, версия 0x%02X, микрошаг 1/%u, ток %u мА, IFCNT %u",
                  drv.version(), drv.microsteps(), drv.rms_current(), drv.IFCNT());
    if (ds & (1UL << 1))  Serial.print(F(", ПЕРЕГРЕВ"));
    if (ds & (1UL << 0))  Serial.print(F(", предупреждение о температуре"));
    if (ds & (0b1111UL << 2)) Serial.print(F(", КЗ обмотки"));
    if (ds & (0b11UL << 6))   Serial.print(F(", обмотка не подключена"));
    Serial.println();
    return drv.microsteps() == MICROSTEPS;
}

// Раз в 5 с: переподключаем пропавший драйвер (например, 12 В подали
// позже ESP) и переинициализируем после сброса чипа. Во время
// движения шину не трогаем — чтение блокирует loop() на пару мс.
void tmcCheck(TMC2209Stepper& drv, bool& ok, const char* name, int currentMa) {
    if (ok) {
        if (drv.test_connection() != 0) {
            Serial.printf("TMC %s: связь потеряна\n", name);
            ok = false;
        } else if (drv.GSTAT() & 0b001) {
            Serial.printf("TMC %s: был сброс, настраиваю заново\n", name);
            ok = tmcInit(drv, name, currentMa);
        }
    } else if (drv.test_connection() == 0) {   // молча ждём, пока ответит
        ok = tmcInit(drv, name, currentMa);
    }
}

// Диагностика драйвера по сети: GET /tmc. Флаги ola/olb (обмотка не
// подключена) и cs_actual осмысленны только при работающем моторе.
String tmcJson(TMC2209Stepper& drv, bool ok) {
    if (!ok || drv.test_connection() != 0) return "{\"ok\":false}";
    uint32_t ds = drv.DRV_STATUS();
    auto flag = [ds](int n) { return (ds >> n) & 1UL ? "true" : "false"; };
    String j = "{\"ok\":true";
    j += ",\"enabled\":"   + String(drv.isEnabled() ? "true" : "false");
    j += ",\"microsteps\":" + String(drv.microsteps());
    j += ",\"rms_ma\":"    + String(drv.rms_current());
    j += ",\"cs_actual\":" + String((ds >> 16) & 0x1F);
    j += ",\"stealth\":"   + String(flag(30));
    j += ",\"standstill\":" + String(flag(31));
    j += ",\"ola\":" + String(flag(6)) + ",\"olb\":" + String(flag(7));
    j += ",\"s2ga\":" + String(flag(2)) + ",\"s2gb\":" + String(flag(3));
    j += ",\"otpw\":" + String(flag(0)) + ",\"ot\":" + String(flag(1));
    j += ",\"t120\":" + String(flag(8));
    j += ",\"sg_result\":" + String(drv.SG_RESULT());
    j += ",\"ifcnt\":"     + String(drv.IFCNT());
    j += "}";
    return j;
}

void handleTmc() {
    String j = "{\"ra\":" + tmcJson(drvRA, raDrvOk) +
               ",\"dec\":" + tmcJson(drvDEC, decDrvOk) + "}";
    server.send(200, "application/json", j);
}

// Поиск неисправности шины: GET /tmc-scan. Уровень линии в покое
// (0 — узел PDN закорочен на GND) и ответ по каждому адресу 0..3:
// 0 — ответил, 1 — читается 0xFFFFFFFF, 2 — нет ответа.
void handleTmcScan() {
    String j = "{\"line_idle\":" + String(digitalRead(TMC_UART_PIN));
    for (uint8_t addr = 0; addr < 4; addr++) {
        TMC2209Stepper probe(&tmcBus, TMC_RSENSE, addr);
        uint8_t conn = probe.test_connection();
        j += ",\"addr" + String(addr) + "\":{\"conn\":" + String(conn);
        if (conn == 0) j += ",\"version\":" + String(probe.version());
        j += "}";
    }
    j += "}";
    server.send(200, "application/json", j);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n============================"));
    Serial.println(F(" EQ2 Mount Controller v6.0  "));
    Serial.println(F(" RTS2 + LX200/OnStep TCP    "));
    Serial.println(F("============================\n"));

    pinMode(RA_ENABLE_PIN,  OUTPUT); digitalWrite(RA_ENABLE_PIN,  HIGH);
    pinMode(DEC_ENABLE_PIN, OUTPUT); digitalWrite(DEC_ENABLE_PIN, HIGH);

    tmcSerial.begin(TMC_BAUD, SWSERIAL_8N1, TMC_UART_PIN, TMC_UART_PIN, false);
    raDrvOk  = tmcInit(drvRA,  "RA",  RA_CURRENT_MA);
    decDrvOk = tmcInit(drvDEC, "DEC", DEC_CURRENT_MA);
    lastTmcCheck = millis();

    stepperRA .setMaxSpeed(raMaxSpeed);  stepperRA .setAcceleration(raAccel);
    stepperDEC.setMaxSpeed(decMaxSpeed); stepperDEC.setAcceleration(decAccel);

    WiFi.begin(ssid, password);
    Serial.print(F("WiFi connecting"));
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500); Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
        Serial.println("Web UI:   http://" + WiFi.localIP().toString() + "/");
        Serial.println("RTS2:     http://" + WiFi.localIP().toString() + "/api/...");
        Serial.println("LX200:    " + WiFi.localIP().toString() + ":4030");
        Serial.println("OTA:      http://" + WiFi.localIP().toString() + "/update");
    } else {
        Serial.println(F("\nWiFi failed — AP mode: EQ2-Mount / telescope"));
        WiFi.softAP("EQ2-Mount", "telescope");
        Serial.print(F("AP IP: ")); Serial.println(WiFi.softAPIP());
    }

    // Web UI маршруты
    server.on("/",         HTTP_GET, []{ server.send(200,"text/html",htmlPage); });
    server.on("/status",   HTTP_GET, handleStatus);
    server.on("/tracking", HTTP_GET, handleTracking);
    server.on("/ra-speed", HTTP_GET, handleRASpeed);
    server.on("/jog",      HTTP_GET, handleJog);
    server.on("/stop-all", HTTP_GET, handleStopAll);
    server.on("/goto",     HTTP_GET, handleGoTo);
    server.on("/sync",     HTTP_GET, handleSync);
    server.on("/set-home", HTTP_GET, handleSetHome);
    server.on("/park",     HTTP_GET, handlePark);
    server.on("/unpark",   HTTP_GET, handleUnpark);
    server.on("/motor",    HTTP_GET, handleMotor);
    server.on("/estop",    HTTP_GET, handleEStop);
    server.on("/dec-hold",  HTTP_GET, handleDecHold);
    server.on("/tmc",      HTTP_GET, handleTmc);
    server.on("/tmc-scan", HTTP_GET, handleTmcScan);

    // RTS2 JSON API (Stellarium TelescopeClientJsonRts2)
    server.on("/api/devbytype",  HTTP_GET, rts2_devbytype);
    server.on("/api/deviceinfo", HTTP_GET, rts2_deviceinfo);
    server.on("/api/get",        HTTP_GET, rts2_get);
    server.on("/api/cmd",        HTTP_GET, rts2_cmd);

    ElegantOTA.begin(&server);
    server.begin();
    Serial.println(F("HTTP server started."));

    // LX200/OnStep TCP сервер
    lxTcpServer.begin();
    lxTcpServer.setNoDelay(true);
    Serial.println(F("LX200 TCP server started on port 4030.\n"));
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    server.handleClient();
    ElegantOTA.loop();

    const unsigned long now = millis();

    // — Контроль драйверов TMC2209 —
    if (now - lastTmcCheck > 5000 && !isSlewing && !raMoving && !decMoving) {
        lastTmcCheck = now;
        tmcCheck(drvRA,  raDrvOk,  "RA",  RA_CURRENT_MA);
        tmcCheck(drvDEC, decDrvOk, "DEC", DEC_CURRENT_MA);
    }

    // — Трекинг —
    if (trackingEnabled && now - lastTrackUpdate > 10) {
        stepperRA.setSpeed(RA_SIDEREAL_SPEED * raSpeedMult * RA_DIR);
        stepperRA.runSpeed();
        raMoving   = true;
        raStopTime = now;
        lastTrackUpdate = now;
    }

    // — Ручное управление —
    if (raManualSpeed != 0.0f) {
        stepperRA.setSpeed(raManualSpeed * RA_STEPS_PER_DEG * RA_DIR);
        stepperRA.runSpeed();
        raMoving = true; raStopTime = now;
    }
    if (decManualSpeed != 0.0f) {
        stepperDEC.setSpeed(decManualSpeed * DEC_STEPS_PER_DEG * DEC_DIR);
        stepperDEC.runSpeed();
        decMoving = true; decStopTime = now;
    }

    // — GoTo RA —
    if (stepperRA.distanceToGo() != 0) {
        stepperRA.run();
        raMoving = true; raStopTime = now; isSlewing = true;
    } else if (raMoving && !trackingEnabled && raManualSpeed == 0.0f) {
        if (now - raStopTime > MOTOR_OFF_DELAY) {
            enableRA(false); raMoving = false;
        }
    }

    // — GoTo DEC — мотор DEC не выключаем (держит ось) —
    if (stepperDEC.distanceToGo() != 0) {
        stepperDEC.run();
        decMoving = true; decStopTime = now; isSlewing = true;
    } else if (decMoving && decManualSpeed == 0.0f) {
        if (now - decStopTime > MOTOR_OFF_DELAY) {
            decMoving = false;
            if (!decHoldEnabled) {
                enableDEC(false);   // отключаем только если hold выключен
            }
            // если decHoldEnabled=true — мотор остаётся включённым, держит ось
        }
    }

    if (stepperRA.distanceToGo() == 0 && stepperDEC.distanceToGo() == 0)
        isSlewing = false;

    updateCurrentCoords();
    lxHandleTCP();
}

// ============================================================
//  ВЕБ-ОБРАБОТЧИКИ
// ============================================================
void handleStatus() {
    // Тернарный оператор ?"true":"false" внутри строковой конкатенации
    // вызывает ошибку C++11 UDL (user-defined literal): компилятор считает
    // "" началом суффикса. Решение — вынести результат в const char*.
    const char* sTracking = trackingEnabled ? "true" : "false";
    const char* sSlewing  = isSlewing       ? "true" : "false";
    const char* sParked   = isParked        ? "true" : "false";
    const char* sDecHold  = decHoldEnabled  ? "true" : "false";

    String j = "{";
    j += "\"ra_deg\":"   + String(currentRA_deg,  6) + ",";
    j += "\"dec_deg\":"  + String(currentDEC_deg, 6) + ",";
    j += "\"tracking\":" + String(sTracking)         + ",";
    j += "\"slewing\":"  + String(sSlewing)          + ",";
    j += "\"parked\":"   + String(sParked)           + ",";
    j += "\"dec_hold\":" + String(sDecHold)          + ",";
    j += "\"ra_speed\":" + String(raSpeedMult, 3)    + ",";
    j += "\"ra_drv\":"   + String(raDrvOk  ? "true" : "false") + ",";
    j += "\"dec_drv\":"  + String(decDrvOk ? "true" : "false") + ",";
    j += "\"ip\":\""     + WiFi.localIP().toString() + "\"";
    j += "}";
    server.send(200, "application/json", j);
}

void handleTracking() {
    bool en = server.hasArg("enable") && server.arg("enable") == "1";
    trackingEnabled = en;
    enableRA(en);
    if (!en) { stepperRA.stop(); raManualSpeed = 0; }
    server.send(200, "text/plain", "OK");
}

void handleRASpeed() {
    if (server.hasArg("speed"))
        raSpeedMult = server.arg("speed").toFloat();
    server.send(200, "text/plain", "OK");
}

void handleJog() {
    String axis = server.hasArg("axis")  ? server.arg("axis")  : "";
    float  spd  = server.hasArg("speed") ? server.arg("speed").toFloat() : 0;
    if (axis == "ra") {
        raManualSpeed = spd;
        if (spd != 0) { enableRA(true); raMoving = true; raStopTime = millis(); }
        else          { stepperRA.stop(); raStopTime = millis(); }
    } else if (axis == "dec") {
        decManualSpeed = spd;
        if (spd != 0) { enableDEC(true); decMoving = true; decStopTime = millis(); }
        else          { stepperDEC.stop(); decStopTime = millis(); }
    }
    server.send(200, "text/plain", "OK");
}

void handleStopAll() {
    trackingEnabled = false;
    raManualSpeed   = 0;
    decManualSpeed  = 0;
    stepperRA .stop();
    stepperDEC.stop();
    if (decHoldEnabled) {
        enableDEC(true);    // удержание включено — держим ось
    } else {
        enableDEC(false);   // удержание выключено — отключаем
    }
    raStopTime = decStopTime = millis();
    server.send(200, "text/plain", "OK");
}

void handleGoTo() {
    if (server.hasArg("ra") && server.hasArg("dec")) {
        targetRA_deg  = server.arg("ra") .toDouble();
        targetDEC_deg = server.arg("dec").toDouble();
        startGoTo(targetRA_deg, targetDEC_deg);
    }
    server.send(200, "text/plain", "OK");
}

void handleSync() {
    if (server.hasArg("ra") && server.hasArg("dec")) {
        double ra  = server.arg("ra") .toDouble();
        double dec = server.arg("dec").toDouble();
        stepperRA .setCurrentPosition((long)(ra  * RA_STEPS_PER_DEG  * RA_DIR));
        stepperDEC.setCurrentPosition((long)(dec * DEC_STEPS_PER_DEG * DEC_DIR));
        Serial.printf("Sync → RA=%.4f° DEC=%.4f°\n", ra, dec);
    }
    server.send(200, "text/plain", "OK");
}

void handleSetHome() {
    stepperRA .setCurrentPosition(0);
    stepperDEC.setCurrentPosition(0);
    Serial.println(F("Home set (0°,0°)"));
    server.send(200, "text/plain", "OK");
}

void handlePark() {
    isParked = true;
    trackingEnabled = false;
    raManualSpeed = decManualSpeed = 0;
    startGoTo(0, 0);
    server.send(200, "text/plain", "OK");
}

void handleUnpark() {
    isParked = false;
    server.send(200, "text/plain", "OK");
}

void handleMotor() {
    if (!server.hasArg("axis")) {
        server.send(400,"application/json","{\"ok\":false,\"error\":\"no axis\"}");
        return;
    }
    String axis = server.arg("axis");
    float  spd  = server.hasArg("speed") ? server.arg("speed").toFloat() : -1;
    float  acc  = server.hasArg("accel") ? server.arg("accel").toFloat() : -1;
    if (spd < 10 || spd > 15000 || acc < 10 || acc > 10000) {
        server.send(400,"application/json","{\"ok\":false,\"error\":\"out of range\"}");
        return;
    }
    if (axis == "ra") {
        raMaxSpeed = spd; raAccel = acc;
        stepperRA.setMaxSpeed(spd); stepperRA.setAcceleration(acc);
    } else if (axis == "dec") {
        decMaxSpeed = spd; decAccel = acc;
        stepperDEC.setMaxSpeed(spd); stepperDEC.setAcceleration(acc);
    }
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleEStop() {
    trackingEnabled = false;
    raManualSpeed   = 0;
    decManualSpeed  = 0;
    isSlewing       = false;
    stepperRA .stop(); stepperDEC.stop();
    enableRA(false);
    if (decHoldEnabled) {
        enableDEC(true);    // удержание включено — держим ось
    } else {
        enableDEC(false);   // удержание выключено — отключаем
    }
    raMoving = decMoving = false;
    Serial.println(F("!!! EMERGENCY STOP !!!"));
    server.send(200, "text/plain", "OK");
}

// Включение/выключение удержания оси DEC мотором
// GET /dec-hold?enable=1  или  GET /dec-hold?enable=0
void handleDecHold() {
    if (server.hasArg("enable")) {
        decHoldEnabled = (server.arg("enable") == "1");
        if (decHoldEnabled) {
            enableDEC(true);   // сразу включаем мотор
            Serial.println(F("DEC hold: ON"));
        } else {
            enableDEC(false);  // сразу отключаем мотор
            Serial.println(F("DEC hold: OFF"));
        }
    }
    // Пробел перед bool-значением обязателен — без него компилятор
    // трактует \"true\" как суффикс строкового литерала (C++11 UDL).
    const char* val = decHoldEnabled ? "true" : "false";
    String j = String("{\"dec_hold\":") + val + "}";
    server.send(200, "application/json", j);
}

// ============================================================
//  RTS2 JSON API
//  Реализован по исходнику TelescopeClientJsonRts2.cpp из Stellarium
// ============================================================

// GET /api/devbytype?t=2  → ["T0"]
void rts2_devbytype() {
    server.send(200, "application/json", "[\"T0\"]");
}

// GET /api/deviceinfo?d=T0  → {"readonly":false}
void rts2_deviceinfo() {
    server.send(200, "application/json", "{\"readonly\":false}");
}

// GET /api/get?d=T0
// Поля: d.TEL.ra (deg), d.TEL.dec (deg),
//       d.LONGITUD (deg), d.LATITUDE (deg),
//       d.ALTITUDE (m), d.target_distance (deg)
void rts2_get() {
    double dRA   = targetRA_deg  - currentRA_deg;
    double dDEC  = targetDEC_deg - currentDEC_deg;
    double tDist = sqrt(dRA*dRA + dDEC*dDEC);

    String j = "{\"d\":{";
    j += "\"TEL\":{\"ra\":"  + String(currentRA_deg,  6) +
                 ",\"dec\":" + String(currentDEC_deg, 6) + "},";
    j += "\"LONGITUD\":"        + String(obsLongitude_deg, 6) + ",";
    j += "\"LATITUDE\":"        + String(obsLatitude_deg,  6) + ",";
    j += "\"ALTITUDE\":"        + String(obsAltitude_m,    2) + ",";
    j += "\"target_distance\":" + String(tDist,            6);
    j += "}}";
    server.send(200, "application/json", j);
}

// GET /api/cmd?d=T0&c=move+RA_deg+DEC_deg
void rts2_cmd() {
    if (!server.hasArg("c")) {
        server.send(400, "application/json", "{\"ret\":1}");
        return;
    }
    String cmd = server.arg("c");
    bool isTLE  = cmd.startsWith("move_tle");
    bool isMove = cmd.startsWith("move") && !isTLE;

    if (isMove) {
        int s1 = cmd.indexOf(' ');
        int s2 = cmd.indexOf(' ', s1+1);
        if (s1 < 0 || s2 < 0) {
            server.send(200,"application/json","{\"ret\":1}"); return;
        }
        double ra  = cmd.substring(s1+1, s2).toDouble();
        double dec = cmd.substring(s2+1).toDouble();
        targetRA_deg  = ra;
        targetDEC_deg = dec;
        startGoTo(ra, dec);
        server.send(200, "application/json", "{\"ret\":0}");
    } else if (isTLE) {
        // TLE для спутников — ESP8266 орбиту не считает,
        // сообщаем успех чтобы Stellarium не показывал ошибку
        server.send(200, "application/json", "{\"ret\":0}");
    } else {
        server.send(200, "application/json", "{\"ret\":1}");
    }
}

// ============================================================
//  LX200 / OnStep TCP HANDLER
// ============================================================
void lxHandleTCP() {
    if (lxTcpServer.hasClient()) {
        if (!lxClientConnected || !lxTcpClient.connected()) {
            if (lxTcpClient) lxTcpClient.stop();
            lxTcpClient       = lxTcpServer.available();
            lxClientConnected = true;
            lxTcpBuf          = "";
            Serial.println(F("LX200 TCP: client connected"));
        } else {
            WiFiClient extra = lxTcpServer.available();
            extra.stop();
        }
    }
    if (!lxClientConnected) return;
    if (!lxTcpClient.connected()) {
        lxTcpClient.stop();
        lxClientConnected = false;
        lxTcpBuf          = "";
        Serial.println(F("LX200 TCP: client disconnected"));
        return;
    }
    while (lxTcpClient.available()) {
        char c = lxTcpClient.read();
        if (c == 0x06) {
            // ACK (0x06) — пинг lx200driver, ответ 'P' (polar aligned)
            lxTcpClient.print('P');
            Serial.println(F("LX< ACK → P"));
            lxInCommand = false;
            lxTcpBuf    = "";
        } else if (c == ':') {
            // ':' — начало команды ТОЛЬКО если мы не внутри команды.
            // Если lxInCommand=true, значит ':' является частью аргумента
            // (например :Sr22:01:48# или :Sd-13*14:15#) — добавляем в буфер.
            if (!lxInCommand) {
                lxTcpBuf    = "";
                lxInCommand = true;
            } else {
                if (lxTcpBuf.length() < 32) lxTcpBuf += c;
            }
        } else if (c == '#') {
            // '#' — конец команды
            if (lxInCommand) {
                lxProcessCommand(lxTcpBuf);
            }
            lxTcpBuf    = "";
            lxInCommand = false;
        } else {
            // Обычный символ — добавляем в буфер только если внутри команды
            if (lxInCommand && lxTcpBuf.length() < 32)
                lxTcpBuf += c;
        }
    }
}

// ============================================================
//  LX200 ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================

// Отправить ответ клиенту + дублировать в Serial
void lxSend(const String& s) {
    if (lxClientConnected && lxTcpClient.connected())
        lxTcpClient.print(s);
    Serial.print(F("  LX> ")); Serial.println(s);
}

// RA градусы → "HH:MM:SS#"  или  "HH:MM.T#"
// lx200driver использует getCommandSexa(:GR#) → ожидает HH:MM:SS или HH:MM.T
String lxFormatRA(double deg) {
    double h  = fmod(deg / 15.0, 24.0);
    if (h < 0.0) h += 24.0;
    int    hh  = (int)h;
    double rem = (h - hh) * 60.0;
    int    mm  = (int)rem;
    int    ss  = (int)((rem - mm) * 60.0);
    char   buf[20];
    if (lxHighPrecision)
        sprintf(buf, "%02d:%02d:%02d#", hh, mm, ss);
    else
        sprintf(buf, "%02d:%02d.%01d#", hh, mm, ss/6);
    return String(buf);
}

// RA градусы → "HH:MM:SS.SSSS#"  (для :GRH# / :GrH#)
String lxFormatRA_HP(double deg) {
    double h  = fmod(deg / 15.0, 24.0);
    if (h < 0.0) h += 24.0;
    int    hh  = (int)h;
    double rem = (h - hh) * 60.0;
    int    mm  = (int)rem;
    double ss  = (rem - mm) * 60.0;
    char   buf[24];
    sprintf(buf, "%02d:%02d:%07.4f#", hh, mm, ss);
    return String(buf);
}

// DEC градусы → "+DD*MM:SS#"  или  "+DD*MM#"
// lx200driver getCommandSexa(:GD#)
String lxFormatDEC(double deg) {
    char   sign = (deg >= 0) ? '+' : '-';
    double a    = fabs(deg);
    int    dd   = (int)a;
    double rem  = (a - dd) * 60.0;
    int    mm   = (int)rem;
    int    ss   = (int)((rem - mm) * 60.0);
    char   buf[20];
    if (lxHighPrecision)
        sprintf(buf, "%c%02d*%02d:%02d#", sign, dd, mm, ss);
    else
        sprintf(buf, "%c%02d*%02d#", sign, dd, mm);
    return String(buf);
}

// DEC градусы → "+DD*MM:SS.SSS#"  (для :GDH# и высокоточных)
String lxFormatDEC_HP(double deg) {
    char   sign = (deg >= 0) ? '+' : '-';
    double a    = fabs(deg);
    int    dd   = (int)a;
    double rem  = (a - dd) * 60.0;
    int    mm   = (int)rem;
    double ss   = (rem - mm) * 60.0;
    char   buf[24];
    sprintf(buf, "%c%02d*%02d:%06.3f#", sign, dd, mm, ss);
    return String(buf);
}

// Azimuth/Altitude → "DDD*MM'SS.SSS#"
String lxFormatAz(double deg) {
    double a  = fmod(fabs(deg), 360.0);
    int    dd = (int)a;
    double r  = (a - dd) * 60.0;
    int    mm = (int)r;
    double ss = (r - mm) * 60.0;
    char   buf[24];
    sprintf(buf, "%03d*%02d'%06.3f#", dd, mm, ss);
    return String(buf);
}

// Широта обсерватории → "+DD*MM:SS.SSS#"
// lx200driver getSiteLatitude(:Gt#) → sDD*MM:SS.SSS
String lxFormatLatitude(double deg) {
    char   sign = (deg >= 0) ? '+' : '-';
    double a    = fabs(deg);
    int    dd   = (int)a;
    double rem  = (a - dd) * 60.0;
    int    mm   = (int)rem;
    double ss   = (rem - mm) * 60.0;
    char   buf[24];
    sprintf(buf, "%c%02d*%02d:%06.3f#", sign, dd, mm, ss);
    return String(buf);
}

// Долгота обсерватории → "sDDD*MM:SS.SSS#"
// lx200driver getSiteLongitude(:Gg#): East = отрицательная!
// Наша obsLongitude_deg = East+, инвертируем для LX200
String lxFormatLongitude(double deg) {
    double lx   = -deg;   // East→West sign flip
    char   sign = (lx >= 0) ? '+' : '-';
    double a    = fabs(lx);
    int    dd   = (int)a;
    double rem  = (a - dd) * 60.0;
    int    mm   = (int)rem;
    double ss   = (rem - mm) * 60.0;
    char   buf[24];
    sprintf(buf, "%c%03d*%02d:%06.3f#", sign, dd, mm, ss);
    return String(buf);
}

// Парсинг RA из "HH:MM:SS" / "HH:MM:SS.SSS" / "HH:MM.T" → градусы
// lx200driver setObjectRA использует тот же формат
double lxParseRA(const String& s) {
    int c1 = s.indexOf(':');
    if (c1 < 0) return 0.0;
    int    hh = s.substring(0, c1).toInt();
    int    c2 = s.indexOf(':', c1+1);
    double mm, ss = 0.0;
    if (c2 > 0) {
        mm = s.substring(c1+1, c2).toDouble();
        ss = s.substring(c2+1).toDouble();
    } else {
        mm = s.substring(c1+1).toDouble();
    }
    return (hh + mm/60.0 + ss/3600.0) * 15.0;
}

// Парсинг DEC из "[s]DD*MM:SS" / "[s]DD*MM:SS.SSS" → градусы
double lxParseDEC(const String& s) {
    int  star = s.indexOf('*');
    if (star < 0) return s.toDouble();
    int  sgn  = (s.charAt(0) == '-') ? -1 : 1;
    int  st   = (s.charAt(0) == '+' || s.charAt(0) == '-') ? 1 : 0;
    int  dd   = s.substring(st, star).toInt();
    int  colon= s.indexOf(':', star+1);
    int  mm;
    double ss = 0.0;
    if (colon > 0) {
        mm = s.substring(star+1, colon).toInt();
        ss = s.substring(colon+1).toDouble();
    } else {
        mm = s.substring(star+1).toInt();
    }
    return sgn * (dd + mm/60.0 + ss/3600.0);
}

// ============================================================
//  LX200 / OnStep ОБРАБОТЧИК КОМАНД
//
//  Источники:
//    • lx200driver.h  (INDI — все макросы и прототипы)
//    • OnStepListCmd.txt
//    • lx200_server.py (эмулятор)
//
//  Формат: команда приходит БЕЗ ведущего ':' и финального '#'
// ============================================================
void lxProcessCommand(const String& cmd) {
    if (cmd.length() == 0) return;
    Serial.print(F("LX< :")); Serial.print(cmd); Serial.println('#');

    // ── Precision (:P# / :U#) ───────────────────────────────────────────
    if (cmd == "P") {
        // :P# — Toggle precision high ↔ low  (ответа нет)
        lxHighPrecision = !lxHighPrecision;

    } else if (cmd == "U") {
        // :U# — Force high precision  (ответа нет)
        lxHighPrecision = true;

    // ── Firmware ID (:GVP# :GVN# :GVD# :GVT# :GVM# :GVC#) ──────────────
    // lx200driver: getProductName, getVersionNumber, getVersionDate,
    //              getVersionTime, getFullVersion
    } else if (cmd == "GVP") {
        lxSend("OnStep#");
    } else if (cmd == "GVN") {
        lxSend("10.24b#");
    } else if (cmd == "GVD") {
        lxSend("May 10 2026#");
    } else if (cmd == "GVT") {
        lxSend("00:00:00#");
    } else if (cmd == "GVM") {
        lxSend("EQ2 ESP8266#");
    } else if (cmd == "GVC") {
        lxSend("EQ2#");

    // ── Текущие RA/DEC (:GR# :GRH# :GD# :GDH#) ─────────────────────────
    // lx200driver: getLX200RA(:GR#), getLX200DEC(:GD#)
    } else if (cmd == "GR") {
        lxSend(lxFormatRA(currentRA_deg));
    } else if (cmd == "GRH") {
        lxSend(lxFormatRA_HP(currentRA_deg));
    } else if (cmd == "GD") {
        lxSend(lxFormatDEC(currentDEC_deg));
    } else if (cmd == "GDH") {
        lxSend(lxFormatDEC_HP(currentDEC_deg));

    // ── Altitude / Azimuth (:GA# :GAH# :GZ# :GZH#) ──────────────────────
    // lx200driver: getLX200Alt(:GA#), getLX200Az(:GZ#)
    } else if (cmd == "GA" || cmd == "GAH") {
        lxSend(lxFormatDEC_HP(currentDEC_deg));
    } else if (cmd == "GZ" || cmd == "GZH") {
        lxSend(lxFormatAz(currentRA_deg));

    // ── Target RA/DEC (:Gr# :GrH# :Gd# :GdH# :Gal# :GaH# :Gz# :GzH#) ──
    // lx200driver: getObjectRA(:Gr#), getObjectDEC(:Gd#)
    } else if (cmd == "Gr") {
        lxSend(lxFormatRA(targetRA_deg));
    } else if (cmd == "GrH") {
        lxSend(lxFormatRA_HP(targetRA_deg));
    } else if (cmd == "Gd") {
        lxSend(lxFormatDEC_HP(targetDEC_deg));
    } else if (cmd == "GdH") {
        lxSend(lxFormatDEC_HP(targetDEC_deg));
    } else if (cmd == "Gal" || cmd == "GaH") {
        lxSend(lxFormatDEC_HP(targetDEC_deg));
    } else if (cmd == "Gz" || cmd == "GzH") {
        lxSend(lxFormatAz(targetRA_deg));

    // ── Set target (:SrHH:MM:SS# / :Sd[s]DD*MM:SS#) ─────────────────────
    // lx200driver: setObjectRA(:Sr..#), setObjectDEC(:Sd..#)
    // Возвращает: 0 on failure, 1 on success
    } else if (cmd.startsWith("Sr")) {
        targetRA_deg = lxParseRA(cmd.substring(2));
        lxSend("1");
        Serial.printf("  [LX] Target RA=%.4f°\n", targetRA_deg);

    } else if (cmd.startsWith("Sd")) {
        targetDEC_deg = lxParseDEC(cmd.substring(2));
        lxSend("1");
        Serial.printf("  [LX] Target DEC=%.4f°\n", targetDEC_deg);

    // ── Tracking rate (:GT#) ─────────────────────────────────────────────
    // lx200driver: getTrackFreq(:GT#) → double
    // OnStep: 60.1642 когда sidereal, 0.0 когда выкл
    } else if (cmd == "GT") {
        lxSend(trackingEnabled ? "60.1642#" : "0.0000#");

    // ── GoTo (:MS# :MA# :MD# :MN# :MNe# :MNw# :MP#) ────────────────────
    // lx200driver: Slew(:MS#) → 0 ok, иначе ошибка
    // OnStep коды: 0=ok 1=below horizon 2=overhead 3=standby
    //              4=parked 5=in progress 6=limits 7=fault 8=motion 9=error
    } else if (cmd == "MS") {
        if (isParked)        { lxSend("4"); }
        else if (isSlewing)  { lxSend("5"); }
        else { startGoTo(targetRA_deg, targetDEC_deg); lxSend("0"); }

    } else if (cmd == "MA") {
        if (isParked) { lxSend("4"); }
        else { startGoTo(targetRA_deg, targetDEC_deg); lxSend("0"); }

    } else if (cmd == "MD") {
        // 0=East, 1=West, 2=Unknown
        lxSend("2");

    } else if (cmd == "MN" || cmd == "MNe" || cmd == "MNw") {
        if (isParked) { lxSend("4"); }
        else { startGoTo(targetRA_deg, targetDEC_deg); lxSend("0"); }

    } else if (cmd == "MP") {
        lxSend(isParked ? "4" : "0");

    // ── Sync (:CM# :CS#) ─────────────────────────────────────────────────
    // lx200driver: Sync(:CM#) → "N/A#" on success, "En#" on failure
    } else if (cmd == "CM" || cmd == "CS") {
        stepperRA .setCurrentPosition(
            (long)(targetRA_deg  * RA_STEPS_PER_DEG  * RA_DIR));
        stepperDEC.setCurrentPosition(
            (long)(targetDEC_deg * DEC_STEPS_PER_DEG * DEC_DIR));
        lxSend("N/A#");
        Serial.printf("  [LX] Synced RA=%.4f° DEC=%.4f°\n",
                      targetRA_deg, targetDEC_deg);

    // ── Alignment (:A[n]# :A+# :AW#) ────────────────────────────────────
    } else if (cmd == "AW") {
        lxSend("1");
    } else if (cmd == "A+") {
        lxSend("1");
    } else if (cmd.startsWith("A") && cmd.length() == 2
               && isDigit(cmd.charAt(1))) {
        lxSend("1");

    // ── Slew indicator (:D#) ─────────────────────────────────────────────
    // lx200driver: isSlewComplete(:D#) / checkLX200EquatorialFormat
    // OnStepListCmd: "\0x7f#" если движется, "#" если стоит
    } else if (cmd == "D") {
        lxSend(isSlewing ? "\x7f#" : "#");

    // ── Halt (:Q# :Qe# :Qw# :Qn# :Qs#) ─────────────────────────────────
    // lx200driver: abortSlew(:Q#), HaltMovement(:Qe#/:Qw#/:Qn#/:Qs#)
    // Ответа нет
    } else if (cmd == "Q" || cmd == "Qe" || cmd == "Qw"
                          || cmd == "Qn" || cmd == "Qs") {
        bool doRA  = (cmd=="Q" || cmd=="Qe" || cmd=="Qw");
        bool doDEC = (cmd=="Q" || cmd=="Qn" || cmd=="Qs");
        if (doRA) {
            raManualSpeed = 0;
            if (!trackingEnabled) { stepperRA.stop(); raStopTime=millis(); }
        }
        if (doDEC) {
            decManualSpeed = 0;
            stepperDEC.stop();
            enableDEC(true);  // держим DEC включённым
        }
        if (cmd == "Q") isSlewing = false;
        // нет ответа

    // ── Tracking (:Te# :Td# :TQ# :TS# :TK# :TL# :T+# :T-# :TR#
    //             :To# :Tr# :Tn# :T1# :T2#) ──────────────────────────────
    // lx200driver: selectTrackingMode
    } else if (cmd == "Te") {
        trackingEnabled = true; enableRA(true); lxSend("1");
    } else if (cmd == "Td") {
        trackingEnabled = false; stepperRA.stop(); raManualSpeed=0; lxSend("1");
    } else if (cmd == "TQ") {
        raSpeedMult = 1.0f; lxSend("1");
    } else if (cmd == "TS") {
        raSpeedMult = 60.0f   / 60.1642f; lxSend("1");
    } else if (cmd == "TK") {
        raSpeedMult = 60.136f / 60.1642f; lxSend("1");
    } else if (cmd == "TL") {
        raSpeedMult = 57.9f   / 60.1642f; lxSend("1");
    } else if (cmd == "T+") {
        raSpeedMult += 0.02f / 60.1642f;  lxSend("1");
    } else if (cmd == "T-") {
        raSpeedMult -= 0.02f / 60.1642f;  lxSend("1");
    } else if (cmd == "TR") {
        raSpeedMult = 1.0f; lxSend("1");
    } else if (cmd=="To" || cmd=="Tr" || cmd=="Tn") {
        lxSend("1");
    } else if (cmd=="T1" || cmd=="T2") {
        lxSend("1");

    // ── Park / Home (:hP# :hR# :hF# :hC# :hQ# :hA[n]#) ─────────────────
    // lx200driver: slewToPark(:hP#), gpsWakeUp/Sleep (:hW#/:hN#)
    } else if (cmd == "hP") {
        if (!isParked) {
            isParked=true; trackingEnabled=false;
            raManualSpeed=decManualSpeed=0;
            startGoTo(0,0); lxSend("1");
        } else { lxSend("0"); }

    } else if (cmd == "hR") {
        if (isParked) { isParked=false; enableDEC(true); lxSend("1"); }
        else          { lxSend("0"); }

    } else if (cmd == "hF") {
        stepperRA .setCurrentPosition(0);
        stepperDEC.setCurrentPosition(0);
        isParked = false;
        // нет ответа

    } else if (cmd == "hC") {
        startGoTo(0, 0);
        // нет ответа

    } else if (cmd == "hQ") {
        lxSend("1");  // "set park pos" — запоминаем текущую (в ОЗУ)

    } else if (cmd.startsWith("hA")) {
        // :hAn# — auto home state (нет ответа)

    // ── Manual move (:Me# :Mw# :Mn# :Ms# :Mp#) ──────────────────────────
    // lx200driver: MoveTo(:Me#/:Mw#/:Mn#/:Ms#)  (нет ответа)
    } else if (cmd == "Me") {
        enableRA(true);  raManualSpeed  =  lxGuideRate;
        raMoving=true;   raStopTime=millis();
    } else if (cmd == "Mw") {
        enableRA(true);  raManualSpeed  = -lxGuideRate;
        raMoving=true;   raStopTime=millis();
    } else if (cmd == "Mn") {
        enableDEC(true); decManualSpeed =  lxGuideRate;
        decMoving=true;  decStopTime=millis();
    } else if (cmd == "Ms") {
        enableDEC(true); decManualSpeed = -lxGuideRate;
        decMoving=true;  decStopTime=millis();
    } else if (cmd == "Mp") {
        // spiral search — не реализовано

    // ── Guide rate (:RG# :RC# :RM# :RF# :RS# :Rn#) ──────────────────────
    // lx200driver: setSlewMode / setSlewSpeed  (нет ответа)
    } else if (cmd == "RG") { lxGuideRate = SIDEREAL_RATE * 1.0f;
    } else if (cmd == "RC") { lxGuideRate = SIDEREAL_RATE * 8.0f;
    } else if (cmd == "RM") { lxGuideRate = SIDEREAL_RATE * 20.0f;
    } else if (cmd == "RF") { lxGuideRate = SIDEREAL_RATE * 48.0f;
    } else if (cmd == "RS") { lxGuideRate = SIDEREAL_RATE * 120.0f;
    } else if (cmd.startsWith("R") && cmd.length()==2 && isDigit(cmd.charAt(1))) {
        const float tbl[] = {0.5f,1.f,2.f,4.f,8.f,16.f,24.f,48.f,120.f,240.f};
        lxGuideRate = SIDEREAL_RATE * tbl[cmd.charAt(1)-'0'];

    // ── Pulse guide (:Mgd[n]# :MGd[n]#) ─────────────────────────────────
    // lx200driver: SendPulseCmd  → 0/1
    } else if (cmd.startsWith("Mgd") || cmd.startsWith("MGd")) {
        lxSend("1");

    // ── Pulse guide rate (:GX90#) ────────────────────────────────────────
    } else if (cmd == "GX90") {
        char buf[12];
        sprintf(buf, "%.2f#", lxGuideRate / SIDEREAL_RATE);
        lxSend(String(buf));

    // ── Status (:GU# :Gu# :GW# :Gm#) ────────────────────────────────────
    // lx200driver: нет прямого макроса, но checkLX200EquatorialFormat/GW
    } else if (cmd == "GU") {
        // P/p=park, T/I=tracking, S/s=slewing, N=not aligned
        String s = "";
        s += (isParked        ? "P" : "p");
        s += (trackingEnabled ? "T" : "I");
        s += (isSlewing       ? "S" : "s");
        s += "N#";
        lxSend(s);

    } else if (cmd == "Gu") {
        // bit-packed: bit0=tracking, bit1=slewing, bit2=parked
        uint8_t b = 0;
        if (trackingEnabled) b |= 0x01;
        if (isSlewing)       b |= 0x02;
        if (isParked)        b |= 0x04;
        char buf[6]; sprintf(buf, "%02X#", b);
        lxSend(String(buf));

    } else if (cmd == "GW") {
        // lx200_server.py: "A#" Altazimuth mode
        // OnStep формат: N<Track><Park>#
        char buf[6];
        sprintf(buf, "N%c%c#",
                trackingEnabled ? 'S' : 'N',
                isParked        ? 'P' : 'N');
        lxSend(String(buf));

    } else if (cmd == "Gm") {
        // lx200driver нет прямого; OnStep: E#/W#/N#
        if (isParked) lxSend("N#");
        else lxSend(currentRA_deg < 180.0 ? "E#" : "W#");

    // ── Site (:Gt# :GtH# :Gg# :GgH# :Gv#) ──────────────────────────────
    // lx200driver: getSiteLatitude(:Gt#), getSiteLongitude(:Gg#)
    } else if (cmd == "Gt" || cmd == "GtH") {
        lxSend(lxFormatLatitude(obsLatitude_deg));
    } else if (cmd == "Gg" || cmd == "GgH") {
        lxSend(lxFormatLongitude(obsLongitude_deg));
    } else if (cmd == "Gv") {
        char buf[14]; sprintf(buf, "+%.1f#", obsAltitude_m);
        lxSend(String(buf));

    // ── Time / Date (:GL# :GLH# :GG# :GS# :GSH# :GC# :Ga# :Gc#) ────────
    // lx200driver: getLocalTime24(:GL#), getLocalTime12(:Ga#),
    //              getSDTime(:GS#), getCalendarDate(:GC#)
    } else if (cmd == "GL" || cmd == "GLH") {
        unsigned long s  = millis()/1000;
        unsigned long hh = (s/3600)%24, mm=(s%3600)/60, ss2=s%60;
        char buf[24];
        if (cmd == "GLH") sprintf(buf, "%02lu:%02lu:%02lu.0000#", hh, mm, ss2);
        else              sprintf(buf, "%02lu:%02lu:%02lu#",       hh, mm, ss2);
        lxSend(String(buf));

    } else if (cmd == "GG") {
        // lx200driver: getUTCOffset(:GG#) → int
        lxSend("+00:00#");

    } else if (cmd == "GS" || cmd == "GSH") {
        // lx200driver: getSDTime(:GS#)
        unsigned long s  = millis()/1000;
        unsigned long hh = (s/3600)%24, mm=(s%3600)/60, ss2=s%60;
        char buf[24];
        if (cmd == "GSH") sprintf(buf, "%02lu:%02lu:%02lu.00#", hh, mm, ss2);
        else              sprintf(buf, "%02lu:%02lu:%02lu#",     hh, mm, ss2);
        lxSend(String(buf));

    } else if (cmd == "GC") {
        // lx200driver: getCalendarDate(:GC#) → MM/DD/YY
        lxSend("01/01/00#");

    } else if (cmd == "Ga") {
        // lx200driver: getLocalTime12(:Ga#)
        unsigned long s  = millis()/1000;
        unsigned long hh = (s/3600)%12; if(!hh) hh=12;
        unsigned long mm = (s%3600)/60, ss2=s%60;
        char buf[14]; sprintf(buf, "%02lu:%02lu:%02lu#", hh, mm, ss2);
        lxSend(String(buf));

    } else if (cmd == "Gc") {
        lxSend("24#");

    // ── Set time/date/UTC (:SC# :SG# :SL#) ──────────────────────────────
    // lx200driver: setCalenderDate, setUTCOffset, setLocalTime → 0/1
    } else if (cmd.startsWith("SC") || cmd.startsWith("SG")
            || cmd.startsWith("SL")) {
        lxSend("1");

    // ── Site names (:GM# :GN# :GO# :GP# :SM# :SN# :SO# :SP# :W[n]#) ────
    // lx200driver: getSiteName, setSiteName, selectSite
    } else if (cmd == "GM") { lxSend("Home#");
    } else if (cmd == "GN") { lxSend("Site2#");
    } else if (cmd == "GO") { lxSend("Site3#");
    } else if (cmd == "GP") { lxSend("Site4#");
    } else if (cmd.startsWith("SM") || cmd.startsWith("SN")
            || cmd.startsWith("SO") || cmd.startsWith("SP")) {
        lxSend("1");
    } else if (cmd.startsWith("W") && cmd.length()==2
               && isDigit(cmd.charAt(1))) {
        // :W[n]# — ничего не возвращает

    // ── Limits (:Gh# :Go# :Sh# :So# :GXE[m]#) ───────────────────────────
    // lx200driver: getMaxElevationLimit(:Go#), getMinElevationLimit(:Gh#)
    //              setMaxElevationLimit(:So#), setMinElevationLimit(:Sh#)
    } else if (cmd == "Gh") { lxSend("-10*#");
    } else if (cmd == "Go") { lxSend("90*#");
    } else if (cmd.startsWith("Sh") || cmd.startsWith("So")) { lxSend("1");
    } else if (cmd.startsWith("GXE")) { lxSend("0#");

    // ── Sensors (:GX9A# :GX9B# :GX9C# :GX9E# :GX9F#) ───────────────────
    // OnStepListCmd: temperature, pressure, humidity, dewpoint, MCU temp
    } else if (cmd == "GX9A") { lxSend("+20.0#");
    } else if (cmd == "GX9B") { lxSend("+1013.0#");
    } else if (cmd == "GX9C") { lxSend("+50.0#");
    } else if (cmd == "GX9E") { lxSend("+10.0#");
    } else if (cmd == "GX9F") { lxSend("+35.0#");

    // ── GX91 (PEC analog) ─────────────────────────────────────────────────
    } else if (cmd == "GX91") { lxSend("0#");

    // ── GX0n (align settings) ────────────────────────────────────────────
    // OnStepListCmd: :GX0[n]# → get align setting
    } else if (cmd.startsWith("GX0")) { lxSend("0#");

    // ── GX4n (axis angles) ───────────────────────────────────────────────
    // OnStepListCmd: :GX4[n]# → (0,1)=DDD:MM:SS, (2,3)=n.nnnnnn, (4,5)=counts
    } else if (cmd.startsWith("GX4")) {
        char n = cmd.length() > 3 ? cmd.charAt(3) : '0';
        if (n=='0' || n=='1') { lxSend("000:00:00#"); }
        else if (n=='2' || n=='3') { lxSend("0.000000#"); }
        else { lxSend("0#"); }

    // ── GXA (axis/driver config) ─────────────────────────────────────────
    // lx200driver / OnStepListCmd: :GXA[n]# / :SXA[n]#
    } else if (cmd.startsWith("GXA")) { lxSend("0#");
    } else if (cmd.startsWith("SXA")) { lxSend("1");

    // ── GXS (axis servo delta) ───────────────────────────────────────────
    } else if (cmd.startsWith("GXS")) { lxSend("0,0#");

    // ── GXU (stepper driver status) ──────────────────────────────────────
    } else if (cmd.startsWith("GXU")) { lxSend("0#");

    // ── GXGA / GXGO (GPIO) ───────────────────────────────────────────────
    } else if (cmd == "GXGA") { lxSend("0#");
    } else if (cmd == "GXGO") { lxSend("0#");

    // ── GX98 (rotator availability) ──────────────────────────────────────
    // OnStepListCmd: D#=rotate+derotate, R#=rotate only, N#=none
    } else if (cmd == "GX98") { lxSend("N#");

    // ── GXF / GXTD / GXTR (frequencies/offsets) ─────────────────────────
    } else if (cmd.startsWith("GXF")) { lxSend("0#");
    } else if (cmd == "GXTD")         { lxSend("0.000000#");
    } else if (cmd == "GXTR")         { lxSend("0.000000#");

    // ── Time/date status (:GX80# :GX81# :GX89#) ─────────────────────────
    } else if (cmd == "GX80") {
        unsigned long s=millis()/1000;
        char buf[20];
        sprintf(buf,"%02lu:%02lu:%02lu.00#",(s/3600)%24,(s%3600)/60,s%60);
        lxSend(String(buf));
    } else if (cmd == "GX81") { lxSend("01/01/00#");
    } else if (cmd == "GX89") { lxSend("0");

    // ── PEC заглушки (:VH# :VR# :Vr# :VS# :VW# :WR+# :WR-#
    //                 :GXE6# :GXE7# :GXE8#) ──────────────────────────────
    // OnStepListCmd: PEC functions — ESP8266 не реализует PEC
    } else if (cmd == "VH")  { lxSend("0#");
    } else if (cmd.startsWith("VR") || cmd.startsWith("Vr")) { lxSend("0,0#");
    } else if (cmd == "VS")  { lxSend("0.0#");
    } else if (cmd == "VW")  { lxSend("0#");
    } else if (cmd == "WR+" || cmd == "WR-") { lxSend("1");
    } else if (cmd == "GXE6" || cmd == "GXE7" || cmd == "GXE8") { lxSend("0#");

    // ── Library заглушки (:LI# :LR# :LB# :LN# :LC# :LD# :LL# :Lo# :LW#)
    // OnStepListCmd: object library — ESP8266 не реализует каталог
    } else if (cmd == "LI") {
        lxSend("#");
    } else if (cmd == "LR") {
        lxSend("#");
    } else if (cmd == "LB" || cmd == "LN" || cmd.startsWith("LC")
            || cmd == "LD"  || cmd == "LL" || cmd.startsWith("LIG")) {
        // нет ответа
    } else if (cmd.startsWith("Lo")) {
        lxSend("1");
    } else if (cmd.startsWith("LW")) {
        lxSend("1");

    // ── Rotator заглушки (:rA# :rT# :rI# :rM# :rD# :rb# :rQ# …) ────────
    // OnStepListCmd / lx200driver: rotator absent on this mount
    } else if (cmd == "rA") {
        lxSend("0");   // no rotator
    } else if (cmd == "rT") {
        lxSend("idle#");
    } else if (cmd == "rI") {
        lxSend("0#");
    } else if (cmd == "rM") {
        lxSend("360#");
    } else if (cmd == "rD") {
        lxSend("0.0#");
    } else if (cmd == "rG") {
        lxSend("+000*00#");
    } else if (cmd.startsWith("rb")) {
        lxSend("0#");
    } else if (cmd == "rQ" || cmd == "rZ" || cmd == "rF" || cmd == "rC"
            || cmd == "rP"  || cmd == "rR" || cmd == "rW" || cmd == "rc"
            || cmd.startsWith("r+") || cmd.startsWith("r-")) {
        // нет ответа для этих rotator-команд

    // ── System / Baud / Reset ────────────────────────────────────────────
    // lx200driver: setLX200Debug, SB
    } else if (cmd.startsWith("SB")) {
        lxSend("1");

    } else if (cmd == "GE") {
        lxSend("00#");

    } else if (cmd == "ERESET") {
        Serial.println(F("  [LX] MCU reset requested"));
        delay(500); ESP.restart();

    } else if (cmd == "ENVRESET") {
        Serial.println(F("  [LX] Flash wipe (not implemented)"));

    } else if (cmd == "ESPFLASH") {
        Serial.println(F("  [LX] Flash mode requested"));
        lxSend("1");

    // ── Reticle (:B+# :B-#) ─────────────────────────────────────────────
    } else if (cmd == "B+" || cmd == "B-") {
        // нет ответа

    // ── SEO ──────────────────────────────────────────────────────────────
    } else if (cmd == "SEO") {
        lxSend("1");

    // ── Неизвестная команда ──────────────────────────────────────────────
    } else {
        Serial.printf("  [LX] Unknown: %s\n", cmd.c_str());
    }
}
