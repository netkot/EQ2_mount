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
//
//  v6.1: модель координат RA = LST − HA (NTP), стороны опоры,
//        GoTo с доводкой, трекинг/джог отделены от run() — см. README
// ============================================================

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <AccelStepper.h>
#include <ElegantOTA.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <time.h>
#include <sys/time.h>
#include <coredecls.h>

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
const double DEC_STEPS_PER_DEG =
    (STEPS_PER_REV * DEC_GEAR_RATIO) / 360.0;   // 44.444 (не округлять — 1% ошибки)

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

// Механический часовой угол оси RA в домашнем положении (противовес вниз,
// труба на полюс), градусы. −90: из дома DEC− опускает трубу на восток,
// +90 — на запад. На монтировке проверено: восток.
const double HOME_HA_DEG  = -90.0;
// GoTo ниже этой высоты над горизонтом не выполняется, градусы
const double GOTO_MIN_ALT = 0.0;

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
bool   raSlew          = false;   // ось RA идёт по moveTo()/run()
bool   decSlew         = false;   // ось DEC идёт по moveTo()/run()
bool   gotoPending     = false;   // GoTo ждёт доводки по RA
int    gotoPass        = 0;

// Источник времени: 0 — не задано, 1 — вручную (браузер/LX200), 2 — NTP
uint8_t timeSource     = 0;
double  lxUtcOffsetH   = 0.0;     // LX200 :SG — сколько часов прибавить к местному, чтобы получить UTC
long    lxLocalSecs    = -1;      // LX200 :SL — местное время, с от полуночи

// Координаты на эпоху даты (RA = LST − HA), градусы
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

double wrap360(double a) { a = fmod(a, 360.0); return a < 0.0 ? a + 360.0 : a; }
double wrap180(double a) { return wrap360(a + 180.0) - 180.0; }

double unixTimeNow() {
    timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

void setClock(double unixSec, uint8_t src) {
    timeval tv;
    tv.tv_sec  = (time_t)unixSec;
    tv.tv_usec = (suseconds_t)((unixSec - floor(unixSec)) * 1e6);
    settimeofday(&tv, nullptr);
    timeSource = src;
    Serial.printf("Time set (%s): %.0f\n", src == 2 ? "NTP" : "manual", unixSec);
}

// Местное звёздное время, градусы (GMST, без нутации — ошибка < 1″ времени)
double lstDeg() {
    double d = (unixTimeNow() - 946728000.0) / 86400.0;   // сутки от J2000.0
    return wrap360(280.46061837 + 360.98564736629 * d + obsLongitude_deg);
}

// Механические углы осей. RA: часовой угол, растёт на запад (по трекингу).
// DEC: 90 — труба на полюс, больше 90 — ось перешла за полюс.
double mechHA()  { return stepperRA .currentPosition() / (double)(RA_STEPS_PER_DEG * RA_DIR); }
double mechDEC() { return stepperDEC.currentPosition() / (DEC_STEPS_PER_DEG * DEC_DIR); }

// За полюсом труба на другой стороне опоры: HA = h + 180°, DEC = 180° − d
bool pierFlipped(double d) { return d > 90.0; }

void mechToEq(double h, double d, double& ra, double& dec) {
    double ha = h;
    dec = d;
    if (pierFlipped(d)) { ha += 180.0; dec = 180.0 - d; }
    ra = wrap360(lstDeg() - ha);
}

// Механическая позиция для (RA, DEC) на нужной стороне опоры;
// h берётся ближайшим к refH (без лишних оборотов оси)
void eqToMech(double ra, double dec, bool flip, double refH, double& h, double& d) {
    double ha = lstDeg() - ra;
    d = dec;
    if (flip) { ha += 180.0; d = 180.0 - dec; }
    h = refH + wrap180(ha - refH);
}

void altAz(double ra, double dec, double& alt, double& az) {
    double ha = (lstDeg() - ra) * DEG_TO_RAD;
    double de = dec * DEG_TO_RAD, la = obsLatitude_deg * DEG_TO_RAD;
    alt = asin(sin(de)*sin(la) + cos(de)*cos(la)*cos(ha)) * RAD_TO_DEG;
    az  = wrap360(atan2(-cos(de)*sin(ha),
                        sin(de)*cos(la) - cos(de)*sin(la)*cos(ha)) * RAD_TO_DEG);
}

void updateCurrentCoords() {
    mechToEq(mechHA(), mechDEC(), currentRA_deg, currentDEC_deg);
}

void enableRA (bool on) { digitalWrite(RA_ENABLE_PIN,  on ? LOW : HIGH); }
void enableDEC(bool on) { digitalWrite(DEC_ENABLE_PIN, on ? LOW : HIGH); }

void startSlewMech(double h, double d) {
    raManualSpeed = decManualSpeed = 0;
    enableRA(true);
    enableDEC(true);
    stepperRA .setSpeed(0);
    stepperDEC.setSpeed(0);
    stepperRA .moveTo(lround(h * RA_STEPS_PER_DEG  * RA_DIR));
    stepperDEC.moveTo(lround(d * DEC_STEPS_PER_DEG * DEC_DIR));
    raSlew = decSlew = true;
    raMoving = decMoving = true;
    isSlewing = true;
    isParked  = false;
}

// GoTo по RA/DEC. Сторона опоры — та, где противовес ниже оси RA
// (|h − HOME_HA_DEG| ≤ 90°). false — время не задано или цель ниже GOTO_MIN_ALT.
bool startGoTo(double ra_deg, double dec_deg) {
    if (!timeSource) {
        Serial.println(F("GoTo refused: time not set"));
        return false;
    }
    double alt, az;
    altAz(ra_deg, dec_deg, alt, az);
    if (alt < GOTO_MIN_ALT) {
        Serial.printf("GoTo refused: RA=%.4f° DEC=%.4f° alt=%.1f°\n", ra_deg, dec_deg, alt);
        return false;
    }
    targetRA_deg  = ra_deg;
    targetDEC_deg = dec_deg;
    double h, d, hf, df;
    eqToMech(ra_deg, dec_deg, false, HOME_HA_DEG, h,  d);
    eqToMech(ra_deg, dec_deg, true,  HOME_HA_DEG, hf, df);
    if (fabs(hf - HOME_HA_DEG) < fabs(h - HOME_HA_DEG)) { h = hf; d = df; }
    startSlewMech(h, d);
    gotoPending = true;
    gotoPass    = 0;
    Serial.printf("GoTo: RA=%.4f° DEC=%.4f° → HA mech=%.4f° DEC mech=%.4f°\n",
                  ra_deg, dec_deg, h, d);
    return true;
}

// Плавная остановка GoTo (с торможением)
void haltSlew() {
    gotoPending = false;
    if (raSlew)  stepperRA .stop();
    if (decSlew) stepperDEC.stop();
}

// Мгновенная остановка: сброс скорости и цели
void abortSlew() {
    gotoPending = false;
    raSlew = decSlew = false;
    stepperRA .setCurrentPosition(stepperRA .currentPosition());
    stepperDEC.setCurrentPosition(stepperDEC.currentPosition());
}

// Sync: монтировка смотрит на (RA, DEC). Сторона опоры — ближайшая к текущей.
// false — время не задано (без LST привязка RA бессмысленна).
bool syncTo(double ra_deg, double dec_deg) {
    if (!timeSource) {
        Serial.println(F("Sync refused: time not set"));
        return false;
    }
    double h = mechHA(), d = mechDEC();
    double h1, d1, h2, d2;
    eqToMech(ra_deg, dec_deg, false, h, h1, d1);
    eqToMech(ra_deg, dec_deg, true,  h, h2, d2);
    if (fabs(h2 - h) + fabs(d2 - d) < fabs(h1 - h) + fabs(d1 - d)) { h1 = h2; d1 = d2; }
    abortSlew();
    stepperRA .setCurrentPosition(lround(h1 * RA_STEPS_PER_DEG  * RA_DIR));
    stepperDEC.setCurrentPosition(lround(d1 * DEC_STEPS_PER_DEG * DEC_DIR));
    Serial.printf("Sync → RA=%.4f° DEC=%.4f° (HA mech=%.4f° DEC mech=%.4f°)\n",
                  ra_deg, dec_deg, h1, d1);
    return true;
}

// Домашнее положение: противовес вниз, труба на полюс
void setHomePosition() {
    abortSlew();
    stepperRA .setCurrentPosition(lround(HOME_HA_DEG * RA_STEPS_PER_DEG  * RA_DIR));
    stepperDEC.setCurrentPosition(lround(90.0        * DEC_STEPS_PER_DEG * DEC_DIR));
    Serial.println(F("Home set (pole, counterweight down)"));
}

void parkMount() {
    trackingEnabled = false;
    gotoPending     = false;
    startSlewMech(HOME_HA_DEG, 90.0);
    isParked = true;
}

// ============================================================
//  HTML веб-интерфейс
// ============================================================
// Во флеше (PROGMEM): в ОЗУ страница не помещается вместе с копией при отдаче
const char htmlPage[] PROGMEM = R"rawliteral(
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
    .help{width:100%;border-collapse:collapse;font-size:13px}
    .help td{padding:5px 6px;border-top:1px solid rgba(255,255,255,0.08);vertical-align:top}
    .help td:first-child{white-space:nowrap;color:#e2b96f}
    .g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
    @media(max-width:500px){.g2,.g3{grid-template-columns:1fr}}

    /* ── Координаты ── */
    .coord-box{background:rgba(0,0,0,0.3);border-radius:7px;
               padding:8px;text-align:center}
    .coord-val{font-size:1.25em;color:#e2b96f;font-weight:700}
    .coord-lbl{font-size:.75em;color:#777;margin-top:2px}

    /* ── Ввод ── */
    label{display:block;margin-bottom:4px;font-size:.85em;color:#bbb}
    input[type=range]{width:100%;height:5px;border-radius:3px;
                      background:rgba(255,255,255,0.15);outline:none;margin-top:2px}
    input[type=number],input[type=text]{width:100%;padding:7px 10px;
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
    .dpad{display:grid;grid-template-columns:repeat(5,52px);
          grid-template-rows:repeat(5,44px);gap:6px;margin:0 auto;width:284px}
    .dpad button{padding:0;font-size:18px;border-radius:7px;margin:0;width:auto;height:auto;line-height:1.1}
    .dpad button small{display:block;font-size:10px;font-weight:600;opacity:.85}
    .dpad .fine{font-size:13px;opacity:.8}
    .cell-N {grid-column:3;grid-row:1}  .cell-n{grid-column:3;grid-row:2}
    .cell-W {grid-column:1;grid-row:3}  .cell-w{grid-column:2;grid-row:3}
    .cell-C {grid-column:3;grid-row:3}
    .cell-e {grid-column:4;grid-row:3}  .cell-E{grid-column:5;grid-row:3}
    .cell-s {grid-column:3;grid-row:4}  .cell-D{grid-column:3;grid-row:5}

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
    <div class="coords">RA <span id="raHms">—</span> (<span id="raVal">—</span>°)&nbsp;&nbsp;DEC <span id="decVal">—</span>°</div>
    <span id="statusBox">Loading…</span>
  </div>

  <!-- ═══ Трекинг + экстренная остановка (всегда видны) ═══════════════ -->
  <div class="card card-always">
    <div class="card-body">
      <div class="g2" style="align-items:center;gap:8px">
        <button id="trackBtn" class="btn-track" onclick="toggleTracking()" title="Включить/выключить ведение: ось RA вращается со звёздной скоростью, звезда стоит в поле">▶ Start Tracking</button>
        <button class="btn-estop" onclick="emergencyStop()" title="Аварийная остановка: мгновенно, без торможения; трекинг выключается, мотор RA обесточивается">⛔ STOP</button>
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
      <!-- Внешние стрелки — скорость с ползунка, внутренние — медленно (JOG_FINE) -->
      <div class="dpad">
        <button class="btn-nav cell-N" title="На север (DEC+) со скоростью с ползунка, пока кнопка нажата"
          onmousedown="jog('dec',1)" onmouseup="jogStop('dec')" onmouseleave="if(event.buttons)jogStop('dec')"
          ontouchstart="jog('dec',1)" ontouchend="jogStop('dec')">▲<small>N · DEC+</small></button>
        <button class="btn-nav fine cell-n" title="Медленно на север (0.1°/с) — для центрирования"
          onmousedown="jogFine('dec',1)" onmouseup="jogStop('dec')" onmouseleave="if(event.buttons)jogStop('dec')"
          ontouchstart="jogFine('dec',1)" ontouchend="jogStop('dec')">▴<small>slow</small></button>
        <button class="btn-nav cell-W" title="На восток (RA+) со скоростью с ползунка, пока кнопка нажата"
          onmousedown="jog('ra',-1)" onmouseup="jogStop('ra')" onmouseleave="if(event.buttons)jogStop('ra')"
          ontouchstart="jog('ra',-1)" ontouchend="jogStop('ra')">◄<small>E · RA+</small></button>
        <button class="btn-nav fine cell-w" title="Медленно на запад (0.1°/с) — для центрирования"
          onmousedown="jogFine('ra',-1)" onmouseup="jogStop('ra')" onmouseleave="if(event.buttons)jogStop('ra')"
          ontouchstart="jogFine('ra',-1)" ontouchend="jogStop('ra')">◂<small>slow</small></button>
        <button class="btn-stop cell-C" onclick="stopAll()" title="Остановить всё с торможением: GoTo, джог и трекинг">■</button>
        <button class="btn-nav fine cell-e" title="Медленно на восток (0.1°/с) — для центрирования"
          onmousedown="jogFine('ra',1)" onmouseup="jogStop('ra')" onmouseleave="if(event.buttons)jogStop('ra')"
          ontouchstart="jogFine('ra',1)" ontouchend="jogStop('ra')">▸<small>slow</small></button>
        <button class="btn-nav cell-E" title="На запад (RA−) со скоростью с ползунка, пока кнопка нажата"
          onmousedown="jog('ra',1)" onmouseup="jogStop('ra')" onmouseleave="if(event.buttons)jogStop('ra')"
          ontouchstart="jog('ra',1)" ontouchend="jogStop('ra')">►<small>W · RA−</small></button>
        <button class="btn-nav fine cell-s" title="Медленно на юг (0.1°/с) — для центрирования"
          onmousedown="jogFine('dec',-1)" onmouseup="jogStop('dec')" onmouseleave="if(event.buttons)jogStop('dec')"
          ontouchstart="jogFine('dec',-1)" ontouchend="jogStop('dec')">▾<small>slow</small></button>
        <button class="btn-nav cell-D" title="На юг (DEC−) со скоростью с ползунка, пока кнопка нажата"
          onmousedown="jog('dec',-1)" onmouseup="jogStop('dec')" onmouseleave="if(event.buttons)jogStop('dec')"
          ontouchstart="jog('dec',-1)" ontouchend="jogStop('dec')">▼<small>S · DEC−</small></button>
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
          <label>RA (° или часы — 02:31:49, 2h31m49s):</label>
          <input type="text" id="gotoRA" value="0" oninput="raHint()">
          <div id="raHint" style="font-size:11px;color:#888;margin-top:2px">= 0.0000°</div>
        </div>
        <div>
          <label>DEC (° или ° ′ ″ — +89 15 51, 89°15′51″):</label>
          <input type="text" id="gotoDEC" value="0" oninput="decHint()">
          <div id="decHint" style="font-size:11px;color:#888;margin-top:2px">= 0.0000°</div>
        </div>
      </div>
      <button class="btn-goto" onclick="doGoTo()" title="Навести трубу на RA/DEC из полей; сторона опоры выбирается так, чтобы противовес был внизу" style="margin-top:8px">🚀 Go To</button>
      <div class="g3">
        <button class="btn-muted" onclick="setHome()" title="Монтировка в доме (противовес вниз, труба на полюс): принять это положение за текущее">⌂ Set Home</button>
        <button class="btn-muted" onclick="polarSet()" title="Как Set Home, плюс подставить в поля текущие координаты полюса">✦ Polar Set</button>
        <button class="btn-muted" onclick="doSync()" title="Труба смотрит на объект с RA/DEC из полей: привязать координаты к нему (центрировать звезду, потом Sync)">⟳ Sync</button>
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
        <button class="btn-warn" onclick="doPark()" title="Выключить трекинг и увести трубу в дом (противовес вниз, труба на полюс)">Park</button>
        <button class="btn-muted" onclick="doUnpark()" title="Снять отметку «запаркована» — разрешить GoTo из планетариев">Unpark</button>
      </div>
      <div style="display:flex;align-items:center;gap:12px;margin-top:12px;flex-wrap:wrap">
        <label class="tog">
          <input type="checkbox" id="decHoldChk" onchange="setDecHold(this.checked)" title="Держать мотор DEC под током после остановки, чтобы ось не проседала" checked>
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
        <button class="btn-muted" onclick="applyMotor('ra')" title="Применить макс. скорость и ускорение RA (до перезагрузки)">Apply RA</button>
        <button class="btn-muted" onclick="applyMotor('dec')" title="Применить макс. скорость и ускорение DEC (до перезагрузки)">Apply DEC</button>
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

  <!-- ═══ Справка по кнопкам (раскрываемая) ═══════════════════════ -->
  <div class="card">
    <div class="card-hdr" onclick="tog(this)">
      <h2>❓ Справка по кнопкам</h2><span class="arr">▼</span>
    </div>
    <div class="card-body">
      <table class="help">
        <tr><td><b>▶ Start Tracking</b></td><td>Включить/выключить ведение: ось RA вращается со звёздной скоростью, звезда стоит в поле</td></tr>
        <tr><td><b>⛔ STOP</b></td><td>Аварийная остановка: мгновенно, без торможения; трекинг выключается, мотор RA обесточивается</td></tr>
        <tr><td><b>■</b></td><td>Остановить всё с торможением: GoTo, джог и трекинг</td></tr>
        <tr><td><b>▲ ▼ ◄ ►</b></td><td>Движение по N/S/E/W со скоростью с ползунка, пока кнопка нажата</td></tr>
        <tr><td><b>▴ ▾ ◂ ▸ (slow)</b></td><td>Медленное движение 0.1°/с — для центрирования звезды</td></tr>
        <tr><td><b>🚀 Go To</b></td><td>Навести трубу на RA/DEC из полей; сторона опоры выбирается так, чтобы противовес был внизу</td></tr>
        <tr><td><b>⌂ Set Home</b></td><td>Монтировка в доме (противовес вниз, труба на полюс): принять это положение за текущее</td></tr>
        <tr><td><b>✦ Polar Set</b></td><td>Как Set Home, плюс подставить в поля текущие координаты полюса</td></tr>
        <tr><td><b>⟳ Sync</b></td><td>Труба смотрит на объект с RA/DEC из полей: привязать координаты к нему (центрировать звезду, потом Sync)</td></tr>
        <tr><td><b>Park</b></td><td>Выключить трекинг и увести трубу в дом (противовес вниз, труба на полюс)</td></tr>
        <tr><td><b>Unpark</b></td><td>Снять отметку «запаркована» — разрешить GoTo из планетариев</td></tr>
        <tr><td><b>DEC Hold</b></td><td>Держать мотор DEC под током после остановки, чтобы ось не проседала</td></tr>
        <tr><td><b>Apply RA / DEC</b></td><td>Применить макс. скорость и ускорение оси (до перезагрузки)</td></tr>
        <tr><td><b>Open Update Page</b></td><td>Открыть страницу загрузки новой прошивки по WiFi</td></tr>
      </table>
      <p style="font-size:11px;color:#888;margin-top:6px">На компьютере то же пояснение всплывает при наведении на кнопку.</p>
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
      <button class="btn-muted" onclick="window.open('/update','_blank')" title="Открыть страницу загрузки новой прошивки по WiFi">Open Update Page</button>
      <p style="font-size:11px;color:#666;margin-top:6px">⚠ Do not disconnect power during update!</p>
    </div>
  </div>

</div><!-- /container -->

<script>
var trackOn=false, timeSent=false;
function fmt(v,d){return Number(v).toFixed(d||4);}
function hms(deg){var s=Math.round(((deg%360+360)%360)/15*3600)%86400;
  return [s/3600|0,(s/60|0)%60,s%60].map(x=>('0'+x).slice(-2)).join(':');}

/* ── Раскрытие секций ── */
function tog(hdr){
  hdr.classList.toggle('open');
  hdr.nextElementSibling.classList.toggle('open');
}

/* ── Поллинг статуса ── */
function poll(){
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('raVal').innerText=fmt(d.ra_deg);
    document.getElementById('raHms').innerText=hms(d.ra_deg).replace(/(\d+):(\d+):(\d+)/,'$1h$2m$3s');
    document.getElementById('decVal').innerText=fmt(d.dec_deg);
    document.getElementById('statusBox').innerHTML=
      '<b>Tracking:</b> '+(d.tracking?'<span style="color:#10b981">ON</span>':'OFF')+
      ' &nbsp;|&nbsp; <b>Slewing:</b> '+(d.slewing?'<span style="color:#f59e0b">YES</span>':'NO')+
      ' &nbsp;|&nbsp; <b>Parked:</b> '+(d.parked?'<span style="color:#60a5fa">YES</span>':'NO')+
      '<br><b>LST:</b> '+hms(d.lst)+' &nbsp;|&nbsp; <b>Alt:</b> '+fmt(d.alt,1)+'°'+
      ' &nbsp;|&nbsp; <b>Pier:</b> '+d.pier+
      (d.time_ok?'':' &nbsp;|&nbsp; <span style="color:#f59e0b">⚠ время не задано</span>');
    if(!d.time_ok&&!timeSent){timeSent=true;fetch('/set-time?ms='+Date.now());}
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
// Медленный джог для центрирования, °/с (≈24× сидерической)
var JOG_FINE=0.1;
function jogFine(axis,dir){fetch('/jog?axis='+axis+'&speed='+(dir*JOG_FINE));}
function jogStop(axis){fetch('/jog?axis='+axis+'&speed=0');}
function stopAll(){fetch('/stop-all').then(poll);}

/* ── GoTo ── */
/* RA: число — градусы; с ':' или h/m/s — часы (дробные часы: '2.53h') */
function parseRA(t){
  t=t.trim().replace(',','.');
  if(/^[+]?\d+(\.\d+)?$/.test(t))return parseFloat(t);
  var m=t.match(/^(\d+(?:\.\d+)?)\s*[h:ч]\s*(?:(\d+(?:\.\d+)?)\s*[m:м]?\s*(?:(\d+(?:\.\d+)?)\s*[sс]?)?)?$/i);
  if(!m)return NaN;
  var h=parseFloat(m[1])+(m[2]?parseFloat(m[2])/60:0)+(m[3]?parseFloat(m[3])/3600:0);
  return h<24?h*15:NaN;
}
function raDeg(){
  var v=parseRA(document.getElementById('gotoRA').value);
  if(isNaN(v)||v<0||v>=360){alert('Неверный RA');return null;}
  return v;
}
/* DEC: знак, затем до трёх чисел (градусы, минуты, секунды) через любые
   разделители — пробел, :, °, ′, ″, d m s. Одно число — дробные градусы. */
function parseDEC(t){
  t=t.trim().replace(',','.');
  var sg=/^-/.test(t)?-1:1;
  var p=t.replace(/[^0-9.]+/g,' ').trim().split(' ');
  if(!p[0]||p.length>3||p.some(x=>isNaN(parseFloat(x))))return NaN;
  var d=parseFloat(p[0]),m=p[1]?parseFloat(p[1]):0,s=p[2]?parseFloat(p[2]):0;
  if(m>=60||s>=60)return NaN;
  var v=sg*(d+m/60+s/3600);
  return Math.abs(v)<=90?v:NaN;
}
function dms(v){var s=Math.round(Math.abs(v)*3600);
  return (v<0?'−':'+')+(s/3600|0)+'°'+('0'+((s/60|0)%60)).slice(-2)+'′'+('0'+s%60).slice(-2)+'″';}
function decDeg(){
  var v=parseDEC(document.getElementById('gotoDEC').value);
  if(isNaN(v)){alert('Неверный DEC');return null;}
  return v;
}
function decHint(){
  var v=parseDEC(document.getElementById('gotoDEC').value);
  document.getElementById('decHint').innerText=isNaN(v)?'неверный формат':'= '+fmt(v)+'° = '+dms(v);
}
function raHint(){
  var v=parseRA(document.getElementById('gotoRA').value);
  document.getElementById('raHint').innerText=(isNaN(v)||v>=360)?'неверный формат':
    '= '+fmt(v)+'° = '+hms(v).replace(/(\d+):(\d+):(\d+)/,'$1h$2m$3s');
}
function doGoTo(){
  var ra=raDeg(), dec=decDeg(); if(ra===null||dec===null)return;
  fetch('/goto?ra='+ra+'&dec='+dec)
    .then(r=>r.text()).then(t=>{if(t!='OK')alert(t);poll();});
}
function setHome(){fetch('/set-home').then(poll);}
/* Труба на полюсе (противовес вниз): то же положение, что Set Home,
   плюс текущие координаты полюса в поля GoTo/Sync. Планетарии увидят
   новую позицию при следующем опросе (:GR#/:GD#, /api/get). */
function polarSet(){
  fetch('/set-home').then(()=>fetch('/status')).then(r=>r.json()).then(d=>{
    document.getElementById('gotoRA').value=hms(d.ra_deg);
    document.getElementById('gotoDEC').value='+90 00 00';
    raHint(); decHint(); poll();
    if(!d.time_ok)alert('Время не задано — RA полюса будет неверным');
  });
}
function doSync(){
  var ra=raDeg(), dec=decDeg(); if(ra===null||dec===null)return;
  fetch('/sync?ra='+ra+'&dec='+dec)
    .then(r=>r.text()).then(t=>{if(t!='OK')alert(t);poll();});
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
    setHomePosition();   // после включения монтировка считается в доме

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
        settimeofday_cb([](bool fromSntp) { if (fromSntp) timeSource = 2; });
        configTime(0, 0, "pool.ntp.org", "time.google.com");
    } else {
        Serial.println(F("\nWiFi failed — AP mode: EQ2-Mount / telescope"));
        WiFi.softAP("EQ2-Mount", "telescope");
        Serial.print(F("AP IP: ")); Serial.println(WiFi.softAPIP());
    }

    // Web UI маршруты
    server.on("/",         HTTP_GET, []{ server.send_P(200, "text/html", htmlPage); });
    server.on("/status",   HTTP_GET, handleStatus);
    server.on("/tracking", HTTP_GET, handleTracking);
    server.on("/ra-speed", HTTP_GET, handleRASpeed);
    server.on("/jog",      HTTP_GET, handleJog);
    server.on("/stop-all", HTTP_GET, handleStopAll);
    server.on("/goto",     HTTP_GET, handleGoTo);
    server.on("/sync",     HTTP_GET, handleSync);
    server.on("/set-home", HTTP_GET, handleSetHome);
    server.on("/set-time", HTTP_GET, handleSetTime);
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

    // — RA: GoTo (run() с разгоном) или постоянная скорость (трекинг + джог) —
    if (raSlew) {
        if (!stepperRA.run()) raSlew = false;
        raStopTime = now;
    } else {
        float sps = raManualSpeed * RA_STEPS_PER_DEG * RA_DIR;
        if (trackingEnabled) sps += RA_SIDEREAL_SPEED * raSpeedMult * RA_DIR;
        if (sps != 0.0f) {
            enableRA(true);
            stepperRA.setSpeed(sps);
            stepperRA.runSpeed();
            raMoving = true; raStopTime = now;
        } else if (raMoving && now - raStopTime > MOTOR_OFF_DELAY) {
            enableRA(false); raMoving = false;
        }
    }

    // — DEC: GoTo или джог. Джог по небу (север +), за полюсом механика наоборот.
    //   Мотор DEC после остановки не выключаем, если включено удержание —
    if (decSlew) {
        if (!stepperDEC.run()) decSlew = false;
        decStopTime = now;
    } else if (decManualSpeed != 0.0f) {
        float sky = pierFlipped(mechDEC()) ? -1.0f : 1.0f;
        enableDEC(true);
        stepperDEC.setSpeed(decManualSpeed * sky * DEC_STEPS_PER_DEG * DEC_DIR);
        stepperDEC.runSpeed();
        decMoving = true; decStopTime = now;
    } else if (decMoving && now - decStopTime > MOTOR_OFF_DELAY) {
        decMoving = false;
        if (!decHoldEnabled) enableDEC(false);
    }

    // — Доводка GoTo: пока оси шли, небо сместилось —
    if (gotoPending && !raSlew && !decSlew) {
        double h = mechHA(), d = mechDEC(), th, td;
        eqToMech(targetRA_deg, targetDEC_deg, pierFlipped(d), h, th, td);
        if (++gotoPass < 4 && fabs(th - h) * RA_STEPS_PER_DEG > 32) {
            startSlewMech(th, td);
        } else {
            gotoPending = false;
        }
    }

    isSlewing = raSlew || decSlew || gotoPending;

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
    double alt, az;
    altAz(currentRA_deg, currentDEC_deg, alt, az);
    bool westSide = !pierFlipped(mechDEC()) == (HOME_HA_DEG < 0);
    j += "\"lst\":"      + String(lstDeg(), 4)      + ",";
    j += "\"alt\":"      + String(alt, 2)           + ",";
    j += "\"az\":"       + String(az, 2)            + ",";
    j += "\"pier\":\""  + String(westSide ? "W" : "E") + "\",";
    j += "\"time_ok\":"  + String(timeSource ? "true" : "false") + ",";
    j += "\"time_src\":" + String(timeSource)       + ",";
    j += "\"ra_drv\":"   + String(raDrvOk  ? "true" : "false") + ",";
    j += "\"dec_drv\":"  + String(decDrvOk ? "true" : "false") + ",";
    j += "\"ip\":\""     + WiFi.localIP().toString() + "\"";
    j += "}";
    server.send(200, "application/json", j);
}

void handleTracking() {
    bool en = server.hasArg("enable") && server.arg("enable") == "1";
    trackingEnabled = en;
    if (en) enableRA(true);   // выключит loop(), когда ось постоит
    else    raManualSpeed = 0;
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
    // RA: + на запад (механически), DEC: + на север (по небу)
    if (spd != 0) haltSlew();
    if (axis == "ra") {
        raManualSpeed = spd;  raStopTime  = millis();
    } else if (axis == "dec") {
        decManualSpeed = spd; decStopTime = millis();
    }
    server.send(200, "text/plain", "OK");
}

void handleStopAll() {
    trackingEnabled = false;
    raManualSpeed   = 0;
    decManualSpeed  = 0;
    haltSlew();             // моторы выключит loop() после торможения
    raStopTime = decStopTime = millis();
    server.send(200, "text/plain", "OK");
}

void handleGoTo() {
    if (server.hasArg("ra") && server.hasArg("dec")) {
        if (!startGoTo(server.arg("ra").toDouble(), server.arg("dec").toDouble())) {
            server.send(400, "text/plain",
                        timeSource ? "Цель ниже горизонта" : "Время не задано");
            return;
        }
    }
    server.send(200, "text/plain", "OK");
}

void handleSync() {
    if (server.hasArg("ra") && server.hasArg("dec")) {
        if (!syncTo(server.arg("ra").toDouble(), server.arg("dec").toDouble())) {
            server.send(400, "text/plain", "Время не задано");
            return;
        }
    }
    server.send(200, "text/plain", "OK");
}

void handleSetHome() {
    setHomePosition();
    server.send(200, "text/plain", "OK");
}

// GET /set-time?ms=<unix ms> — время из браузера, если нет NTP
void handleSetTime() {
    if (server.hasArg("ms") && timeSource < 2)
        setClock(server.arg("ms").toDouble() / 1000.0, 1);
    server.send(200, "application/json",
                String("{\"time_src\":") + timeSource + "}");
}

void handlePark() {
    parkMount();
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
    abortSlew();
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
        server.send(200, "application/json",
                    startGoTo(ra, dec) ? "{\"ret\":0}" : "{\"ret\":1}");
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

// Дней от 1970-01-01 до даты (григорианский календарь)
long daysFromCivil(int y, int m, int d) {
    y -= m <= 2;
    long era = (y >= 0 ? y : y - 399) / 400;
    long yoe = y - era * 400;
    long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

// Местное время по смещению из :SG
void lxLocalTime(tm& t) {
    time_t lt = (time_t)(unixTimeNow() - lxUtcOffsetH * 3600.0);
    gmtime_r(&lt, &t);
}

// "HH:MM:SS#" (или с дробной частью для HP-варианта)
String lxFormatHMS(long secs, const char* frac) {
    secs = ((secs % 86400) + 86400) % 86400;
    char buf[24];
    sprintf(buf, "%02ld:%02ld:%02ld%s#", secs / 3600, (secs / 60) % 60, secs % 60, frac);
    return String(buf);
}

String lxLocalTimeStr(const char* frac) {
    tm t; lxLocalTime(t);
    return lxFormatHMS(t.tm_hour * 3600L + t.tm_min * 60 + t.tm_sec, frac);
}

String lxLocalDateStr() {
    tm t; lxLocalTime(t);
    char buf[12];
    sprintf(buf, "%02d/%02d/%02d#", t.tm_mon + 1, t.tm_mday, t.tm_year % 100);
    return String(buf);
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
    } else if (cmd == "GA" || cmd == "GAH" || cmd == "GZ" || cmd == "GZH") {
        double alt, az;
        altAz(currentRA_deg, currentDEC_deg, alt, az);
        lxSend(cmd.charAt(1) == 'A' ? lxFormatDEC_HP(alt) : lxFormatAz(az));

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
    } else if (cmd == "Gal" || cmd == "GaH" || cmd == "Gz" || cmd == "GzH") {
        double alt, az;
        altAz(targetRA_deg, targetDEC_deg, alt, az);
        lxSend(cmd.charAt(1) == 'a' ? lxFormatDEC_HP(alt) : lxFormatAz(az));

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
        else { lxSend(startGoTo(targetRA_deg, targetDEC_deg) ? "0" : "1"); }

    } else if (cmd == "MA") {
        if (isParked) { lxSend("4"); }
        else { lxSend(startGoTo(targetRA_deg, targetDEC_deg) ? "0" : "1"); }

    } else if (cmd == "MD") {
        // 0=East, 1=West, 2=Unknown
        lxSend("2");

    } else if (cmd == "MN" || cmd == "MNe" || cmd == "MNw") {
        if (isParked) { lxSend("4"); }
        else { lxSend(startGoTo(targetRA_deg, targetDEC_deg) ? "0" : "1"); }

    } else if (cmd == "MP") {
        lxSend(isParked ? "4" : "0");

    // ── Sync (:CM# :CS#) ─────────────────────────────────────────────────
    // lx200driver: Sync(:CM#) → "N/A#" on success, "En#" on failure
    } else if (cmd == "CM" || cmd == "CS") {
        lxSend(syncTo(targetRA_deg, targetDEC_deg) ? "N/A#" : "E0#");

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
        if (doRA)  raManualSpeed  = 0;
        if (doDEC) decManualSpeed = 0;
        if (cmd == "Q") haltSlew();
        // нет ответа

    // ── Tracking (:Te# :Td# :TQ# :TS# :TK# :TL# :T+# :T-# :TR#
    //             :To# :Tr# :Tn# :T1# :T2#) ──────────────────────────────
    // lx200driver: selectTrackingMode
    } else if (cmd == "Te") {
        trackingEnabled = true; enableRA(true); lxSend("1");
    } else if (cmd == "Td") {
        trackingEnabled = false; raManualSpeed=0; lxSend("1");
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
        if (!isParked) { parkMount(); lxSend("1"); }
        else           { lxSend("0"); }

    } else if (cmd == "hR") {
        if (isParked) { isParked=false; enableDEC(true); lxSend("1"); }
        else          { lxSend("0"); }

    } else if (cmd == "hF") {
        setHomePosition();
        isParked = false;
        // нет ответа

    } else if (cmd == "hC") {
        startSlewMech(HOME_HA_DEG, 90.0);
        // нет ответа

    } else if (cmd == "hQ") {
        lxSend("1");  // "set park pos" — запоминаем текущую (в ОЗУ)

    } else if (cmd.startsWith("hA")) {
        // :hAn# — auto home state (нет ответа)

    // ── Manual move (:Me# :Mw# :Mn# :Ms# :Mp#) ──────────────────────────
    // lx200driver: MoveTo(:Me#/:Mw#/:Mn#/:Ms#)  (нет ответа)
    } else if (cmd == "Me") {
        enableRA(true);  raManualSpeed  = -lxGuideRate;   // восток: HA уменьшается
        raMoving=true;   raStopTime=millis();
    } else if (cmd == "Mw") {
        enableRA(true);  raManualSpeed  =  lxGuideRate;
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
        // сторона опоры, где стоит труба: у HOME_HA_DEG < 0 обычная сторона — западная
        else lxSend(!pierFlipped(mechDEC()) == (HOME_HA_DEG < 0) ? "W#" : "E#");

    // ── Site (:Gt# :GtH# :Gg# :GgH# :Gv#) ──────────────────────────────
    // lx200driver: getSiteLatitude(:Gt#), getSiteLongitude(:Gg#)
    } else if (cmd == "Gt" || cmd == "GtH") {
        lxSend(lxFormatLatitude(obsLatitude_deg));
    } else if (cmd == "Gg" || cmd == "GgH") {
        lxSend(lxFormatLongitude(obsLongitude_deg));
    } else if (cmd == "Gv") {
        char buf[14]; sprintf(buf, "+%.1f#", obsAltitude_m);
        lxSend(String(buf));

    // lx200driver: setSiteLatitude(:StsDD*MM#), setSiteLongitude(:SgDDD*MM#, West +)
    } else if (cmd.startsWith("St")) {
        obsLatitude_deg = lxParseDEC(cmd.substring(2));
        lxSend("1");
        Serial.printf("  [LX] Latitude=%.4f°\n", obsLatitude_deg);
    } else if (cmd.startsWith("Sg")) {
        obsLongitude_deg = wrap180(-lxParseDEC(cmd.substring(2)));
        lxSend("1");
        Serial.printf("  [LX] Longitude=%.4f° E\n", obsLongitude_deg);

    // ── Time / Date (:GL# :GLH# :GG# :GS# :GSH# :GC# :Ga# :Gc#) ────────
    // lx200driver: getLocalTime24(:GL#), getLocalTime12(:Ga#),
    //              getSDTime(:GS#), getCalendarDate(:GC#)
    } else if (cmd == "GL" || cmd == "GLH") {
        lxSend(lxLocalTimeStr(cmd == "GLH" ? ".0000" : ""));

    } else if (cmd == "GG") {
        // lx200driver: getUTCOffset(:GG#) — часы, которые прибавить к местному до UTC
        int m = (int)lround(fabs(lxUtcOffsetH) * 60.0);
        char buf[12];
        sprintf(buf, "%c%02d:%02d#", lxUtcOffsetH < 0 ? '-' : '+', m / 60, m % 60);
        lxSend(String(buf));

    } else if (cmd == "GS" || cmd == "GSH") {
        // lx200driver: getSDTime(:GS#) — местное звёздное время
        lxSend(lxFormatHMS(lround(lstDeg() / 15.0 * 3600.0) , cmd == "GSH" ? ".00" : ""));

    } else if (cmd == "GC") {
        // lx200driver: getCalendarDate(:GC#) → MM/DD/YY
        lxSend(lxLocalDateStr());

    } else if (cmd == "Ga") {
        // lx200driver: getLocalTime12(:Ga#)
        tm t; lxLocalTime(t);
        int hh = t.tm_hour % 12; if (!hh) hh = 12;
        lxSend(lxFormatHMS(hh * 3600L + t.tm_min * 60 + t.tm_sec, ""));

    } else if (cmd == "Gc") {
        lxSend("24#");

    // ── Set time/date/UTC (:SC# :SG# :SL#) ──────────────────────────────
    // lx200driver: setCalenderDate, setUTCOffset, setLocalTime → 0/1
    // Часы ставятся по :SC (после :SG и :SL), только если нет NTP
    } else if (cmd.startsWith("SG")) {
        lxUtcOffsetH = cmd.substring(2).toFloat();
        lxSend("1");
    } else if (cmd.startsWith("SL")) {
        String a = cmd.substring(2);
        lxLocalSecs = a.substring(0, 2).toInt() * 3600L
                    + a.substring(3, 5).toInt() * 60 + a.substring(6, 8).toInt();
        lxSend("1");
    } else if (cmd.startsWith("SC")) {
        String a = cmd.substring(2);   // MM/DD/YY
        if (lxLocalSecs >= 0 && timeSource < 2) {
            long days = daysFromCivil(2000 + a.substring(6, 8).toInt(),
                                      a.substring(0, 2).toInt(), a.substring(3, 5).toInt());
            setClock(days * 86400.0 + lxLocalSecs + lxUtcOffsetH * 3600.0, 1);
        }
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
    } else if (cmd == "Gh") {
        char buf[8]; sprintf(buf, "%+03d*#", (int)GOTO_MIN_ALT);
        lxSend(String(buf));
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
    } else if (cmd == "GX80") { lxSend(lxLocalTimeStr(".00"));
    } else if (cmd == "GX81") { lxSend(lxLocalDateStr());
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
