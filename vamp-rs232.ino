/*
 * Inverter Monitor & Heater Controller — PI30 (QPIGS) for ECO II 4.2kW Plus
 * v8.0.3 — на дашборде инвертора показываем только ВА (полная мощность)
 *
 * QPIGS-раскладка (0-based индексы):
 *   0  gridV    1  gridHz   2  acOutV   3  acOutHz
 *   4  acOutVA  5  acOutW   6  loadPct  7  busV
 *   8  battV    9  battChgA 10 battCap  11 tempC
 *  12  pvCurrentA  13 pvVoltage  14 sccBattV
 *  15  battDischRaw (×0.01 → A)  16 deviceStatus
 *  17  fanOffset   18 eepromVer  19 pvPowerW  20 status2
 *
 * Апаратне забезпечення:
 *   GPIO16 (RX2) <- MAX3232 TXD
 *   GPIO17 (TX2) -> MAX3232 RXD
 *   GPIO2  -> LED    GPIO19 -> TRIAC
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#define RX2_PIN        16
#define TX2_PIN        17
#define LED_PIN        2
#define TRIAC_PIN      19
#define SKETCH_VERSION "8.0.3-pi30"
#define SERIAL_BUF_SIZE 4096

constexpr uint32_t TRIAC_WINDOW_MS = 1000;
constexpr uint32_t TRIAC_TICK_MS   = 10;
constexpr uint32_t INVERTER_BAUD   = 2400;

volatile uint8_t currentTriacDuty     = 0;
volatile uint8_t currentHeaterPercent = 0;

TaskHandle_t triacTaskHandle    = nullptr;
TaskHandle_t inverterTaskHandle = nullptr;

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS     = 12000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS  = 10000;
constexpr uint32_t RESTART_DELAY_MS            = 2000;
constexpr uint32_t WEB_UPDATE_TIMEOUT_MS       = 30000;
constexpr float    DEFAULT_UPDATE_INTERVAL_SEC = 2.0f;

SemaphoreHandle_t dataMutex     = nullptr;
SemaphoreHandle_t inverterMutex = nullptr;
volatile bool webUpdateActive                  = false;
volatile unsigned long lastWebUpdateActivityMs = 0;
volatile bool dumpEnabled                      = false;
volatile bool lastPollOk                       = false;
volatile uint16_t lastRespLen                  = 0;

uint16_t cfgHeaterPowerVA     = 100;
uint16_t cfgMaxAllocatedVA    = 100;
float    cfgUpdateIntervalSec = DEFAULT_UPDATE_INTERVAL_SEC;
float    cfgBattOffVoltage    = 27.00f;
float    cfgBattHysteresisV   = 0.50f;
float    cfgFullChargeCurrentA  = 0.00f;
float    cfgFullChargeVoltage   = 29.00f;
float    cfgBattDischargeAllowW = 0.00f;

float battV = 0.00f, pvV = 0.00f, pvI = 0.00f;
float invV  = 0.00f, invI = 0.00f, gridFreq = 0.00f, systemV = 0.00f;
int32_t freePower = 0, currentPower = 0, pvPower = 0, invPower = 0, PpowerHeater = 0;
int32_t acOutVA = 0;
float battI = 0.00f, battP = 0.00f;
float battCapPct = 0.0f, battTempC = 0.0f;
int32_t loadPercent = 0;

volatile bool switch_state = false;
static char targMessageGlobal[128] = "--";

// ================== PI30 СТАН ==================
struct InverterData {
  int    workMode     = 0;
  String machineType  = "Invertor-rs232";
  String machinePower = "MAXIMAL";

  float   gridV = 0, gridHz = 0;
  float   acOutV = 0, acOutHz = 0;
  int32_t acOutVA = 0, acOutW = 0;
  int32_t loadPercent = 0, busV = 0;
  float   battVq = 0, battCap = 0, tempC = 0;
  int32_t battChgA = 0;
  float   pvCurrentA = 0, pvVoltage = 0, sccBattV = 0;
  int32_t battDischRaw = 0;
  uint32_t deviceStatus = 0;
  int32_t fanOffset = 0, eepromVer = 0;
  int32_t pvPowerW = 0, status2 = 0;

  float f_raw[24] = {0};
  int   f_count   = 0;
} inv;

static char serialBuf[SERIAL_BUF_SIZE];
static volatile uint16_t serialHead = 0, serialTail = 0;
static portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

HardwareSerial Inv(2);

void serialLog(const char* msg) {
  size_t len = strlen(msg);
  portENTER_CRITICAL(&serialMux);
  for (size_t i = 0; i < len; i++) {
    serialBuf[serialHead] = msg[i];
    serialHead = (serialHead + 1) % SERIAL_BUF_SIZE;
    if (serialHead == serialTail) serialTail = (serialTail + 1) % SERIAL_BUF_SIZE;
  }
  portEXIT_CRITICAL(&serialMux);
}

// ================== PI30 PROTOCOL ==================
static uint16_t pi30_crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

static void pi30_buildCmd(const char* text, uint8_t* buf, size_t& len) {
  len = 0;
  size_t n = strlen(text);
  for (size_t i = 0; i < n; i++) buf[len++] = (uint8_t)text[i];
  uint16_t crc = pi30_crc16((const uint8_t*)text, n);
  buf[len++] = (uint8_t)(crc >> 8);
  buf[len++] = (uint8_t)(crc & 0xFF);
  buf[len++] = 0x0D;
}

static uint32_t pi30_callCount = 0;

static String pi30_query(const char* cmd, char startChar = '(', char endChar = '\r',
                         uint32_t timeoutMs = 2500) {
  uint8_t frame[32];
  size_t frameLen;
  pi30_buildCmd(cmd, frame, frameLen);

  while (Inv.available()) Inv.read();
  Inv.write(frame, frameLen);
  Inv.flush();

  bool doLog = (pi30_callCount < 3) || dumpEnabled;
  if (doLog) {
    char buf[96];
    int p = snprintf(buf, sizeof(buf), ">>> TX[%d]:", (int)frameLen);
    for (size_t i = 0; i < frameLen && p < (int)sizeof(buf) - 4; i++)
      p += snprintf(buf + p, sizeof(buf) - p, " %02X", frame[i]);
    p += snprintf(buf + p, sizeof(buf) - p, "\n");
    serialLog(buf);
  }

  String resp = "";
  unsigned long t0 = millis();
  bool started = false;
  uint32_t lastByteMs = 0;

  while (millis() - t0 < timeoutMs) {
    if (Inv.available()) {
      char c = Inv.read();
      lastByteMs = millis();
      if (!started) {
        if (c == startChar) { resp += c; started = true; }
      } else {
        resp += c;
        if (c == endChar) break;
      }
    } else if (started && lastByteMs && (millis() - lastByteMs > 600)) {
      break;
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  if (doLog) {
    char buf[320];
    int p = snprintf(buf, sizeof(buf), ">>> RX[%d]:", resp.length());
    for (size_t i = 0; i < resp.length() && p < (int)sizeof(buf) - 6; i++) {
      uint8_t b = (uint8_t)resp[i];
      p += snprintf(buf + p, sizeof(buf) - p, " %02X", b);
    }
    if (resp.length() > 0 && p < (int)sizeof(buf) - 4) {
      p += snprintf(buf + p, sizeof(buf) - p, "  [");
      for (size_t i = 0; i < resp.length() && p < (int)sizeof(buf) - 4; i++) {
        char c = resp[i];
        buf[p++] = (c >= 32 && c < 127) ? c : '.';
      }
      p += snprintf(buf + p, sizeof(buf) - p, "]\n");
    } else {
      p += snprintf(buf + p, sizeof(buf) - p, "  <EMPTY — no response!>\n");
    }
    serialLog(buf);
    pi30_callCount++;
  }

  return resp;
}

// РОЗБІР QPIGS (ECO II, 21 полів)
static bool pi30_parseQPIGS(const String& resp, InverterData& out) {
  if (!resp.startsWith("(") || !resp.endsWith("\r")) return false;
  String body = resp.substring(1, resp.length() - 1);
  body.trim();

  float f[24];
  int count = 0, start = 0, len = body.length();
  while (start < len && count < 24) {
    while (start < len && body[start] == ' ') start++;
    if (start >= len) break;
    int end = start;
    while (end < len && body[end] != ' ') end++;
    String fld = body.substring(start, end);
    fld.trim();
    f[count++] = fld.toFloat();
    start = end + 1;
  }
  if (count < 15) return false;

  out.f_count = count;
  for (int i = 0; i < count; i++) out.f_raw[i] = f[i];

  out.gridV       = f[0];
  out.gridHz      = f[1];
  out.acOutV      = f[2];
  out.acOutHz     = f[3];
  out.acOutVA     = (int32_t)f[4];
  out.acOutW      = (int32_t)f[5];
  out.loadPercent = (int32_t)f[6];
  out.busV        = (int32_t)f[7];
  out.battVq      = f[8];
  out.battChgA    = (int32_t)f[9];
  out.battCap     = f[10];
  out.tempC       = f[11];
  out.pvCurrentA  = f[12];
  out.pvVoltage   = f[13];
  out.sccBattV    = f[14];
  out.battDischRaw = (int32_t)f[15];
  if (count > 16) out.deviceStatus = (uint32_t)f[16];
  if (count > 17) out.fanOffset    = (int32_t)f[17];
  if (count > 18) out.eepromVer    = (int32_t)f[18];
  if (count > 19) out.pvPowerW     = (int32_t)f[19];
  if (count > 20) out.status2      = (int32_t)f[20];
  return true;
}

static int pi30_parseQMOD(const String& resp) {
  if (!resp.startsWith("(") || resp.length() < 2) return -1;
  switch (resp[1]) {
    case 'P': return 0;  case 'S': return 1;  case 'L': return 2;
    case 'B': return 3;  case 'F': return 4;  case 'H': return 5;
    case 'D': return 6;  default:  return -1;
  }
}

// ================== ДОПОМІЖНІ ==================

bool isLocalIP(IPAddress ip) {
  if (ip[0] == 10) return true;
  if (ip[0] == 172 && ip[1] >= 16 && ip[1] <= 31) return true;
  if (ip[0] == 192 && ip[1] == 168) return true;
  IPAddress localIP = WiFi.localIP();
  IPAddress softIP  = WiFi.softAPIP();
  if (localIP[0] != 0 && ip == localIP) return true;
  if (softIP[0]  != 0 && ip == softIP)  return true;
  return false;
}

void requireLocalIP(AsyncWebServerRequest *request,
                    std::function<void(AsyncWebServerRequest*)> handler) {
  if (!request->client() || !isLocalIP(request->client()->remoteIP())) {
    request->send(403, "text/plain", "Access denied: Local network only");
    return;
  }
  handler(request);
}

AsyncWebServer server(80);
DNSServer dnsServer;
Preferences preferences;
String wifiSsid, wifiPassword;
bool setupMode = false;
unsigned long lastWifiReconnectMs = 0;
bool needRestart = false;
unsigned long restartRequestedMs = 0;

// ================== SETTINGS ==================
void loadSettings() {
  preferences.begin("settings", true);
  cfgHeaterPowerVA       = preferences.getUShort("heaterVA", 100);
  cfgMaxAllocatedVA      = preferences.getUShort("maxAllocVA", 100);
  cfgUpdateIntervalSec   = constrain(preferences.getFloat("updIntSec", DEFAULT_UPDATE_INTERVAL_SEC), 0.5f, 10.0f);
  cfgBattOffVoltage      = constrain(preferences.getFloat("battOffV", 27.0f), 20.0f, 80.0f);
  cfgBattHysteresisV     = constrain(preferences.getFloat("battHystV", 0.5f), 0.0f, 10.0f);
  cfgFullChargeCurrentA  = constrain(preferences.getFloat("chgCurA", 0.0f), -100.0f, 100.0f);
  cfgFullChargeVoltage   = constrain(preferences.getFloat("fullChgV", 29.0f), 20.0f, 80.0f);
  cfgBattDischargeAllowW = constrain(preferences.getFloat("battDisW", 0.0f), 0.0f, 15000.0f);
  switch_state           = preferences.getBool("swState", false);
  preferences.end();
}

void saveSettings() {
  preferences.begin("settings", false);
  preferences.putUShort("heaterVA",   cfgHeaterPowerVA);
  preferences.putUShort("maxAllocVA", cfgMaxAllocatedVA);
  preferences.putFloat ("updIntSec",  cfgUpdateIntervalSec);
  preferences.putFloat ("battOffV",   cfgBattOffVoltage);
  preferences.putFloat ("battHystV",  cfgBattHysteresisV);
  preferences.putFloat ("chgCurA",    cfgFullChargeCurrentA);
  preferences.putFloat ("fullChgV",   cfgFullChargeVoltage);
  preferences.putFloat ("battDisW",   cfgBattDischargeAllowW);
  preferences.putBool  ("swState",    switch_state);
  preferences.end();
}

// ================== СІМІСТОР ==================
void triacTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  uint32_t windowStart = millis();

  for (;;) {
    uint8_t targetPercent = switch_state ? currentHeaterPercent : 0;

    if (targetPercent == 0) {
      digitalWrite(TRIAC_PIN, LOW);
      currentTriacDuty = 0;
      windowStart = millis();
    } else if (targetPercent >= 100) {
      digitalWrite(TRIAC_PIN, HIGH);
      currentTriacDuty = 100;
      windowStart = millis();
    } else {
      uint32_t now = millis();
      uint32_t elapsed = now - windowStart;
      if (elapsed >= TRIAC_WINDOW_MS) {
        windowStart += TRIAC_WINDOW_MS;
        if ((int32_t)(now - windowStart) >= (int32_t)TRIAC_WINDOW_MS) windowStart = now;
        elapsed = now - windowStart;
      }
      uint32_t onTimeMs = (TRIAC_WINDOW_MS * (uint32_t)targetPercent) / 100;
      if (elapsed < onTimeMs) {
        digitalWrite(TRIAC_PIN, HIGH);
        currentTriacDuty = targetPercent;
      } else {
        digitalWrite(TRIAC_PIN, LOW);
        currentTriacDuty = 0;
      }
    }
    vTaskDelayUntil(&lastWakeTime, TRIAC_TICK_MS);
  }
}

// ================== ОПИТУВАННЯ ==================
void pollInverterOnce() {
  digitalWrite(LED_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(20));

  digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);
  String resp = pi30_query("QPIGS");
  digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);

  bool ok = (resp.length() >= 60) && pi30_parseQPIGS(resp, inv);

  if (!ok) {
    lastPollOk  = false;
    lastRespLen = resp.length();
    char buf[128];
    snprintf(buf, sizeof(buf), "!!! QPIGS parse failed, respLen=%d\n", resp.length());
    serialLog(buf);
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);
  String respMod = pi30_query("QMOD", '(', '\r', 1200);
  int mode = pi30_parseQMOD(respMod);
  if (mode >= 0) inv.workMode = mode;
  digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);

  lastPollOk  = true;
  lastRespLen = resp.length();

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // ---- PV ----
  pvV = inv.pvVoltage;
  pvI = inv.pvCurrentA;
  pvPower = inv.pvPowerW;
  if (pvPower <= 0) pvPower = (int32_t)(pvV * pvI);
  if (pvV < 0)  pvV = 0;
  if (pvI < 0)  pvI = 0;
  if (pvPower < 0) pvPower = 0;

  // ---- Інвертор ----
  invV        = inv.acOutV;
  invPower    = inv.acOutW;             // активна (для балансу ТЭНа)
  acOutVA     = inv.acOutVA;            // полная (для отображения)
  invI        = (invV > 0.5f) ? (float)inv.acOutVA / invV : 0.0f;
  loadPercent = inv.loadPercent;
  gridFreq    = inv.gridHz;
  systemV     = (float)inv.busV;

  // ---- АКБ ----
  battV = inv.battVq;
  float chgA   = (float)inv.battChgA;
  float dischA = (float)inv.battDischRaw;
  battI = chgA - dischA;
  battP = battV * battI;
  battCapPct = inv.battCap;
  battTempC  = inv.tempC;

  // ---- Логіка керування ТЕНом ----
  bool isBatteryMode = (inv.workMode == 3);

  if (!switch_state) {
    freePower = 0; currentPower = 0;
  } else {
    freePower = pvPower - invPower;
    if (cfgFullChargeCurrentA > 0.0f && battV < cfgFullChargeVoltage) {
      freePower -= (int32_t)(battV * cfgFullChargeCurrentA);
    }
    freePower += (int32_t)cfgBattDischargeAllowW;
  }

  static bool isBattLow = false;
  if (cfgBattOffVoltage > 0.0f) {
    if (battV < cfgBattOffVoltage) isBattLow = true;
    else if (battV >= cfgBattOffVoltage + cfgBattHysteresisV) isBattLow = false;
  } else isBattLow = false;

  const char* targMessage = "Все добре";

  if (!switch_state) {
    freePower = 0; currentPower = 0;
    targMessage = "Споживання вимкнено";
  } else if (!isBatteryMode) {
    freePower = 0; currentPower = 0;
    targMessage = "Інвертер споживає з розетки";
  } else if (isBattLow) {
    freePower = 0; currentPower = 0;
    targMessage = "Напруга акумулятора менше встановленого мінімуму";
  } else if (freePower < 0) {
    currentPower += freePower;
    if (currentPower < 0) currentPower = 0;
    freePower = 0;
    targMessage = "Недостатньо потужності";
  } else if (battI < cfgFullChargeCurrentA && battV < cfgFullChargeVoltage) {
    int32_t deficitW = (int32_t)((cfgFullChargeCurrentA - battI) * battV);
    if (deficitW < cfgMaxAllocatedVA / 20) deficitW = cfgMaxAllocatedVA / 20;
    currentPower -= deficitW;
    if (currentPower < 0) currentPower = 0;
    freePower = 0;
    targMessage = "Струм акумулятора менше мінімуму — зменшую потужність на 5%";
  } else if (freePower == 0) {
    targMessage = "Потужність на максимумі";
  } else {
    if (currentPower >= cfgMaxAllocatedVA) {
      currentPower = cfgMaxAllocatedVA;
      targMessage = "Потужність обрізано до максимуму";
    } else if (freePower > cfgMaxAllocatedVA / 20) {
      currentPower += cfgMaxAllocatedVA / 20;
      if (currentPower > cfgMaxAllocatedVA) currentPower = cfgMaxAllocatedVA;
      targMessage = "Потужність збільшена на 5%";
    } else {
      currentPower += freePower;
      if (currentPower > cfgMaxAllocatedVA) currentPower = cfgMaxAllocatedVA;
      targMessage = "Потужність збільшено";
    }
  }

  PpowerHeater = currentPower;
  if (cfgHeaterPowerVA > 0)
    currentHeaterPercent = constrain((PpowerHeater * 100) / cfgHeaterPowerVA, 0, 100);
  else currentHeaterPercent = 0;

  strncpy(targMessageGlobal, targMessage, sizeof(targMessageGlobal) - 1);
  targMessageGlobal[sizeof(targMessageGlobal) - 1] = '\0';

  char logLine[512];
  snprintf(logLine, sizeof(logLine),
    "[%lu] mode=%d gridV=%.1f gridHz=%.1f outV=%.1f outVA=%d outW=%d load=%d%% "
    "battV=%.2f battI=%.2f battCap=%.0f%% temp=%.0f "
    "pvV=%.1f pvI=%.1f pvW=%d free=%d cur=%d hP=%d%% msg=%s\n",
    millis(), inv.workMode, inv.gridV, inv.gridHz, inv.acOutV, (int)inv.acOutVA, (int)inv.acOutW,
    (int)inv.loadPercent, battV, battI, battCapPct, battTempC,
    pvV, pvI, (int)pvPower, (int)freePower, (int)currentPower,
    (int)currentHeaterPercent, targMessage);
  serialLog(logLine);

  xSemaphoreGive(dataMutex);
  digitalWrite(LED_PIN, HIGH);
}

// ================== ЗАДАЧА ==================
void inverterTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(2000));
  for (;;) {
    if (!webUpdateActive) {
      if (xSemaphoreTake(inverterMutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
        pollInverterOnce();
        xSemaphoreGive(inverterMutex);
      }
    }
    uint32_t intervalMs = (uint32_t)(cfgUpdateIntervalSec * 1000.0f);
    if (intervalMs < 500) intervalMs = 500;
    vTaskDelay(pdMS_TO_TICKS(intervalMs));
  }
}

// ================== HTML: INDEX ==================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Inverter</title>
<style>
body{background:#121212;color:#e6e6e6;font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;padding:12px;margin:0;display:flex;flex-direction:column;align-items:center}
.dashboard{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;width:100%;max-width:960px;align-items:stretch}
.col{display:flex;flex-direction:column;gap:12px}
.panel{background:#1e1e1e;border-radius:10px;padding:14px;box-shadow:0 4px 6px rgba(0,0,0,.3);flex:1;display:flex;flex-direction:column}
.panel-header{display:flex;align-items:center;gap:8px;color:#888;font-size:11px;font-weight:bold;letter-spacing:1px;margin-bottom:12px;text-transform:uppercase}
.panel-header svg{width:14px;height:14px;fill:#ffd166}
.panel-header.blue svg{fill:#4da3ff}
.grid-2x2{display:grid;grid-template-columns:1fr 1fr;gap:8px;flex:1;grid-auto-rows:1fr}
.metric{text-align:center;background:#252525;border-radius:6px;padding:8px 6px;display:flex;flex-direction:column;justify-content:center}
.metric .label{color:#666;font-size:9px;margin-bottom:4px;text-transform:uppercase}
.metric .value{font-size:15px;font-weight:bold;font-family:monospace}
.yellow{color:#ffd166}.blue{color:#4da3ff}.red{color:#ff4d4d}.green{color:#00ff88}
.big-box{background:#252525;border-radius:6px;padding:10px;text-align:center;margin-top:8px;flex:1;display:flex;flex-direction:column;justify-content:center}
.big-box .label{color:#888;font-size:11px;margin-bottom:4px;text-transform:uppercase}
.big-box .value{font-size:20px;font-weight:bold;font-family:monospace}
.stack{display:flex;flex-direction:column;gap:8px;flex:1}
.stack .metric{flex:1}
.center-panel{gap:8px;justify-content:space-between}
.current-power-box{background:#161616;border-radius:6px;padding:8px;text-align:center;border:1px solid #2a2a2a}
.current-power-box .label{color:#666;font-size:10px;margin-bottom:2px;text-transform:uppercase}
.current-power-box .value{font-size:18px;font-weight:bold;color:#00ff88;font-family:monospace}
.status-text{text-align:center;font-size:12px;font-weight:bold;line-height:1.6}
.status-text .machine{color:#4da3ff}.status-text .state{color:#ffd166}
.alert-box{border:2px solid #ff4d4d;border-radius:6px;padding:10px;background:rgba(255,77,77,.05);min-height:60px;box-sizing:border-box;display:flex;align-items:center;justify-content:center;text-align:center}
.alert-box .value{color:#ff4d4d;font-size:12px;font-weight:bold;line-height:1.4;word-break:break-word}
.free-power-box{background:#161616;border-radius:6px;padding:8px;text-align:center;border:1px solid #2a2a2a}
.free-power-box .label{color:#666;font-size:10px;margin-bottom:2px;text-transform:uppercase}
.free-power-box .value{font-size:18px;font-weight:bold;color:#00ff88;font-family:monospace}
.heater-slider-box{background:#161616;border-radius:6px;padding:8px;text-align:center;border:1px solid #2a2a2a;margin-top:6px}
.heater-slider-box .label{color:#666;font-size:10px;margin-bottom:2px;text-transform:uppercase}
.slider-track{background:#333;border-radius:4px;height:12px;width:100%;margin-top:4px;overflow:hidden}
.slider-fill{height:100%;transition:width .4s,background .4s;width:0%}
.slider-text{color:#888;font-size:10px;margin-top:4px;font-family:monospace}
.switch-row{display:flex;align-items:center;justify-content:center;gap:12px;margin-top:6px;padding:10px 14px;background:#161616;border-radius:6px;border:1px solid #2a2a2a}
.toggle-switch{position:relative;width:96px;height:56px;background:#333;border-radius:28px;cursor:pointer;transition:.3s;border:2px solid #444;flex-shrink:0}
.toggle-switch.active{background:rgba(0,255,136,.15);border-color:#00ff88}
.toggle-knob{position:absolute;top:4px;left:4px;width:44px;height:44px;background:#888;border-radius:50%;transition:transform .3s,background .3s}
.toggle-switch.active .toggle-knob{transform:translateX(40px);background:#00ff88}
.switch-label{color:#888;font-size:12px;font-weight:bold;text-transform:uppercase}
.controls{margin-top:15px;display:flex;flex-wrap:wrap;gap:8px;justify-content:center;max-width:960px}
.btn{background:#333;color:#fff;border:1px solid #444;padding:7px 12px;border-radius:5px;cursor:pointer;font-size:11px;font-family:inherit}
.btn:hover{background:#444}
.btn.primary{background:#00ff88;color:#000;border:none;font-weight:bold}
.btn.danger{background:#ff4d4d;color:#fff;border:none;font-weight:bold}
.update-section{background:#1e1e1e;padding:16px;border-radius:10px;margin-top:12px;width:100%;max-width:500px;box-sizing:border-box;display:none;box-shadow:0 4px 6px rgba(0,0,0,.3)}
.update-section.active{display:block}
@media(max-width:768px){.dashboard{grid-template-columns:1fr}}
</style></head><body>
<div class="dashboard">
  <div class="col">
    <div class="panel">
      <div class="panel-header"><svg viewBox="0 0 24 24"><path d="M12 7c-2.76 0-5 2.24-5 5s2.24 5 5 5 5-2.24 5-5-2.24-5-5-5z"/></svg>SOLAR POWER</div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">PV VOLTAGE</div><div class="value yellow" id="pvV">--</div></div>
        <div class="metric"><div class="label">PV CURRENT</div><div class="value yellow" id="pvI">--</div></div>
      </div>
      <div class="big-box"><div class="label">PV POWER (NOW)</div><div class="value yellow" id="pvPwr">--</div></div>
    </div>
    <div class="panel">
      <div class="panel-header blue"><svg viewBox="0 0 24 24"><path d="M13 3h-2v10h2V3z"/></svg>INVERTOR POWER</div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">INV VOLTAGE</div><div class="value blue" id="invV">--</div></div>
        <div class="metric"><div class="label">INV CURRENT</div><div class="value blue" id="invI">--</div></div>
      </div>
      <div class="big-box"><div class="label">INV POWER</div><div class="value blue" id="invVA">--</div></div>
    </div>
  </div>
  <div class="col">
    <div class="panel center-panel">
      <div class="current-power-box"><div class="label">CURRENT HEATER POWER</div><div class="value" id="curP">0 W</div></div>
      <div class="heater-slider-box">
        <div class="label">HEATER LOAD <span id="hNominal" style="color:#666"></span></div>
        <div class="slider-track"><div id="hSlider" class="slider-fill" style="width:0%;background:#00ff88"></div></div>
        <div id="hSliderText" class="slider-text">0%</div>
      </div>
      <div class="switch-row">
        <span class="switch-label">СПОЖИВАННЯ</span>
        <div class="toggle-switch" id="switchBtn" onclick="toggleSwitch()"><div class="toggle-knob"></div></div>
        <span class="switch-label" id="switchStatus" style="color:#888">ВИМК</span>
      </div>
      <div class="status-text"><span class="machine" id="machineInfo">--</span><br><span class="state" id="stateInfo">--</span></div>
      <div class="alert-box" id="alertBox"><div class="value" id="targMsg">--</div></div>
      <div class="free-power-box"><div class="label">FREE POWER</div><div class="value" id="freeP">0 W</div></div>
    </div>
  </div>
  <div class="col">
    <div class="panel">
      <div class="panel-header blue"><svg viewBox="0 0 24 24"><path d="M15.67 4H14V2h-4v2H8.33C7.6 4 7 4.6 7 5.33v15.33C7 21.4 7.6 22 8.33 22h7.33c.74 0 1.34-.6 1.34-1.33V5.33C17 4.6 16.4 4 15.67 4z"/></svg>BATTERY</div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">BATTERY VOLTAGE</div><div class="value blue" id="battV">--</div></div>
        <div class="metric"><div class="label">BATTERY CURRENT</div><div class="value blue" id="battI">--</div></div>
      </div>
      <div class="big-box"><div class="label">BATTERY POWER</div><div class="value" id="battPwr">--</div></div>
      <div class="metric" style="margin-top:8px"><div class="label">BATTERY CAPACITY</div><div class="value green" id="battCap">--</div></div>
    </div>
    <div class="panel">
      <div class="panel-header"><svg viewBox="0 0 24 24"><path d="M11 21h-1l1-7H7.5c-.58 0-.57-.32-.38-.66.19-.34.05-.08.07-.12C8.48 10.94 10.42 7.54 13 3h1l-1 7h3.5c.49 0 .56.33.47.51l-.07.15C12.96 17.55 11 21 11 21z"/></svg>POWER INFO</div>
      <div class="stack">
        <div class="metric"><div class="label">GRID FREQ</div><div class="value blue" id="gridFreq">--</div></div>
        <div class="metric"><div class="label">LOAD</div><div class="value yellow" id="loadPct">--</div></div>
        <div class="metric"><div class="label">TEMP</div><div class="value green" id="tempC">--</div></div>
      </div>
    </div>
  </div>
</div>
<div class="controls">
<button class="btn primary" onclick="update()">Refresh</button>
<button class="btn" onclick="location.href='/settings'">Settings</button>
<button class="btn" id="signalBtn" onclick="location.href='/signal'">Сигнал</button>
<button class="btn" onclick="location.href='/serial'">Serial Monitor</button>
<button class="btn" onclick="resetWifi()">Reset WiFi</button>
<button class="btn danger" onclick="restartEsp()">Restart ESP</button>
<button class="btn" id="fwBtn" onclick="toggleUpdate()">Firmware Update</button>
</div>
<div class="update-section" id="updateSection">
<b style="color:#00ff88;font-size:11px">FIRMWARE UPDATE</b><br><br>
<input type="file" id="fwfile" accept=".bin" style="width:100%;padding:6px;background:#333;color:#fff;border:1px solid #555;border-radius:5px;box-sizing:border-box;font-size:11px">
<button class="btn primary" onclick="doUpdate()" style="width:100%;margin-top:6px">Update firmware</button>
<div id="prog" style="height:3px;background:#333;border-radius:2px;margin-top:6px;overflow:hidden;display:none"><div id="pbar" style="height:100%;width:0;background:#00ff88"></div></div>
<div id="ust" style="font-size:9px;color:#888;margin-top:4px">Select .bin file</div>
</div>
<script>
function wsStr(v){
  switch(v){
    case 0:return"Power On";case 1:return"Standby";case 2:return"Line (Bypass)";
    case 3:return"Battery (Off-Grid)";case 4:return"Fault";case 5:return"Power Save";
    case 6:return"Shutdown";default:return"Unknown";
  }
}
function isBad(v){return v===null||v===undefined||v===65535}
function fmt1(v,u){if(isBad(v))return"--";return Number(v).toFixed(1)+(u?" "+u:"")}
function fmt2(v,u){if(isBad(v))return"--";return Number(v).toFixed(2)+(u?" "+u:"")}
function fmt0(v,u){if(isBad(v))return"--";return Math.round(Number(v))+(u?" "+u:"")}
function toggleUpdate(){document.getElementById("updateSection").classList.toggle("active")}
async function resetWifi(){if(confirm("Reset WiFi?")){await fetch("/reset_wifi");location.reload()}}
async function restartEsp(){if(!confirm("Restart ESP32?"))return;await fetch("/restart");alert("Restarting...");setTimeout(()=>location.reload(),5000)}
function doUpdate(){
  let f=document.getElementById("fwfile").files[0];
  if(!f){alert("Select .bin file");return}
  if(!confirm("Start firmware update?"))return;
  let fd=new FormData();fd.append("update",f,f.name);
  let x=new XMLHttpRequest();
  document.getElementById("prog").style.display="block";
  document.getElementById("ust").innerText="Uploading...";
  x.open("POST","/update");
  x.upload.onprogress=e=>{if(e.lengthComputable){let p=Math.round(e.loaded/e.total*100);document.getElementById("pbar").style.width=p+"%";document.getElementById("ust").innerText="Upload: "+p+"%"}};
  x.onload=()=>{if(x.status==200){document.getElementById("ust").innerText="OK! Restarting...";setTimeout(()=>location.reload(),8000)}else document.getElementById("ust").innerText="ERROR: "+x.responseText};
  x.onerror=()=>document.getElementById("ust").innerText="Connection error";
  x.send(fd);
}
async function toggleSwitch(){
  try{
    let resp=await fetch("/toggle_switch",{method:"POST"});
    if(resp.status==403){alert("Access denied");return}
    let data=await resp.json();updateSwitchUI(data.state);
  }catch(e){alert("Connection error!")}
}
function updateSwitchUI(state){
  let s=document.getElementById("switchBtn"),t=document.getElementById("switchStatus");
  if(state){s.classList.add("active");t.innerText="УВІМК";t.style.color="#00ff88";}
  else{s.classList.remove("active");t.innerText="ВИМК";t.style.color="#888";}
}
async function update(){
  try{
    let d=await(await fetch("/api")).json();

    let modelName = ((d.mt && d.mt !== "UNKNOWN") ? d.mt + " " : "") + ((d.mp && d.mp !== "—") ? d.mp : "");
    document.getElementById("machineInfo").innerText = modelName;
    document.getElementById("stateInfo").innerText = "Mode: " + wsStr(d.ws);

    document.getElementById("pvV").innerText  = fmt1(d.pvV,"V");
    document.getElementById("pvI").innerText  = fmt1(d.pvI,"A");
    document.getElementById("pvPwr").innerText= fmt0(d.pvPower,"W");

    document.getElementById("invV").innerText  = fmt1(d.invV,"V");
    document.getElementById("invI").innerText  = fmt1(d.invI,"A");
    document.getElementById("invVA").innerText = fmt0(d.invVA,"VA");

    document.getElementById("gridFreq").innerText = (d.freq>0)?d.freq.toFixed(1)+" Hz":"--";
    document.getElementById("loadPct").innerText  = fmt0(d.loadPct,"%");
    document.getElementById("tempC").innerText    = fmt0(d.tempC,"°C");

    document.getElementById("battV").innerText = (d.battV>0)?d.battV.toFixed(2)+" V":"--";
    let bi=document.getElementById("battI");
    bi.innerText=d.battI.toFixed(2)+" A";
    bi.className="value "+(d.battI>=0?"blue":"red");
    let bp=document.getElementById("battPwr");
    bp.innerText=Math.round(d.battP)+" W";
    bp.className="value "+(d.battP>=0?"blue":"red");
    document.getElementById("battCap").innerText = fmt0(d.battCap,"%");

    document.getElementById("curP").innerText  = fmt0(d.cp,"W");
    document.getElementById("freeP").innerText = fmt0(d.fp,"W");
    let hpct=d.hpct||0;
    let h=document.getElementById("hSlider");
    h.style.width=hpct+"%";
    let hc = hpct<33 ? "#00ff88" : (hpct<66 ? "#ffd166" : "#ff4d4d");
    h.style.background=hc;
    document.getElementById("hSliderText").innerText=hpct+"%";
    document.getElementById("hSliderText").style.color=hc;
    if(d.hva) document.getElementById("hNominal").innerText="("+d.hva+" VA nominal)";
    updateSwitchUI(d.sw);

    let sigBtn=document.getElementById("signalBtn");
    if(sigBtn) sigBtn.innerText="Сигнал: "+((d.rssi!=null&&d.rssi!==0)?d.rssi+" dBm":"--");

    let tm=d.tm||"--",tmEl=document.getElementById("targMsg"),box=document.getElementById("alertBox");
    tmEl.innerText=tm;
    if(d.pollOk===0){
      tmEl.innerText="Нема зв'язку з інвертором (RX="+d.respLen+" байт)";
      tmEl.style.color="#ff4d4d";
      box.style.borderColor="#ff4d4d";
      box.style.background="rgba(255,77,77,.05)";
    } else if(tm==="Все добре"){
      tmEl.style.color="#00ff88";
      box.style.borderColor="#00ff88";
      box.style.background="rgba(0,255,136,.05)";
    } else {
      tmEl.style.color="#ff4d4d";
      box.style.borderColor="#ff4d4d";
      box.style.background="rgba(255,77,77,.05)";
    }
  }catch(e){console.error(e)}
}
setInterval(update,1000);update();
</script></body></html>
)rawliteral";

// ================== HTML: SERIAL ==================
const char SERIAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Serial Monitor</title>
<style>
body{background:#0f0f0f;color:#e6e6e6;font-family:monospace;padding:15px;margin:0}
h1{color:#00ff88;text-align:center;font-size:18px}
#log{background:#1c1c1c;border:1px solid #333;border-radius:8px;padding:10px;height:70vh;overflow-y:auto;font-size:12px;line-height:1.6;white-space:pre-wrap;word-break:break-all}
.btn{background:#00ff88;color:#000;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-weight:bold;margin:3px}
.bg{background:#555;color:#fff}.ctr{text-align:center;margin:10px 0}
</style></head><body>
<h1>Serial Monitor</h1>
<div class="ctr">
<button class="btn" onclick="fetchLog()">Refresh</button>
<button class="btn bg" onclick="document.getElementById('log').innerText=''">Clear</button>
<button class="btn bg" id="autoBtn" onclick="toggleAuto()">Auto-scroll: ON</button>
<button class="btn bg" id="dumpBtn" onclick="toggleDump()">Dump: OFF</button>
<button class="btn bg" onclick="location.href='/'">Back</button>
</div>
<div id="log">Loading...</div>
<script>
let autoScroll=true, dumpOn=false;
function toggleAuto(){autoScroll=!autoScroll;document.getElementById("autoBtn").innerText=autoScroll?"Auto-scroll: ON":"Auto-scroll: OFF";}
function updDump(){let b=document.getElementById("dumpBtn");b.innerText=dumpOn?"Dump: ON":"Dump: OFF";b.style.background=dumpOn?"#00ff88":"#555";b.style.color=dumpOn?"#000":"#fff";}
async function loadDump(){try{let d=await(await fetch("/api_dump")).json();dumpOn=(d.dump===1);}catch(e){}updDump();}
async function toggleDump(){dumpOn=!dumpOn;try{let d=await(await fetch("/api_dump?state="+(dumpOn?"1":"0"))).json();dumpOn=(d.dump===1);}catch(e){}updDump();}
async function fetchLog(){try{let t=await(await fetch("/api_serial")).text();let el=document.getElementById("log");el.innerText=t;if(autoScroll)el.scrollTop=el.scrollHeight;}catch(e){document.getElementById("log").innerText="Error: "+e;}}
loadDump();fetchLog();setInterval(fetchLog,2000);
</script></body></html>
)rawliteral";

// ================== HTML: SETTINGS ==================
const char SETTINGS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Settings</title>
<style>
body{background:#0f0f0f;color:#e6e6e6;font-family:monospace;padding:15px;margin:0}
h1{color:#00ff88;text-align:center;font-size:18px}h2{color:#4da3ff;font-size:14px;margin-top:20px}
.card{background:#1c1c1c;border-radius:10px;padding:15px;margin:10px 0}
label{color:#ccc;font-size:13px;font-weight:bold;display:block;margin-top:14px}
.desc{color:#888;font-size:11px;margin:4px 0 8px 0;line-height:1.5;background:#141414;padding:8px 10px;border-left:3px solid #4da3ff;border-radius:3px}
.hint{color:#ffd166;font-size:10px;margin:2px 0 6px 0;line-height:1.4;font-style:italic}
input{width:100%;box-sizing:border-box;padding:10px;margin:5px 0;background:#333;color:#fff;border:1px solid #555;border-radius:6px;font-size:14px}
.btn{background:#00ff88;color:#000;border:none;padding:10px 20px;border-radius:6px;cursor:pointer;font-weight:bold;margin:5px}
.bg{background:#555;color:#fff}.msg{color:#00ff88;text-align:center;margin:10px 0;display:none}
.err{color:#ff4d4d;text-align:center;margin:10px 0;display:none}
</style></head><body>
<h1>Settings</h1>
<div class="msg" id="msg">Saved!</div><div class="err" id="err">Access denied</div>

<h2>General</h2><div class="card">
<label>Heater power (VA)</label>
<div class="desc">Номінальна потужність ТЕНа. Від неї залежить відсоток навантаження.</div>
<input type="number" id="heaterVA" min="100" max="15000" value="1000">

<label>Max allocated power (VA)</label>
<div class="desc">Стеля потужності на ТЕН.</div>
<input type="number" id="maxAllocVA" min="100" max="15000" value="500">

<label>Update interval (sec)</label>
<div class="desc">Як часто опитується інвертор. Рекомендовано 1.0–3.0 сек.</div>
<input type="number" id="updIntSec" min="0.5" max="10" step="0.1" value="2">
</div>

<h2>Load shutdown</h2><div class="card">
<label>Hysteresis (V)</label>
<input type="number" id="battHystV" min="0" max="5" step="0.1" value="0.5">

<label>Shutdown voltage (V)</label>
<input type="number" id="battOffV" min="20" max="80" step="0.1" value="27">
</div>

<h2>Battery charging</h2><div class="card">
<label>Full charge voltage (V)</label>
<input type="number" id="fullChgV" min="20" max="80" step="1" value="29">

<label>Charge current (A)</label>
<div class="desc">Додатнє — зарезервувати струм для заряджання АКБ. Від'ємне — дозволити ТЕНу брати з АКБ.</div>
<input type="number" id="chgCurA" min="-100" max="100" step="1" value="0">

<label>Battery discharge allow (W)</label>
<div class="desc">Дозвіл брати з АКБ на ТЕН.</div>
<input type="number" id="battDisW" min="0" max="15000" step="50" value="0">
</div>

<div style="text-align:center;margin-top:15px">
<button class="btn" onclick="save()">Save</button>
<button class="btn bg" onclick="location.href='/'">Back</button>
</div>
<script>
async function load(){
  try{
    let d=await(await fetch("/api_settings")).json();
    ["heaterVA","maxAllocVA","updIntSec","battOffV","battHystV","chgCurA","fullChgV","battDisW"]
      .forEach(k=>document.getElementById(k).value=d[k]);
  }catch(e){}
}
async function save(){
  let body=["heaterVA","maxAllocVA","updIntSec","battOffV","battHystV","chgCurA","fullChgV","battDisW"]
    .map(k=>k+"="+document.getElementById(k).value).join("&");
  try{
    let r=await fetch("/save_settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body});
    if(r.status==200){
      document.getElementById("msg").style.display="block";
      document.getElementById("err").style.display="none";
      setTimeout(()=>document.getElementById("msg").style.display="none",2000);
    } else if(r.status==403){
      document.getElementById("err").style.display="block";
      document.getElementById("msg").style.display="none";
      setTimeout(()=>document.getElementById("err").style.display="none",3000);
    } else alert("Save error!");
  }catch(e){alert("Connection error!");}
}
load();
</script></body></html>
)rawliteral";

// ================== HTML: SIGNAL ==================
const char SIGNAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Рівень сигналу</title>
<style>
body{background:#0f0f0f;color:#e6e6e6;font-family:monospace;padding:15px;margin:0}
h1{color:#00ff88;text-align:center;font-size:18px}
.card{background:#1c1c1c;border-radius:10px;padding:20px;max-width:420px;margin:12px auto}
.big{text-align:center;font-size:36px;font-weight:bold;margin:8px 0}
.small{text-align:center;color:#888;font-size:12px;margin-bottom:14px}
.bar{height:14px;background:#333;border-radius:7px;overflow:hidden;margin:10px 0 18px}
.fill{height:100%;width:0;background:#00ff88;transition:.4s}
.row{display:flex;justify-content:space-between;padding:7px 0;border-bottom:1px solid #2a2a2a;font-size:12px}
.row:last-child{border-bottom:none}.k{color:#888}.v{font-weight:bold;text-align:right;word-break:break-all}
.btn{background:#00ff88;color:#000;border:none;padding:10px 18px;border-radius:6px;cursor:pointer;font-weight:bold;margin:5px}
.bg{background:#555;color:#fff}.center{text-align:center;margin-top:14px}
</style></head><body>
<h1>Рівень сигналу</h1>
<div class="card">
<div class="big" id="q">--</div><div class="small" id="rssi">--</div>
<div class="bar"><div class="fill" id="fill"></div></div>
<div class="row"><span class="k">Статус</span><span class="v" id="status">--</span></div>
<div class="row"><span class="k">Мережа</span><span class="v" id="ssid">--</span></div>
<div class="row"><span class="k">IP</span><span class="v" id="ip">--</span></div>
<div class="row"><span class="k">MAC</span><span class="v" id="mac">--</span></div>
</div>
<div class="center"><button class="btn bg" onclick="location.href='/'">Назад</button></div>
<script>
function esc(s){return String(s||'--').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}
async function upd(){
  try{
    let d=await(await fetch('/api_signal')).json();
    let q=d.quality||0;
    document.getElementById('q').innerText=q+'%';
    document.getElementById('rssi').innerText='RSSI: '+((d.rssi!=null&&d.rssi!==0)?d.rssi+' dBm':'--');
    let f=document.getElementById('fill');
    f.style.width=q+'%';
    f.style.background=q>70?'#00ff88':(q>40?'#ffd166':'#ff4d4d');
    document.getElementById('status').innerText=d.status==='CONNECTED'?'Підключено':'Не підключено';
    document.getElementById('ssid').innerText=esc(d.ssid);
    document.getElementById('ip').innerText=esc(d.ip);
    document.getElementById('mac').innerText=esc(d.mac);
  }catch(e){document.getElementById('q').innerText='Помилка'}
}
upd();setInterval(upd,2000);
</script></body></html>
)rawliteral";

// ================== HTML: SETUP AP ==================
const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi Setup</title>
<style>body{background:#0f0f0f;color:#fff;font-family:monospace;padding:20px;text-align:center}
.card{background:#1c1c1c;border-radius:12px;padding:20px;max-width:360px;margin:0 auto}
select,input,button{width:100%;box-sizing:border-box;padding:12px;margin:8px 0;background:#333;color:#fff;border:1px solid #555;border-radius:8px}
button{background:#00ff88;color:#000;font-weight:bold;cursor:pointer;border:none}</style></head><body>
<div class="card"><h2>WiFi Setup</h2>
<form action="/save_wifi" method="POST">
<select name="ssid" id="ss" required><option>Scanning...</option></select>
<input type="password" name="password" placeholder="Password">
<button type="submit">Save</button></form></div>
<script>fetch("/scan_wifi").then(r=>r.json()).then(n=>{let s=document.getElementById("ss");s.innerHTML="";
if(!n.length){s.innerHTML="<option value=''>No networks</option>";return}
n.forEach(x=>{let o=document.createElement("option");o.value=x.ssid;o.text=x.ssid+" ("+x.rssi+")";s.appendChild(o)})})</script></body></html>
)rawliteral";

// ================== SYSTEM ==================
void triggerReboot() { needRestart = true; restartRequestedMs = millis(); }

static String jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:
        if ((uint8_t)c < 0x20) {
          char b[8]; snprintf(b, sizeof(b), "\\u%04x", (uint8_t)c); out += b;
        } else out += c;
    }
  }
  return out;
}

void startSetupAP() {
  setupMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Inverter-Setup");
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) { r->send_P(200, "text/html", SETUP_HTML); });

  server.on("/scan_wifi", HTTP_GET, [](AsyncWebServerRequest* r) {
    int n = WiFi.scanNetworks();
    auto* res = r->beginResponseStream("application/json");
    res->print("[");
    for (int i = 0; i < n; i++) {
      if (i) res->print(",");
      res->print("{\"ssid\":\"");
      res->print(jsonEscape(WiFi.SSID(i)));
      res->print("\",\"rssi\":");
      res->print(WiFi.RSSI(i));
      res->print("}");
    }
    res->print("]");
    WiFi.scanDelete();
    r->send(res);
  });

  server.on("/save_wifi", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (r->hasParam("ssid", true) && r->hasParam("password", true)) {
      preferences.begin("wifi_config", false);
      preferences.putString("ssid", r->getParam("ssid", true)->value());
      preferences.putString("password", r->getParam("password", true)->value());
      preferences.end();
      r->send(200, "text/html", "<h3 style='color:white;text-align:center'>Saved! Rebooting...</h3>");
      triggerReboot();
    } else r->send(400, "text/plain", "Bad Request");
  });
  server.onNotFound([](AsyncWebServerRequest* r) { r->send_P(200, "text/html", SETUP_HTML); });
  server.begin();
}

// ================== OTA / WEB ==================
void onUpdateUpload(AsyncWebServerRequest* request, String filename, size_t index,
                    uint8_t* data, size_t len, bool final) {
  if (!request->client() || !isLocalIP(request->client()->remoteIP())) {
    if (!index) { Update.abort(); webUpdateActive = false; }
    return;
  }
  lastWebUpdateActivityMs = millis();
  if (!index) {
    webUpdateActive = true;
    if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) {
      Update.printError(Serial);
      webUpdateActive = false;
      return;
    }
  }
  if (!Update.hasError() && Update.write(data, len) != len) Update.printError(Serial);
  if (final) {
    if (Update.end(true)) Serial.printf("Update OK: %u bytes\n", (unsigned)(index + len));
    else { Update.printError(Serial); webUpdateActive = false; }
  }
}

int wifiQualityPercent(int rssi) {
  if (rssi == 0) return 0;
  if (rssi >= -50) return 100;
  if (rssi <= -90) return 0;
  return (int)((rssi + 90) * 2.5f);
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) { r->send_P(200, "text/html", INDEX_HTML); });

  server.on("/api", HTTP_GET, [](AsyncWebServerRequest *r) {
    bool locked = dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) == pdTRUE;

    char targCopy[sizeof(targMessageGlobal)];
    strncpy(targCopy, targMessageGlobal, sizeof(targCopy));
    targCopy[sizeof(targCopy) - 1] = '\0';

    DynamicJsonDocument doc(3072);
    doc["ws"]  = inv.workMode;
    doc["mt"]  = inv.machineType;
    doc["mp"]  = inv.machinePower;
    doc["fp"]  = (int)freePower;
    doc["cp"]  = (int)currentPower;
    doc["pph"] = (int)PpowerHeater;
    doc["hpct"]= currentHeaterPercent;
    doc["hva"] = (unsigned)cfgHeaterPowerVA;
    doc["maVA"]= (unsigned)cfgMaxAllocatedVA;
    doc["sw"]  = switch_state ? 1 : 0;
    doc["tm"]  = targCopy;
    doc["pvV"]     = pvV;
    doc["pvI"]     = pvI;
    doc["pvPower"] = (int)pvPower;
    doc["systemV"] = systemV;
    doc["invV"]    = invV;
    doc["invI"]    = invI;
    doc["invVA"]   = (int)acOutVA;
    doc["invPower"]= (int)invPower;
    doc["battV"]   = battV;
    doc["battI"]   = battI;
    doc["battP"]   = battP;
    doc["battCap"] = battCapPct;
    doc["tempC"]   = battTempC;
    doc["freq"]    = gridFreq;
    doc["loadPct"] = loadPercent;
    doc["rssi"]    = WiFi.RSSI();
    doc["isLocal"] = (r->client() && isLocalIP(r->client()->remoteIP())) ? 1 : 0;
    doc["pollOk"]  = lastPollOk ? 1 : 0;
    doc["respLen"] = (unsigned)lastRespLen;

    JsonArray raw = doc.createNestedArray("raw");
    for (int i = 0; i < inv.f_count; i++) raw.add(inv.f_raw[i]);

    auto* res = r->beginResponseStream("application/json; charset=utf-8");
    serializeJson(doc, *res);
    r->send(res);
    if (locked) xSemaphoreGive(dataMutex);
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *r) { r->send_P(200, "text/html", SETTINGS_HTML); });

  server.on("/api_settings", HTTP_GET, [](AsyncWebServerRequest *r) {
    DynamicJsonDocument doc(512);
    doc["heaterVA"]   = (unsigned)cfgHeaterPowerVA;
    doc["maxAllocVA"] = (unsigned)cfgMaxAllocatedVA;
    doc["updIntSec"]  = cfgUpdateIntervalSec;
    doc["battOffV"]   = cfgBattOffVoltage;
    doc["battHystV"]  = cfgBattHysteresisV;
    doc["chgCurA"]    = cfgFullChargeCurrentA;
    doc["fullChgV"]   = cfgFullChargeVoltage;
    doc["battDisW"]   = cfgBattDischargeAllowW;
    auto* res = r->beginResponseStream("application/json");
    serializeJson(doc, *res);
    r->send(res);
  });

  server.on("/serial", HTTP_GET, [](AsyncWebServerRequest *r) { r->send_P(200, "text/html", SERIAL_HTML); });

  server.on("/api_dump", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (r->hasParam("state")) {
      String v = r->getParam("state")->value();
      dumpEnabled = (v == "1");
    }
    auto* res = r->beginResponseStream("application/json");
    res->printf("{\"dump\":%d}", dumpEnabled ? 1 : 0);
    r->send(res);
  });

  server.on("/api_serial", HTTP_GET, [](AsyncWebServerRequest *r) {
    auto* res = r->beginResponseStream("text/plain; charset=utf-8");
    uint16_t t;
    portENTER_CRITICAL(&serialMux);
    t = serialTail;
    portEXIT_CRITICAL(&serialMux);

    char chunk[64];
    while (true) {
      uint16_t chunkLen = 0;
      portENTER_CRITICAL(&serialMux);
      uint16_t h = serialHead;
      while (t != h && chunkLen < sizeof(chunk)) {
        chunk[chunkLen++] = serialBuf[t];
        t = (t + 1) % SERIAL_BUF_SIZE;
      }
      portEXIT_CRITICAL(&serialMux);
      if (chunkLen == 0) break;
      res->write((const uint8_t*)chunk, chunkLen);
    }
    r->send(res);
  });

  server.on("/signal", HTTP_GET, [](AsyncWebServerRequest *r) { r->send_P(200, "text/html", SIGNAL_HTML); });

  server.on("/api_signal", HTTP_GET, [](AsyncWebServerRequest *r) {
    int rssi = WiFi.RSSI();
    DynamicJsonDocument doc(256);
    doc["rssi"]    = rssi;
    doc["quality"] = wifiQualityPercent(rssi);
    doc["status"]  = (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "DISCONNECTED";
    doc["ssid"]    = WiFi.SSID();
    doc["ip"]      = WiFi.localIP().toString();
    doc["mac"]     = WiFi.macAddress();
    auto* res = r->beginResponseStream("application/json; charset=utf-8");
    serializeJson(doc, *res);
    r->send(res);
  });

  server.on("/save_settings", HTTP_POST, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      uint16_t newHeaterVA  = cfgHeaterPowerVA;
      uint16_t newMaxAlloc  = cfgMaxAllocatedVA;
      float newUpdInt    = cfgUpdateIntervalSec;
      float newBattOffV  = cfgBattOffVoltage;
      float newBattHystV = cfgBattHysteresisV;
      float newChgCurA   = cfgFullChargeCurrentA;
      float newFullChgV  = cfgFullChargeVoltage;
      float newBattDisW  = cfgBattDischargeAllowW;

      if (req->hasParam("heaterVA",   true)) newHeaterVA  = constrain(req->getParam("heaterVA",   true)->value().toInt(),   100,    15000);
      if (req->hasParam("maxAllocVA", true)) newMaxAlloc  = constrain(req->getParam("maxAllocVA", true)->value().toInt(),   100,    15000);
      if (req->hasParam("updIntSec",  true)) newUpdInt    = constrain(req->getParam("updIntSec",  true)->value().toFloat(), 0.5f,   10.0f);
      if (req->hasParam("battOffV",   true)) newBattOffV  = constrain(req->getParam("battOffV",   true)->value().toFloat(), 20.0f,  80.0f);
      if (req->hasParam("battHystV",  true)) newBattHystV = constrain(req->getParam("battHystV",  true)->value().toFloat(), 0.0f,   10.0f);
      if (req->hasParam("chgCurA",    true)) newChgCurA   = constrain(req->getParam("chgCurA",    true)->value().toFloat(), -100.0f, 100.0f);
      if (req->hasParam("fullChgV",   true)) newFullChgV  = constrain(req->getParam("fullChgV",   true)->value().toFloat(), 20.0f,  80.0f);
      if (req->hasParam("battDisW",   true)) newBattDisW  = constrain(req->getParam("battDisW",   true)->value().toFloat(), 0.0f,   15000.0f);

      if (dataMutex && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        cfgHeaterPowerVA       = newHeaterVA;
        cfgMaxAllocatedVA      = newMaxAlloc;
        cfgUpdateIntervalSec   = newUpdInt;
        cfgBattOffVoltage      = newBattOffV;
        cfgBattHysteresisV     = newBattHystV;
        cfgFullChargeCurrentA  = newChgCurA;
        cfgFullChargeVoltage   = newFullChgV;
        cfgBattDischargeAllowW = newBattDisW;
        xSemaphoreGive(dataMutex);
      }
      saveSettings();
      req->send(200, "text/plain", "OK");
    });
  });

  server.on("/toggle_switch", HTTP_POST, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      switch_state = !switch_state;
      if (!switch_state && dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        freePower = 0; currentPower = 0; PpowerHeater = 0; currentHeaterPercent = 0;
        strncpy(targMessageGlobal, "Споживання вимкнено", sizeof(targMessageGlobal) - 1);
        targMessageGlobal[sizeof(targMessageGlobal) - 1] = '\0';
        xSemaphoreGive(dataMutex);
      }
      saveSettings();
      auto* res = req->beginResponseStream("application/json");
      res->printf("{\"state\":%d}", switch_state ? 1 : 0);
      req->send(res);
    });
  });

  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) { req->send(200, "text/plain", "OK"); triggerReboot(); });
  });

  server.on("/reset_wifi", HTTP_GET, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      preferences.begin("wifi_config", false); preferences.clear(); preferences.end();
      req->send(200, "text/plain", "OK"); triggerReboot();
    });
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (!r->client() || !isLocalIP(r->client()->remoteIP())) {
      webUpdateActive = false;
      Update.abort();
      r->send(403, "text/plain", "Access denied: Local network only");
      return;
    }
    if (Update.hasError()) { webUpdateActive = false; r->send(500, "text/plain", "FAIL"); }
    else { r->send(200, "text/plain", "OK"); triggerReboot(); }
  }, onUpdateUpload);

  server.onNotFound([](AsyncWebServerRequest *r) { r->redirect("/"); });
  server.begin();
}

// ================== CRC self-test ==================
void pi30_crcSelfTest() {
  uint8_t frame[16];
  size_t len;
  pi30_buildCmd("QPIGS", frame, len);
  const uint8_t expected[8] = {0x51,0x50,0x49,0x47,0x53,0xB7,0xA9,0x0D};
  bool ok = (len == 8) && (memcmp(frame, expected, 8) == 0);

  char buf[128];
  int p = snprintf(buf, sizeof(buf), ">>> CRC self-test QPIGS: ");
  for (size_t i = 0; i < len; i++)
    p += snprintf(buf + p, sizeof(buf) - p, "%02X ", frame[i]);
  p += snprintf(buf + p, sizeof(buf) - p, "-> %s\n", ok ? "OK" : "MISMATCH!");
  serialLog(buf);
}

void setupOTA() {
  ArduinoOTA.setHostname("inverter");
  ArduinoOTA.setPassword("admin");
  ArduinoOTA.onStart([]() { serialLog("OTA: start\n"); });
  ArduinoOTA.onEnd  ([]() { serialLog("OTA: done\n"); });
  ArduinoOTA.onError([](ota_error_t e) {
    char b[64]; snprintf(b, sizeof(b), "OTA error [%u]\n", e); serialLog(b);
  });
  ArduinoOTA.begin();
}

// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(115200);

  pinMode(TRIAC_PIN, OUTPUT);
  digitalWrite(TRIAC_PIN, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);

  Inv.begin(INVERTER_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  vTaskDelay(pdMS_TO_TICKS(300));

  pi30_crcSelfTest();

  WiFi.persistent(false);
  preferences.begin("wifi_config", true);
  wifiSsid     = preferences.getString("ssid", "");
  wifiPassword = preferences.getString("password", "");
  preferences.end();
  loadSettings();

  if (wifiSsid.length()) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < WIFI_CONNECT_TIMEOUT_MS) delay(300);
  }
  if (WiFi.status() != WL_CONNECTED) { startSetupAP(); return; }

  MDNS.begin("inverter");
  dataMutex     = xSemaphoreCreateMutex();
  inverterMutex = xSemaphoreCreateMutex();

  setupWebServer();
  setupOTA();

  xTaskCreatePinnedToCore(triacTask,    "triacTask",    2048, nullptr, 2, &triacTaskHandle,    0);
  xTaskCreatePinnedToCore(inverterTask, "inverterTask", 4096, nullptr, 1, &inverterTaskHandle, 0);

  serialLog("=== System started (PI30 QPIGS ECO II, v8.0.3) ===\n");
}

void loop() {
  unsigned long now = millis();
  if (needRestart && now - restartRequestedMs >= RESTART_DELAY_MS) ESP.restart();

  if (setupMode) {
    dnsServer.processNextRequest();
    vTaskDelay(pdMS_TO_TICKS(5));
    return;
  }

  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();

  if (webUpdateActive && (now - lastWebUpdateActivityMs) > WEB_UPDATE_TIMEOUT_MS) {
    webUpdateActive = false;
    Update.abort();
    serialLog("OTA timeout — cleared\n");
  }

  if (WiFi.status() != WL_CONNECTED && now - lastWifiReconnectMs >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWifiReconnectMs = now;
    WiFi.reconnect();
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}
