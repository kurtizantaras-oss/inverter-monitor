/*
 * Inverter Monitor & Heater Controller — PI30 (QPIGS) for ECO II 4.2kW Plus
 * v8.6.9 — SSE + насос (power-gated) + post-run + история 7 суток
 *          + /history + счётчики энергии PV/Grid (кВт·ч, NVS, сброс)
 *          + Settings: блоки + подробные описания
 *          + UI: реальная мощность ТЭНа (actualTriacDuty, усреднённая за окно ШИМ)
 *          + energyGridWh только mode==2, nvsMutex, saveEnergy(), histStarted
 *          + сохранение истории в NVS (namespace "hist"), загрузка при старте
 *          + УБРАН pre-start: ТЭН греет сразу, как только насос запустился
 *          + FIX: target обнуляется при !pumpAllowed (ТЭН не греет в post-run)
 *          + FIX: NVS-история разбита на 2 ключа (b1/b2) — не теряется
 *          + FIX: currentPower обрезается до cfgMaxAllocatedVA
 *          + FIX: currentHeaterPercent сбрасывается при 3+ подряд неудачных опросах
 *          + FIX: saveEnergy раз в 10 мин + только при изменении (бережём NVS)
 *          + OPT: pi30_query → static char[] (без String в горячем пути)
 *          + OPT: pi30_parseQPIGS → const char* (без String)
 *          + OPT: защита от деления на 0 в triacTask (dutyTicks > 0)
 *
 * QPIGS-раскладка (0-based индексы):
 *   0 gridV 1 gridHz 2 acOutV 3 acOutHz 4 acOutVA 5 acOutW 6 loadPct 7 busV
 *   8 battV 9 battChgA 10 battCap 11 tempC 12 pvCurrentA 13 pvVoltage 14 sccBattV
 *  15 battDischRaw 16 deviceStatus 17 fanOffset 18 eepromVer 19 pvPowerW 20 status2
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
#define PUMP_PIN       18
#define SKETCH_VERSION "8.6.9-pi30-sse-pump-power-actual-smooth-hist-energy-nvs-theme"
#define SERIAL_BUF_SIZE 4096

constexpr uint32_t TRIAC_WINDOW_MS = 1000;
constexpr uint32_t TRIAC_TICK_MS   = 10;
constexpr uint32_t INVERTER_BAUD   = 2400;

uint32_t cfgPumpPostrunMs  = 30000;
int32_t  cfgPumpMinFreeW   = 0;

constexpr uint32_t HIST_BUCKET_MS  = 30UL * 60UL * 1000UL;
constexpr uint16_t HIST_BUCKETS    = 336;
constexpr uint32_t HIST_PUSH_EVERY = 5UL * 60UL * 1000UL;
constexpr uint16_t HIST_PER_DAY    = 48;

struct HistBucket {
  int16_t  pv;
  int16_t  inv;
  int16_t  batt;
  uint32_t ts;
};
HistBucket histBuf[HIST_BUCKETS];
uint16_t   histHead     = 0;
uint16_t   histCount    = 0;
uint32_t   histWinStart = 0;
bool       histStarted  = false;
int32_t    histAccPv    = 0;
int32_t    histAccInv   = 0;
int32_t    histAccBatt  = 0;
uint32_t   histAccN     = 0;
uint32_t   lastHistPushMs = 0;
uint32_t   lastHistSaveMs = 0;

double   energyPvWh   = 0.0;
double   energyGridWh = 0.0;
uint32_t lastEnergyMs = 0;

volatile uint8_t currentTriacDuty     = 0;
volatile uint8_t currentHeaterPercent = 0;
volatile uint8_t actualTriacDuty      = 0;
volatile bool    pumpRunning          = false;

TaskHandle_t triacTaskHandle    = nullptr;
TaskHandle_t inverterTaskHandle = nullptr;

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS     = 12000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS  = 10000;
constexpr uint32_t RESTART_DELAY_MS            = 2000;
constexpr uint32_t WEB_UPDATE_TIMEOUT_MS       = 30000;
constexpr uint32_t ENERGY_SAVE_INTERVAL_MS     = 600000UL;   // 10 мин
constexpr float    DEFAULT_UPDATE_INTERVAL_SEC = 2.0f;

SemaphoreHandle_t dataMutex     = nullptr;
SemaphoreHandle_t inverterMutex = nullptr;
SemaphoreHandle_t nvsMutex      = nullptr;

volatile bool webUpdateActive                  = false;
volatile unsigned long lastWebUpdateActivityMs = 0;
volatile bool dumpEnabled                      = false;
volatile bool lastPollOk                       = false;
volatile uint16_t lastRespLen                  = 0;

static uint8_t consecutiveFailCount = 0;

static double lastSavedEnergyPv   = -1.0;
static double lastSavedEnergyGrid = -1.0;

AsyncEventSource events("/events");
volatile uint32_t dataVersion = 0;

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
volatile int32_t freePower = 0, currentPower = 0, pvPower = 0, invPower = 0, PpowerHeater = 0;
int32_t acOutVA = 0;
float battI = 0.00f, battP = 0.00f;
float battCapPct = 0.0f, battTempC = 0.0f;
int32_t loadPercent = 0;

volatile bool switch_state = false;
static char targMessageGlobal[128] = "--";

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

// OPT: без String в горячем пути. Выход — статический буфер.
static bool pi30_query(const char* cmd, char* out, size_t outSize,
                       char startChar = '(', char endChar = '\r',
                       uint32_t timeoutMs = 2500) {
  if (!out || outSize == 0) return false;
  out[0] = '\0';

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

  size_t respLen = 0;
  unsigned long t0 = millis();
  bool started = false;
  uint32_t lastByteMs = 0;

  while (millis() - t0 < timeoutMs) {
    if (Inv.available()) {
      char c = Inv.read();
      lastByteMs = millis();
      if (!started) {
        if (c == startChar) {
          if (respLen < outSize - 1) out[respLen++] = c;
          started = true;
        }
      } else {
        if (respLen < outSize - 1) {
          out[respLen++] = c;
        }
        if (c == endChar) break;
      }
    } else if (started && lastByteMs && (millis() - lastByteMs > 600)) {
      break;
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  out[respLen] = '\0';

  if (doLog) {
    char buf[320];
    int p = snprintf(buf, sizeof(buf), ">>> RX[%u]:", (unsigned)respLen);
    for (size_t i = 0; i < respLen && p < (int)sizeof(buf) - 6; i++) {
      uint8_t b = (uint8_t)out[i];
      p += snprintf(buf + p, sizeof(buf) - p, " %02X", b);
    }
    if (respLen > 0 && p < (int)sizeof(buf) - 4) {
      p += snprintf(buf + p, sizeof(buf) - p, "  [");
      for (size_t i = 0; i < respLen && p < (int)sizeof(buf) - 4; i++) {
        char c = out[i];
        buf[p++] = (c >= 32 && c < 127) ? c : '.';
      }
      p += snprintf(buf + p, sizeof(buf) - p, "]\n");
    } else {
      p += snprintf(buf + p, sizeof(buf) - p, "  <EMPTY — no response!>\n");
    }
    serialLog(buf);
    pi30_callCount++;
  }

  return (respLen > 0);
}

// OPT: без String. Работает с сырым буфером.
static bool pi30_parseQPIGS(const char* resp, size_t respLen, InverterData& out) {
  if (!resp || respLen < 2) return false;
  if (resp[0] != '(' || resp[respLen - 1] != '\r') return false;

  // Тело: от resp[1] до resp[respLen-2] (исключая '\r')
  size_t bodyStart = 1;
  size_t bodyEnd   = respLen - 1;   // позиция '\r'
  // trim по краям
  while (bodyStart < bodyEnd && (resp[bodyStart] == ' ' || resp[bodyStart] == '\r' || resp[bodyStart] == '\n')) bodyStart++;
  while (bodyEnd > bodyStart && (resp[bodyEnd - 1] == ' ' || resp[bodyEnd - 1] == '\r' || resp[bodyEnd - 1] == '\n')) bodyEnd--;

  float f[24];
  int count = 0;
  size_t start = bodyStart;
  while (start < bodyEnd && count < 24) {
    while (start < bodyEnd && resp[start] == ' ') start++;
    if (start >= bodyEnd) break;
    size_t end = start;
    while (end < bodyEnd && resp[end] != ' ') end++;

    // atof на подстроке без копирования: временный буфер
    char field[32];
    size_t flen = end - start;
    if (flen > sizeof(field) - 1) flen = sizeof(field) - 1;
    memcpy(field, resp + start, flen);
    field[flen] = '\0';
    f[count++] = atof(field);

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

static int pi30_parseQMOD(const char* resp, size_t respLen) {
  if (!resp || respLen < 2 || resp[0] != '(') return -1;
  switch (resp[1]) {
    case 'P': return 0;  case 'S': return 1;  case 'L': return 2;
    case 'B': return 3;  case 'F': return 4;  case 'H': return 5;
    case 'D': return 6;  default:  return -1;
  }
}

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

void loadSettings() {
  if (nvsMutex) xSemaphoreTake(nvsMutex, portMAX_DELAY);
  preferences.begin("settings", true);
  cfgHeaterPowerVA       = preferences.getUShort("heaterVA", 100);
  cfgMaxAllocatedVA      = preferences.getUShort("maxAllocVA", 100);
  cfgUpdateIntervalSec   = constrain(preferences.getFloat("updIntSec", DEFAULT_UPDATE_INTERVAL_SEC), 0.5f, 10.0f);
  cfgBattOffVoltage      = constrain(preferences.getFloat("battOffV", 27.0f), 20.0f, 80.0f);
  cfgBattHysteresisV     = constrain(preferences.getFloat("battHystV", 0.5f), 0.0f, 10.0f);
  cfgFullChargeCurrentA  = constrain(preferences.getFloat("chgCurA", 0.0f), -100.0f, 100.0f);
  cfgFullChargeVoltage   = constrain(preferences.getFloat("fullChgV", 29.0f), 20.0f, 80.0f);
  cfgBattDischargeAllowW = constrain(preferences.getFloat("battDisW", 0.0f), 0.0f, 15000.0f);
  cfgPumpPostrunMs       = preferences.getUInt("pumpPostMs", 30000);
  cfgPumpMinFreeW        = preferences.getInt("pumpMinW", 0);
  if (cfgPumpPostrunMs  > 600000) cfgPumpPostrunMs  = 600000;
  if (cfgPumpMinFreeW < 0)     cfgPumpMinFreeW = 0;
  if (cfgPumpMinFreeW > 15000) cfgPumpMinFreeW = 15000;
  switch_state           = preferences.getBool("swState", false);
  energyPvWh             = preferences.getDouble("enPvWh",   0.0);
  energyGridWh           = preferences.getDouble("enGridWh", 0.0);
  preferences.end();
  if (nvsMutex) xSemaphoreGive(nvsMutex);

  lastSavedEnergyPv   = energyPvWh;
  lastSavedEnergyGrid = energyGridWh;
}

void saveSettings() {
  if (nvsMutex) xSemaphoreTake(nvsMutex, portMAX_DELAY);
  preferences.begin("settings", false);
  preferences.putUShort("heaterVA",   cfgHeaterPowerVA);
  preferences.putUShort("maxAllocVA", cfgMaxAllocatedVA);
  preferences.putFloat ("updIntSec",  cfgUpdateIntervalSec);
  preferences.putFloat ("battOffV",   cfgBattOffVoltage);
  preferences.putFloat ("battHystV",  cfgBattHysteresisV);
  preferences.putFloat ("chgCurA",    cfgFullChargeCurrentA);
  preferences.putFloat ("fullChgV",   cfgFullChargeVoltage);
  preferences.putFloat ("battDisW",   cfgBattDischargeAllowW);
  preferences.putUInt  ("pumpPostMs", cfgPumpPostrunMs);
  preferences.putInt   ("pumpMinW",   cfgPumpMinFreeW);
  preferences.putBool  ("swState",    switch_state);
  preferences.putDouble("enPvWh",     energyPvWh);
  preferences.putDouble("enGridWh",   energyGridWh);
  preferences.end();
  if (nvsMutex) xSemaphoreGive(nvsMutex);

  lastSavedEnergyPv   = energyPvWh;
  lastSavedEnergyGrid = energyGridWh;
}

void saveEnergy() {
  if (energyPvWh == lastSavedEnergyPv && energyGridWh == lastSavedEnergyGrid) return;

  if (nvsMutex) xSemaphoreTake(nvsMutex, portMAX_DELAY);
  preferences.begin("settings", false);
  preferences.putDouble("enPvWh",   energyPvWh);
  preferences.putDouble("enGridWh", energyGridWh);
  preferences.end();
  if (nvsMutex) xSemaphoreGive(nvsMutex);

  lastSavedEnergyPv   = energyPvWh;
  lastSavedEnergyGrid = energyGridWh;
}

void loadHistoryFromNVS() {
  if (nvsMutex) xSemaphoreTake(nvsMutex, portMAX_DELAY);
  preferences.begin("hist", true);
  histHead  = preferences.getUShort("head", 0);
  histCount = preferences.getUShort("count", 0);
  if (histCount > HIST_BUCKETS) histCount = HIST_BUCKETS;
  if (histHead  >= HIST_BUCKETS) histHead = 0;

  if (histCount > 0) {
    static HistBucket tmp[HIST_BUCKETS];
    size_t total = histCount * sizeof(HistBucket);
    size_t half  = total / 2;
    size_t r1 = preferences.getBytes("b1", tmp, half);
    size_t r2 = preferences.getBytes("b2", (uint8_t*)tmp + half, total - half);
    if (r1 + r2 != total) {
      char b[96];
      snprintf(b, sizeof(b), "!!! HIST load short: r1=%u r2=%u need=%u\n",
               (unsigned)r1, (unsigned)r2, (unsigned)total);
      serialLog(b);
    }
    for (uint16_t k = 0; k < histCount; k++) {
      uint16_t idx = (histHead + HIST_BUCKETS - histCount + k) % HIST_BUCKETS;
      histBuf[idx] = tmp[k];
    }
  }
  preferences.end();
  if (nvsMutex) xSemaphoreGive(nvsMutex);

  char b[96];
  snprintf(b, sizeof(b), "HIST loaded from NVS: count=%u head=%u\n",
           (unsigned)histCount, (unsigned)histHead);
  serialLog(b);
}

void saveHistoryToNVS() {
  if (nvsMutex) xSemaphoreTake(nvsMutex, portMAX_DELAY);
  preferences.begin("hist", false);
  preferences.putUShort("head",  histHead);
  preferences.putUShort("count", histCount);
  if (histCount > 0) {
    static HistBucket tmp[HIST_BUCKETS];
    for (uint16_t k = 0; k < histCount; k++) {
      uint16_t idx = (histHead + HIST_BUCKETS - histCount + k) % HIST_BUCKETS;
      tmp[k] = histBuf[idx];
    }
    size_t total = histCount * sizeof(HistBucket);
    size_t half  = total / 2;
    size_t w1 = preferences.putBytes("b1", tmp, half);
    size_t w2 = preferences.putBytes("b2", (uint8_t*)tmp + half, total - half);
    if (w1 + w2 != total) {
      char b[96];
      snprintf(b, sizeof(b), "!!! HIST save short: w1=%u w2=%u need=%u\n",
               (unsigned)w1, (unsigned)w2, (unsigned)total);
      serialLog(b);
    }
  }
  preferences.end();
  if (nvsMutex) xSemaphoreGive(nvsMutex);
}

void triacTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  uint32_t windowStart = millis();

  uint32_t lastHeaterOnMs = 0;

  uint32_t dutySum    = 0;
  uint32_t dutyTicks  = 0;
  const uint32_t ticksPerWindow = TRIAC_WINDOW_MS / TRIAC_TICK_MS;

  for (;;) {
    uint32_t now = millis();

    bool wantHeat  = switch_state && (currentHeaterPercent > 0);
    uint8_t target = wantHeat ? currentHeaterPercent : 0;

    bool pumpAllowed = switch_state && wantHeat && (freePower >= cfgPumpMinFreeW);

    if (pumpAllowed) {
      if (!pumpRunning) {
        pumpRunning = true;
        digitalWrite(PUMP_PIN, HIGH);
      }
      lastHeaterOnMs = now;
    } else {
      if (pumpRunning && (now - lastHeaterOnMs >= cfgPumpPostrunMs)) {
        pumpRunning = false;
        digitalWrite(PUMP_PIN, LOW);
      }
      if (!switch_state && pumpRunning) {
        pumpRunning = false;
        digitalWrite(PUMP_PIN, LOW);
      }
    }

    // FIX: ТЭН греет ТОЛЬКО пока есть разрешение (мощность + switch).
    if (!pumpAllowed) target = 0;

    if (target == 0) {
      digitalWrite(TRIAC_PIN, LOW);
      currentTriacDuty = 0;
      actualTriacDuty  = 0;
      windowStart = now;
      dutySum = 0;
      dutyTicks = 0;
    } else if (target >= 100) {
      digitalWrite(TRIAC_PIN, HIGH);
      currentTriacDuty = 100;
      actualTriacDuty  = 100;
      windowStart = now;
      dutySum = 0;
      dutyTicks = 0;
    } else {
      uint32_t elapsed = now - windowStart;
      if (elapsed >= TRIAC_WINDOW_MS) {
        windowStart += TRIAC_WINDOW_MS;
        if ((int32_t)(now - windowStart) >= (int32_t)TRIAC_WINDOW_MS) windowStart = now;
        elapsed = now - windowStart;
      }
      uint32_t onTimeMs = (TRIAC_WINDOW_MS * (uint32_t)target) / 100;
      if (elapsed < onTimeMs) {
        digitalWrite(TRIAC_PIN, HIGH);
        currentTriacDuty = target;
      } else {
        digitalWrite(TRIAC_PIN, LOW);
        currentTriacDuty = 0;
      }
      dutySum   += (currentTriacDuty > 0) ? 100UL : 0UL;
      dutyTicks += 1;
      if (dutyTicks >= ticksPerWindow) {
        // OPT: защита от деления на 0 (формально dutyTicks >= 100, но для читаемости)
        if (dutyTicks > 0) actualTriacDuty = (uint8_t)(dutySum / dutyTicks);
        dutySum   = 0;
        dutyTicks = 0;
      }
    }

    vTaskDelayUntil(&lastWakeTime, TRIAC_TICK_MS);
  }
}

static void pushHistoryToClients() {
  if (events.count() == 0) return;

  uint16_t liveCount = (histAccN > 0) ? 1 : 0;
  uint16_t total = histCount + liveCount;

  String out;
  out.reserve(96 + total * 24);
  out += "{\"n\":";
  out += String(total);

  char ebPv[24], ebGrid[24];
  snprintf(ebPv,   sizeof(ebPv),   "%.3f", energyPvWh / 1000.0);
  snprintf(ebGrid, sizeof(ebGrid), "%.3f", energyGridWh / 1000.0);
  out += ",\"enPv\":";
  out += ebPv;
  out += ",\"enGrid\":";
  out += ebGrid;
  out += ",\"s\":[";

  for (uint16_t k = 0; k < histCount; k++) {
    uint16_t idx = (histHead + HIST_BUCKETS - histCount + k) % HIST_BUCKETS;
    if (k) out += ',';
    out += String(histBuf[idx].pv);   out += ',';
    out += String(histBuf[idx].inv);  out += ',';
    out += String(histBuf[idx].batt);
  }

  if (liveCount) {
    if (histCount) out += ',';
    int32_t avgPv   = histAccPv   / (int32_t)histAccN;
    int32_t avgInv  = histAccInv  / (int32_t)histAccN;
    int32_t avgBatt = histAccBatt / (int32_t)histAccN;
    out += String(avgPv);   out += ',';
    out += String(avgInv);  out += ',';
    out += String(avgBatt);
  }

  out += "]}";
  events.send(out.c_str(), "history", millis());
}

static void pushDataToClients() {
  if (events.count() == 0) return;

  DynamicJsonDocument doc(2048);
  bool locked = dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE;
  if (!locked) return;

  doc["ver"]     = dataVersion;
  doc["ws"]      = inv.workMode;
  doc["mt"]      = inv.machineType;
  doc["mp"]      = inv.machinePower;
  doc["fp"]      = (int)freePower;
  doc["cp"]      = (int)(((int32_t)cfgHeaterPowerVA * actualTriacDuty) / 100);
  doc["cpp"]     = (int)currentPower;
  doc["pph"]     = (int)PpowerHeater;
  doc["hpct"]    = actualTriacDuty;
  doc["hpct_t"]  = currentHeaterPercent;
  doc["hva"]     = (unsigned)cfgHeaterPowerVA;
  doc["maVA"]    = (unsigned)cfgMaxAllocatedVA;
  doc["sw"]      = switch_state ? 1 : 0;
  doc["pump"]    = pumpRunning ? 1 : 0;
  doc["tm"]      = targMessageGlobal;
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
  doc["pollOk"]  = lastPollOk ? 1 : 0;
  doc["respLen"] = (unsigned)lastRespLen;
  doc["stale"]   = 0;

  xSemaphoreGive(dataMutex);

  String out;
  out.reserve(measureJson(doc) + 16);
  serializeJson(doc, out);
  events.send(out.c_str(), "data", millis());
}

void pollInverterOnce() {
  digitalWrite(LED_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(20));

  // OPT: статический буфер вместо String
  char respBuf[256];
  digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);
  bool gotResp = pi30_query("QPIGS", respBuf, sizeof(respBuf));
  digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);

  size_t respLen = strlen(respBuf);
  bool ok = gotResp && (respLen >= 60) && pi30_parseQPIGS(respBuf, respLen, inv);

  if (!ok) {
    lastPollOk  = false;
    lastRespLen = (uint16_t)respLen;

    if (consecutiveFailCount < 255) consecutiveFailCount++;
    if (consecutiveFailCount >= 3) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      PpowerHeater = 0;
      currentHeaterPercent = 0;
      freePower = 0;
      currentPower = 0;
      strncpy(targMessageGlobal, "Немає зв'язку з інвертором", sizeof(targMessageGlobal) - 1);
      targMessageGlobal[sizeof(targMessageGlobal) - 1] = '\0';
      xSemaphoreGive(dataMutex);
    }

    char buf[128];
    snprintf(buf, sizeof(buf), "!!! QPIGS parse failed, respLen=%u, fails=%u\n",
             (unsigned)respLen, (unsigned)consecutiveFailCount);
    serialLog(buf);
    digitalWrite(LED_PIN, HIGH);
    dataVersion++;
    pushDataToClients();
    return;
  }

  consecutiveFailCount = 0;

  char respModBuf[32];
  digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);
  bool gotMod = pi30_query("QMOD", respModBuf, sizeof(respModBuf), '(', '\r', 1200);
  int mode = gotMod ? pi30_parseQMOD(respModBuf, strlen(respModBuf)) : -1;
  if (mode >= 0) inv.workMode = mode;
  digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);

  lastPollOk  = true;
  lastRespLen = (uint16_t)respLen;

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  pvV = inv.pvVoltage;
  pvI = inv.pvCurrentA;
  pvPower = inv.pvPowerW;
  if (pvPower <= 0) pvPower = (int32_t)(pvV * pvI);
  if (pvV < 0)  pvV = 0;
  if (pvI < 0)  pvI = 0;
  if (pvPower < 0) pvPower = 0;

  invV        = inv.acOutV;
  invPower    = inv.acOutW;
  acOutVA     = inv.acOutVA;
  invI        = (invV > 0.5f) ? (float)inv.acOutVA / invV : 0.0f;
  loadPercent = inv.loadPercent;
  gridFreq    = inv.gridHz;
  systemV     = (float)inv.busV;

  battV = inv.battVq;
  float chgA   = (float)inv.battChgA;
  float dischA = (float)inv.battDischRaw;
  battI = chgA - dischA;
  battP = battV * battI;
  battCapPct = inv.battCap;
  battTempC  = inv.tempC;

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

  if (currentPower > cfgMaxAllocatedVA) currentPower = cfgMaxAllocatedVA;
  if (currentPower < 0) currentPower = 0;

  PpowerHeater = currentPower;
  if (cfgHeaterPowerVA > 0)
    currentHeaterPercent = constrain((PpowerHeater * 100) / cfgHeaterPowerVA, 0, 100);
  else currentHeaterPercent = 0;

  strncpy(targMessageGlobal, targMessage, sizeof(targMessageGlobal) - 1);
  targMessageGlobal[sizeof(targMessageGlobal) - 1] = '\0';

  char logLine[512];
  snprintf(logLine, sizeof(logLine),
    "[%lu] mode=%d outV=%.1f outVA=%d outW=%d battV=%.2f battI=%.2f "
    "pvV=%.1f pvI=%.1f pvW=%d free=%d cur=%d hP=%d%% act=%d%% pump=%d post=%.1fs minW=%d msg=%s\n",
    millis(), inv.workMode, inv.acOutV, (int)inv.acOutVA, (int)inv.acOutW,
    battV, battI, pvV, pvI, (int)pvPower, (int)freePower, (int)currentPower,
    (int)currentHeaterPercent, (int)actualTriacDuty, pumpRunning ? 1 : 0,
    cfgPumpPostrunMs / 1000.0f, (int)cfgPumpMinFreeW, targMessage);
  serialLog(logLine);

  {
    uint32_t nowMs = millis();
    if (lastEnergyMs == 0) lastEnergyMs = nowMs;
    float dtH = (nowMs - lastEnergyMs) / 3600000.0f;
    lastEnergyMs = nowMs;
    if (dtH > 0.0f && dtH < 0.1f) {
      energyPvWh += (double)pvPower * dtH;
      if (inv.workMode == 2 && invPower > 0)
        energyGridWh += (double)invPower * dtH;
    }
  }

  {
    uint32_t nowMs = millis();
    if (!histStarted) { histWinStart = nowMs; histStarted = true; }

    histAccPv   += (int32_t)pvPower;
    histAccInv  += (int32_t)invPower;
    histAccBatt += (int32_t)battP;
    histAccN++;

    if (histCount == 0 && histAccN == 1) {
      pushHistoryToClients();
      lastHistPushMs = nowMs;
    }

    if (nowMs - histWinStart >= HIST_BUCKET_MS) {
      int32_t avgPv   = histAccPv   / (int32_t)histAccN;
      int32_t avgInv  = histAccInv  / (int32_t)histAccN;
      int32_t avgBatt = histAccBatt / (int32_t)histAccN;

      HistBucket &b = histBuf[histHead];
      b.pv   = (int16_t)constrain(avgPv,   -32768, 32767);
      b.inv  = (int16_t)constrain(avgInv,  -32768, 32767);
      b.batt = (int16_t)constrain(avgBatt, -32768, 32767);
      b.ts   = histWinStart;

      histHead = (histHead + 1) % HIST_BUCKETS;
      if (histCount < HIST_BUCKETS) histCount++;

      histAccPv = histAccInv = histAccBatt = 0;
      histAccN  = 0;
      histWinStart = nowMs;

      if (nowMs - lastHistSaveMs >= HIST_BUCKET_MS) {
        lastHistSaveMs = nowMs;
        saveHistoryToNVS();
      }

      if (nowMs - lastHistPushMs >= HIST_PUSH_EVERY) {
        lastHistPushMs = nowMs;
        pushHistoryToClients();
      }
    }
  }

  xSemaphoreGive(dataMutex);
  digitalWrite(LED_PIN, HIGH);

  dataVersion++;
  pushDataToClients();
}

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
<!DOCTYPE html><html lang="uk"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Inverter Dashboard</title>
<style>
:root{
  --bg:#0e1116; --bg2:#151a21; --panel:#1a2029; --panel2:#20272f;
  --border:#2a3340; --border-hi:#3a4553;
  --text:#e6edf3; --text-dim:#8a97a8; --text-muted:#5b6677;
  --accent:#00e0a1; --accent-dim:#00b080;
  --yellow:#ffc857; --blue:#4ea8ff; --red:#ff5c5c; --purple:#b48cff;
  --shadow:0 8px 24px rgba(0,0,0,.35);
  --shadow-sm:0 2px 8px rgba(0,0,0,.2);
  --radius:14px; --radius-sm:10px;
  --transition:.25s cubic-bezier(.4,0,.2,1);
}
html[data-theme="light"]{
  --bg:#eef2f7; --bg2:#e3e9f0; --panel:#ffffff; --panel2:#f4f7fb;
  --border:#d6dde6; --border-hi:#c0cad6;
  --text:#1a2230; --text-dim:#5a6878; --text-muted:#8a97a8;
  --accent:#00a878; --accent-dim:#008a62;
  --yellow:#e8a317; --blue:#2b7fd4; --red:#e63946; --purple:#8b5cf6;
  --shadow:0 8px 24px rgba(30,50,80,.08);
  --shadow-sm:0 2px 8px rgba(30,50,80,.06);
}
*{box-sizing:border-box}
body{
  background:var(--bg);
  background-image:
    radial-gradient(circle at 15% 0%, color-mix(in srgb, var(--accent) 8%, transparent) 0%, transparent 45%),
    radial-gradient(circle at 85% 100%, color-mix(in srgb, var(--blue) 7%, transparent) 0%, transparent 45%);
  color:var(--text);font-family:'Segoe UI',system-ui,-apple-system,sans-serif;
  padding:16px;margin:0;min-height:100vh;display:flex;flex-direction:column;align-items:center;
  transition:background-color var(--transition),color var(--transition);
}
.dashboard{display:grid;grid-template-columns:1fr 1fr 1.15fr;gap:16px;width:100%;max-width:1100px;align-items:stretch;margin-top:4px}
.col{display:flex;flex-direction:column;gap:16px}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:var(--radius);padding:16px;box-shadow:var(--shadow);display:flex;flex-direction:column;transition:background-color var(--transition),border-color var(--transition),box-shadow var(--transition)}
.panel:hover{border-color:var(--border-hi)}
.panel-header{display:flex;align-items:center;gap:8px;color:var(--text-dim);font-size:11px;font-weight:700;letter-spacing:1.2px;margin-bottom:14px;text-transform:uppercase}
.panel-header svg{width:16px;height:16px;fill:var(--yellow)}
.panel-header.blue svg{fill:var(--blue)}
.panel-header.green svg{fill:var(--accent)}
.grid-2x2{display:grid;grid-template-columns:1fr 1fr;gap:10px;flex:1;grid-auto-rows:1fr}
.metric{text-align:center;background:var(--panel2);border-radius:var(--radius-sm);padding:10px 8px;display:flex;flex-direction:column;justify-content:center;border:1px solid transparent;transition:all var(--transition)}
.metric:hover{border-color:var(--border);transform:translateY(-1px)}
.metric .label{color:var(--text-muted);font-size:9px;margin-bottom:5px;text-transform:uppercase;letter-spacing:.8px;font-weight:600}
.metric .value{font-size:16px;font-weight:700;font-family:'JetBrains Mono',Consolas,monospace}
.yellow{color:var(--yellow)}.blue{color:var(--blue)}.red{color:var(--red)}.green{color:var(--accent)}.purple{color:var(--purple)}
.big-box{background:linear-gradient(135deg,var(--panel2) 0%, color-mix(in srgb, var(--accent) 6%, var(--panel2)) 100%);border-radius:var(--radius-sm);padding:12px;text-align:center;margin-top:10px;flex:1;display:flex;flex-direction:column;justify-content:center;border:1px solid var(--border);position:relative;overflow:hidden}
.big-box::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;background:linear-gradient(90deg,transparent,var(--accent),transparent);opacity:.6}
.big-box.blue::before{background:linear-gradient(90deg,transparent,var(--blue),transparent)}
.big-box.yellow::before{background:linear-gradient(90deg,transparent,var(--yellow),transparent)}
.big-box .label{color:var(--text-dim);font-size:10px;margin-bottom:4px;text-transform:uppercase;letter-spacing:.8px;font-weight:600}
.big-box .value{font-size:22px;font-weight:800;font-family:'JetBrains Mono',Consolas,monospace}
.stack{display:flex;flex-direction:column;gap:10px;flex:1}
.stack .metric{flex:1}
.center-panel{gap:10px;justify-content:space-between}
.current-power-box{background:var(--panel2);border-radius:var(--radius-sm);padding:10px;text-align:center;border:1px solid var(--border);transition:all var(--transition)}
.current-power-box .label{color:var(--text-muted);font-size:10px;margin-bottom:3px;text-transform:uppercase;letter-spacing:.8px;font-weight:600}
.current-power-box .value{font-size:20px;font-weight:800;color:var(--accent);font-family:'JetBrains Mono',Consolas,monospace}

.machine-line{
  display:flex;flex-direction:column;align-items:center;gap:1px;
  margin:-4px 0 6px 0;text-align:center;
}
.machine-line .machine-name{font-size:13px;font-weight:800;color:var(--blue);letter-spacing:.3px}
.machine-line .machine-sub {font-size:10px;font-weight:600;color:var(--text-dim);text-transform:lowercase;letter-spacing:.5px}
.machine-line .machine-mode{font-size:10px;font-weight:600;color:var(--yellow);margin-top:2px}

.label-2line{
  line-height:1.15;
  text-align:center;
}

.ring-row{display:flex;align-items:center;gap:12px;margin-top:4px}
.ring{position:relative;width:96px;height:96px;flex-shrink:0}
.ring svg{transform:rotate(-90deg);width:100%;height:100%}
.ring .bg{fill:none;stroke:var(--border);stroke-width:8}
.ring .fg{fill:none;stroke:var(--accent);stroke-width:8;stroke-linecap:round;transition:stroke-dashoffset .5s ease,stroke .4s}
.ring .center{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center}
.ring .center .pct{font-size:18px;font-weight:800;font-family:'JetBrains Mono',Consolas,monospace;color:var(--text)}
.ring .center .lbl{font-size:8px;color:var(--text-muted);text-transform:uppercase;letter-spacing:.8px;margin-top:1px}
.heater-info{flex:1;min-width:0}
.heater-info .title{font-size:10px;color:var(--text-muted);text-transform:uppercase;letter-spacing:.8px;font-weight:600;margin-bottom:4px}
.heater-info .nominal{font-size:13px;font-weight:700;color:var(--text);font-family:'JetBrains Mono',Consolas,monospace}
.heater-info .sub{font-size:10px;color:var(--text-dim);margin-top:2px}
.pump-box{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:2px;background:var(--panel2);border-radius:var(--radius-sm);padding:8px 12px;border:1px solid var(--border);transition:all var(--transition);min-width:74px;flex-shrink:0}
.pump-box .label{color:var(--text-muted);font-size:9px;text-transform:uppercase;letter-spacing:.8px;font-weight:600}
.pump-box .value{font-size:14px;font-weight:800;color:var(--text-muted);font-family:'JetBrains Mono',Consolas,monospace;transition:all var(--transition)}
.pump-box.on{border-color:var(--accent);background:color-mix(in srgb, var(--accent) 8%, var(--panel2));box-shadow:0 0 0 3px color-mix(in srgb, var(--accent) 12%, transparent)}
.pump-box.on .value{color:var(--accent)}
.pump-box .dot{width:8px;height:8px;border-radius:50%;background:var(--text-muted);transition:all var(--transition)}
.pump-box.on .dot{background:var(--accent);box-shadow:0 0 8px var(--accent);animation:pulse 1.6s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.switch-row{display:flex;align-items:center;justify-content:center;gap:12px;margin-top:4px;padding:8px 14px;background:var(--panel2);border-radius:var(--radius-sm);border:1px solid var(--border);transition:all var(--transition)}
.switch-row.active{border-color:var(--accent);background:color-mix(in srgb, var(--accent) 6%, var(--panel2))}
.toggle-switch{position:relative;width:72px;height:38px;background:var(--border);border-radius:19px;cursor:pointer;transition:all var(--transition);border:2px solid var(--border-hi);flex-shrink:0}
.toggle-switch.active{background:color-mix(in srgb, var(--accent) 20%, var(--border));border-color:var(--accent)}
.toggle-knob{position:absolute;top:3px;left:3px;width:28px;height:28px;background:var(--text-dim);border-radius:50%;transition:all var(--transition);box-shadow:0 2px 6px rgba(0,0,0,.3)}
.toggle-switch.active .toggle-knob{transform:translateX(34px);background:var(--accent);box-shadow:0 0 12px color-mix(in srgb, var(--accent) 60%, transparent)}
.switch-label{color:var(--text-dim);font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.5px}
.alert-box{border:2px solid var(--red);border-radius:var(--radius-sm);padding:12px;background:color-mix(in srgb, var(--red) 6%, transparent);min-height:64px;display:flex;align-items:center;justify-content:center;text-align:center;transition:all var(--transition)}
.alert-box .value{color:var(--red);font-size:12px;font-weight:700;line-height:1.5;word-break:break-word}

.power-pair{display:flex;gap:10px;align-items:stretch}
.batt-row{display:flex;gap:10px;align-items:stretch}
.batt-row .big-box{flex:1;margin-top:10px}
.batt-row .metric{flex:0 0 40%;margin-top:10px}

.power-info-row{display:flex;gap:10px;align-items:stretch}
.power-info-row .metric{flex:1}

.grid-indicator{
  display:flex;flex-direction:column;align-items:center;justify-content:center;gap:3px;
  background:var(--panel2);border-radius:var(--radius-sm);padding:8px 12px;
  border:1px solid var(--text-muted);min-width:72px;flex-shrink:0;
  transition:all var(--transition);
}
.grid-indicator .dot{width:10px;height:10px;border-radius:50%;background:var(--text-muted);box-shadow:0 0 6px var(--text-muted)}
.grid-indicator .lbl{color:var(--text-muted);font-size:10px;font-weight:800;text-transform:uppercase;letter-spacing:.8px}

.grid-indicator.grid-on{border-color:var(--accent);background:color-mix(in srgb, var(--accent) 8%, var(--panel2))}
.grid-indicator.grid-on .dot{background:var(--accent);box-shadow:0 0 8px var(--accent);animation:pulse 1.6s infinite}
.grid-indicator.grid-on .lbl{color:var(--accent)}

.grid-indicator.grid-charge{border-color:var(--red);background:color-mix(in srgb, var(--red) 8%, var(--panel2))}
.grid-indicator.grid-charge .dot{background:var(--red);box-shadow:0 0 8px var(--red);animation:pulse 1.6s infinite}
.grid-indicator.grid-charge .lbl{color:var(--red)}

.controls{margin-top:18px;display:flex;flex-wrap:wrap;gap:10px;justify-content:center;width:100%;max-width:1100px}
.btn{
  color:#fff;border:1px solid transparent;padding:9px 16px;border-radius:var(--radius-sm);
  cursor:pointer;font-size:12px;font-family:inherit;font-weight:700;
  transition:all var(--transition);display:inline-flex;align-items:center;gap:6px;
  box-shadow:var(--shadow-sm);
}
.btn:hover{transform:translateY(-1px);filter:brightness(1.08)}
.btn:active{transform:translateY(0)}
.btn svg{width:14px;height:14px;fill:currentColor;flex-shrink:0}
.btn.green{background:var(--accent);color:#000}
.btn.yellow{background:var(--yellow);color:#000}
.btn.blue{background:var(--blue);color:#000}
.btn.purple{background:var(--purple);color:#000}
.btn.gray{background:var(--panel2);color:var(--text);border-color:var(--border)}
.btn.gray:hover{background:var(--border);color:var(--text)}
.btn.red{background:var(--red);color:#fff}
.update-section{background:var(--panel);padding:18px;border-radius:var(--radius);margin-top:14px;width:100%;max-width:520px;box-sizing:border-box;display:none;box-shadow:var(--shadow);border:1px solid var(--border)}
.update-section.active{display:block;animation:slideDown .3s ease}
@keyframes slideDown{from{opacity:0;transform:translateY(-8px)}to{opacity:1;transform:translateY(0)}}
.update-section b{color:var(--accent);font-size:12px;letter-spacing:.5px}
.update-section input[type=file]{width:100%;padding:10px;background:var(--panel2);color:var(--text);border:1px solid var(--border);border-radius:var(--radius-sm);box-sizing:border-box;font-size:12px;margin-top:8px}
.update-section input[type=file]::file-selector-button{background:var(--accent);color:#000;border:none;padding:6px 12px;border-radius:6px;font-weight:700;cursor:pointer;margin-right:10px;font-family:inherit}
.prog{height:4px;background:var(--border);border-radius:2px;margin-top:10px;overflow:hidden;display:none}
.prog>div{height:100%;width:0;background:var(--accent);transition:width .2s;border-radius:2px}
#ust{font-size:11px;color:var(--text-dim);margin-top:6px;text-align:center}
@media(max-width:900px){.dashboard{grid-template-columns:1fr 1fr}.dashboard .col:nth-child(2){grid-column:1/-1;order:-1}}
@media(max-width:600px){.dashboard{grid-template-columns:1fr}.dashboard .col:nth-child(2){order:0}body{padding:10px}}
</style>
</head><body>

<div class="dashboard">
  <div class="col">
    <div class="panel">
      <div class="panel-header">
        <svg viewBox="0 0 24 24"><path d="M12 7c-2.76 0-5 2.24-5 5s2.24 5 5 5 5-2.24 5-5-2.24-5-5-5zm0 8c-1.66 0-3-1.34-3-3s1.34-3 3-3 3 1.34 3 3-1.34 3-3 3zM12 3v2m0 14v2M3 12h2m14 0h2M5.6 5.6l1.4 1.4m10 10l1.4 1.4M5.6 18.4l1.4-1.4m10-10l1.4-1.4"/></svg>
        Solar Power
      </div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">PV Voltage</div><div class="value yellow" id="pvV">--</div></div>
        <div class="metric"><div class="label">PV Current</div><div class="value yellow" id="pvI">--</div></div>
      </div>
      <div class="big-box yellow"><div class="label">PV Power Now</div><div class="value yellow" id="pvPwr">--</div></div>
    </div>
    <div class="panel">
      <div class="panel-header blue">
        <svg viewBox="0 0 24 24"><path d="M13 3h-2v10h2V3zm4.83 2.17l-1.42 1.42C17.99 7.86 19 9.81 19 12c0 3.87-3.13 7-7 7s-7-3.13-7-7c0-2.19 1.01-4.14 2.58-5.42L6.17 5.17C4.23 6.82 3 9.26 3 12c0 4.97 4.03 9 9 9s9-4.03 9-9c0-2.74-1.23-5.18-3.17-6.83z"/></svg>
        Inverter Power
      </div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">INV Voltage</div><div class="value blue" id="invV">--</div></div>
        <div class="metric"><div class="label">INV Current</div><div class="value blue" id="invI">--</div></div>
      </div>
      <div class="big-box blue"><div class="label">INV Power</div><div class="value blue" id="invVA">--</div></div>
    </div>
  </div>

  <div class="col">
    <div class="panel center-panel">
      <div class="machine-line">
        <span class="machine-name" id="machineInfo">Invertor</span>
        <span class="machine-sub"  id="machinePower">max power</span>
        <span class="machine-mode" id="stateInfo">Mode: --</span>
      </div>

      <div class="power-pair">
        <div class="current-power-box" style="flex:1">
          <div class="label label-2line">FREE<br>POWER</div>
          <div class="value" id="freeP">0 W</div>
        </div>
        <div class="current-power-box" style="flex:1">
          <div class="label">Current Heater Power</div>
          <div class="value" id="curP">0 W</div>
        </div>
      </div>

      <div class="ring-row">
        <div class="ring">
          <svg viewBox="0 0 100 100">
            <circle class="bg" cx="50" cy="50" r="42"/>
            <circle class="fg" id="ringFg" cx="50" cy="50" r="42" stroke-dasharray="263.89" stroke-dashoffset="263.89"/>
          </svg>
          <div class="center">
            <div class="pct" id="ringPct">0%</div>
            <div class="lbl">Heater</div>
          </div>
        </div>
        <div class="heater-info">
          <div class="title">Heater Load</div>
          <div class="nominal" id="hNominal">-- VA</div>
          <div class="sub">Nominal power</div>
        </div>
        <div class="pump-box" id="pumpBox">
          <div class="dot"></div>
          <div class="label">Pump</div>
          <div class="value" id="pumpState">OFF</div>
        </div>
      </div>

      <div class="switch-row" id="switchRow">
        <span class="switch-label">Споживання</span>
        <div class="toggle-switch" id="switchBtn" onclick="toggleSwitch()"><div class="toggle-knob"></div></div>
        <span class="switch-label" id="switchStatus" style="color:var(--text-muted)">ВИМК</span>
      </div>

      <div class="alert-box" id="alertBox"><div class="value" id="targMsg">--</div></div>
    </div>
  </div>

  <div class="col">
    <div class="panel">
      <div class="panel-header blue">
        <svg viewBox="0 0 24 24"><path d="M15.67 4H14V2h-4v2H8.33C7.6 4 7 4.6 7 5.33v15.33C7 21.4 7.6 22 8.33 22h7.33c.74 0 1.34-.6 1.34-1.33V5.33C17 4.6 16.4 4 15.67 4z"/></svg>
        Battery
      </div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">Battery Voltage</div><div class="value blue" id="battV">--</div></div>
        <div class="metric"><div class="label">Battery Current</div><div class="value blue" id="battI">--</div></div>
      </div>
      <div class="batt-row">
        <div class="big-box blue" style="margin-top:10px">
          <div class="label">Battery Power</div>
          <div class="value" id="battPwr">--</div>
        </div>
        <div class="metric" style="margin-top:10px">
          <div class="label">Battery Capacity</div>
          <div class="value green" id="battCap">--</div>
        </div>
      </div>
    </div>
    <div class="panel">
      <div class="panel-header green">
        <svg viewBox="0 0 24 24"><path d="M11 21h-1l1-7H7.5c-.58 0-.57-.32-.38-.66.19-.34.05-.08.07-.12C8.48 10.94 10.42 7.54 13 3h1l-1 7h3.5c.49 0 .56.33.47.51l-.07.15C12.96 17.55 11 21 11 21z"/></svg>
        Power Info
      </div>
      <div class="stack">
        <div class="power-info-row">
          <div class="grid-indicator" id="gridBox">
            <div class="dot"></div>
            <div class="lbl">GRID</div>
          </div>
          <div class="metric" style="flex:1">
            <div class="label">Grid Frequency</div>
            <div class="value blue" id="gridFreq">--</div>
          </div>
          <div class="metric" style="flex:1">
            <div class="label">Load</div>
            <div class="value yellow" id="loadPct">--</div>
          </div>
        </div>
        <div class="metric"><div class="label">Temperature</div><div class="value green" id="tempC">--</div></div>
      </div>
    </div>
  </div>
</div>

<div class="controls">
  <button class="btn green" onclick="update()">
    <svg viewBox="0 0 24 24"><path d="M17.65 6.35A7.958 7.958 0 0012 4a8 8 0 108 8h-2a6 6 0 11-6-6c1.66 0 3.14.69 4.22 1.78L13 11h7V4l-2.35 2.35z"/></svg>
    Refresh
  </button>
  <button class="btn yellow" onclick="location.href='/history'">
    <svg viewBox="0 0 24 24"><path d="M3 13h2v7H3v-7zm4-6h2v13H7V7zm4-4h2v17h-2V3zm4 8h2v9h-2v-9zm4-4h2v13h-2V7z"/></svg>
    Історія
  </button>
  <button class="btn blue" onclick="location.href='/settings'">
    <svg viewBox="0 0 24 24"><path d="M19.14 12.94c.04-.3.06-.61.06-.94 0-.32-.02-.64-.07-.94l2.03-1.58a.49.49 0 00.12-.61l-1.92-3.32a.488.488 0 00-.59-.22l-2.39.96c-.5-.38-1.03-.7-1.62-.94l-.36-2.54a.484.484 0 00-.48-.41h-3.84c-.24 0-.43.17-.47.41l-.36 2.54c-.59.24-1.13.57-1.62.94l-2.39-.96c-.22-.08-.47 0-.59.22L2.74 8.87c-.12.21-.08.47.12.61l2.03 1.58c-.05.3-.09.63-.09.94s.02.64.07.94l-2.03 1.58a.49.49 0 00-.12.61l1.92 3.32c.12.22.37.29.59.22l2.39-.96c.5.38 1.03.7 1.62.94l.36 2.54c.05.24.24.41.48.41h3.84c.24 0 .44-.17.47-.41l.36-2.54c.59-.24 1.13-.56 1.62-.94l2.39.96c.22.08.47 0 .59-.22l1.92-3.32c.12-.22.07-.47-.12-.61l-2.01-1.58zM12 15.6A3.6 3.6 0 1112 8.4a3.6 3.6 0 010 7.2z"/></svg>
    Settings
  </button>
  <button class="btn purple" id="signalBtn" onclick="location.href='/signal'">
    <svg viewBox="0 0 24 24"><path d="M2 22h20V2L2 22zm18-2h-3V9.83l3-3V20z"/></svg>
    Сигнал: --
  </button>
  <button class="btn gray" onclick="location.href='/serial'">
    <svg viewBox="0 0 24 24"><path d="M20 4H4a2 2 0 00-2 2v12a2 2 0 002 2h16a2 2 0 002-2V6a2 2 0 00-2-2zM7.5 17h-2v-2h2v2zm0-4h-2v-2h2v2zm0-4h-2V7h2v2zm9 8h-6v-2h6v2zm0-4h-6v-2h6v2zm0-4h-6V7h6v2z"/></svg>
    Serial
  </button>
  <button class="btn yellow" onclick="resetWifi()">
    <svg viewBox="0 0 24 24"><path d="M1 9l2 2c4.97-4.97 13.03-4.97 18 0l2-2C16.93 2.93 7.08 2.93 1 9zm8 8l3 3 3-3a4.237 4.237 0 00-6 0zm-4-4l2 2a7.074 7.074 0 0110 0l2-2C15.14 9.14 8.87 9.14 5 13z"/></svg>
    Reset WiFi
  </button>
  <button class="btn red" onclick="restartEsp()">
    <svg viewBox="0 0 24 24"><path d="M17.65 6.35A7.958 7.958 0 0012 4a8 8 0 108 8h-2a6 6 0 11-6-6c1.66 0 3.14.69 4.22 1.78L13 11h7V4l-2.35 2.35z"/></svg>
    Restart
  </button>
  <button class="btn blue" id="fwBtn" onclick="toggleUpdate()">
    <svg viewBox="0 0 24 24"><path d="M9 16h6v-6h4l-7-7-7 7h4v6zm-4 2h14v2H5v-2z"/></svg>
    Firmware
  </button>
  <button class="btn gray" onclick="toggleTheme()" id="themeBtn" title="Змінити тему">
    <svg id="themeIcon" viewBox="0 0 24 24"><path d="M12 3a9 9 0 109 9c0-.46-.04-.92-.1-1.36a5.389 5.389 0 01-4.4 2.26 5.403 5.403 0 01-3.14-9.8c-.44-.06-.9-.1-1.36-.1z"/></svg>
    <span id="themeLabel">Dark</span>
  </button>
</div>

<div class="update-section" id="updateSection">
  <b>FIRMWARE UPDATE</b>
  <input type="file" id="fwfile" accept=".bin">
  <button class="btn green" onclick="doUpdate()" style="width:100%;margin-top:10px">Update firmware</button>
  <div class="prog" id="prog"><div id="pbar"></div></div>
  <div id="ust">Select .bin file</div>
</div>

<script>
function setTheme(t){
  document.documentElement.setAttribute('data-theme', t);
  localStorage.setItem('inv_theme', t);
  const lbl = document.getElementById('themeLabel');
  const icon = document.getElementById('themeIcon');
  if (lbl) lbl.textContent = (t === 'light') ? 'Light' : 'Dark';
  if (icon) icon.style.transform = (t === 'light') ? 'rotate(180deg)' : 'rotate(0)';
}
function toggleTheme(){
  const cur = document.documentElement.getAttribute('data-theme');
  setTheme(cur === 'light' ? 'dark' : 'light');
}
(function initTheme(){
  const saved = localStorage.getItem('inv_theme');
  const sys = window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  setTheme(saved || sys);
})();

function wsStr(v){
  switch(v){
    case 0:return"Power On";case 1:return"Standby";case 2:return"Line (Bypass)";
    case 3:return"Battery (Off-Grid)";case 4:return"Fault";case 5:return"Power Save";
    case 6:return"Shutdown";default:return"Unknown";
  }
}
function isBad(v){return v===null||v===undefined||v===65535}
function fmt1(v,u){if(isBad(v))return"--";return Number(v).toFixed(1)+(u?" "+u:"")}
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
  const s=document.getElementById("switchBtn");
  const t=document.getElementById("switchStatus");
  const row=document.getElementById("switchRow");
  if(state){
    s.classList.add("active");row.classList.add("active");
    t.innerText="УВІМК";t.style.color="var(--accent)";
  } else {
    s.classList.remove("active");row.classList.remove("active");
    t.innerText="ВИМК";t.style.color="var(--text-muted)";
  }
}

const RING_R = 42;
const RING_C = 2 * Math.PI * RING_R;
document.getElementById('ringFg').setAttribute('stroke-dasharray', RING_C);

function applyData(d){
  let mt = (d.mt && d.mt !== "UNKNOWN") ? d.mt : "Invertor";
  document.getElementById("machineInfo").innerText  = mt;
  document.getElementById("machinePower").innerText = "max power";
  document.getElementById("stateInfo").innerText    = "Mode: " + wsStr(d.ws);

  document.getElementById("pvV").innerText  = fmt1(d.pvV,"V");
  document.getElementById("pvI").innerText  = fmt1(d.pvI,"A");
  document.getElementById("pvPwr").innerText= fmt0(d.pvPower,"VA");

  document.getElementById("invV").innerText  = fmt1(d.invV,"V");
  document.getElementById("invI").innerText  = fmt1(d.invI,"A");
  document.getElementById("invVA").innerText = fmt0(d.invVA,"VA");

  const hasGrid = (d.freq > 0);
  document.getElementById("gridFreq").innerText = hasGrid ? d.freq.toFixed(1)+" Hz" : "--";
  const battCharging = (d.battP > 0) || (d.battI > 0);
  const gBox = document.getElementById("gridBox");
  if (gBox) {
    gBox.classList.remove("grid-on", "grid-charge");
    if (!hasGrid) {
    } else if (battCharging) {
      gBox.classList.add("grid-charge");
    } else {
      gBox.classList.add("grid-on");
    }
  }

  document.getElementById("loadPct").innerText  = fmt0(d.loadPct,"%");
  document.getElementById("tempC").innerText    = fmt0(d.tempC,"°C");

  document.getElementById("battV").innerText = (d.battV>0)?d.battV.toFixed(2)+" V":"--";
  let bi=document.getElementById("battI");
  bi.innerText = (d.battI!=null?Number(d.battI).toFixed(2):"--")+" A";
  bi.className="value "+(d.battI>=0?"blue":"red");
  let bp=document.getElementById("battPwr");
  bp.innerText=Math.round(d.battP)+" W";
  bp.className="value "+(d.battP>=0?"blue":"red");
  document.getElementById("battCap").innerText = fmt0(d.battCap,"%");

  document.getElementById("curP").innerText  = fmt0(d.cp,"W");
  document.getElementById("freeP").innerText = fmt0(d.fp,"W");
  let hpct = Math.max(0, Math.min(100, d.hpct||0));
  document.getElementById("hNominal").innerText = d.hva ? (d.hva + " VA") : "-- VA";

  const ringFg = document.getElementById('ringFg');
  const ringPct = document.getElementById('ringPct');

  ringFg.style.strokeDashoffset = RING_C * (1 - hpct/100);

  let ringColor = "var(--accent)";
  if (hpct >= 66) ringColor = "var(--red)";
  else if (hpct >= 33) ringColor = "var(--yellow)";
  ringFg.style.stroke = ringColor;

  ringPct.innerText = hpct + "%";
  ringPct.style.color = "var(--text)";
  ringPct.title = "";

  const pBox = document.getElementById("pumpBox");
  const pVal = document.getElementById("pumpState");
  if (d.pump) { pBox.classList.add("on"); pVal.innerText = "ON"; }
  else { pBox.classList.remove("on"); pVal.innerText = "OFF"; }

  updateSwitchUI(d.sw);

  const sigBtn = document.getElementById("signalBtn");
  if(sigBtn) sigBtn.innerText = "Сигнал: " + ((d.rssi!=null && d.rssi!==0) ? d.rssi+" dBm" : "--");

  const tmEl = document.getElementById("targMsg");
  const box  = document.getElementById("alertBox");
  let tm = d.tm || "--";
  tmEl.innerText = tm;
  if (d.pollOk === 0) {
    tmEl.innerText = "Нема зв'язку з інвертором (RX=" + d.respLen + " байт)";
    tmEl.style.color = "var(--red)";
    box.style.borderColor = "var(--red)";
    box.style.background = "color-mix(in srgb, var(--red) 8%, transparent)";
  } else if (d.stale === 1) {
    tmEl.innerText = "Дані застаріли (немає доступу до м'ютексу)";
    tmEl.style.color = "var(--yellow)";
    box.style.borderColor = "var(--yellow)";
    box.style.background = "color-mix(in srgb, var(--yellow) 8%, transparent)";
  } else if (tm === "Все добре") {
    tmEl.style.color = "var(--accent)";
    box.style.borderColor = "var(--accent)";
    box.style.background = "color-mix(in srgb, var(--accent) 8%, transparent)";
  } else {
    tmEl.style.color = "var(--red)";
    box.style.borderColor = "var(--red)";
    box.style.background = "color-mix(in srgb, var(--red) 8%, transparent)";
  }
}

function update(){
  fetch("/api").then(r=>r.json()).then(d=>{lastVer=d.ver;applyData(d);}).catch(()=>{});
}

let lastVer = -1;
fetch("/api").then(r=>r.json()).then(d=>{ lastVer=d.ver; applyData(d); }).catch(console.error);

const es = new EventSource("/events");
es.addEventListener("data", e=>{
  try{
    const d = JSON.parse(e.data);
    if (d.ver === lastVer) return;
    lastVer = d.ver;
    applyData(d);
  }catch(err){ console.error(err); }
});
es.onerror = ()=>{};
</script>
</body></html>
)rawliteral";

// ================== HTML: HISTORY ==================
const char HISTORY_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="uk"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Power History</title>
<style>
:root{
  --bg:#0e1116; --bg2:#151a21; --panel:#1a2029; --panel2:#20272f;
  --border:#2a3340; --border-hi:#3a4553;
  --text:#e6edf3; --text-dim:#8a97a8; --text-muted:#5b6677;
  --accent:#00e0a1; --accent-dim:#00b080;
  --yellow:#ffc857; --blue:#4ea8ff; --red:#ff5c5c;
  --shadow:0 8px 24px rgba(0,0,0,.35);
  --radius:14px; --radius-sm:10px;
  --transition:.25s cubic-bezier(.4,0,.2,1);
}
html[data-theme="light"]{
  --bg:#eef2f7; --bg2:#e3e9f0; --panel:#ffffff; --panel2:#f4f7fb;
  --border:#d6dde6; --border-hi:#c0cad6;
  --text:#1a2230; --text-dim:#5a6878; --text-muted:#8a97a8;
  --accent:#00a878; --accent-dim:#008a62;
  --yellow:#e8a317; --blue:#2b7fd4; --red:#e63946;
  --shadow:0 8px 24px rgba(30,50,80,.08);
}
*{box-sizing:border-box}
body{
  background:var(--bg);
  background-image:
    radial-gradient(circle at 15% 0%, color-mix(in srgb, var(--yellow) 7%, transparent) 0%, transparent 45%),
    radial-gradient(circle at 85% 100%, color-mix(in srgb, var(--blue) 6%, transparent) 0%, transparent 45%);
  color:var(--text);font-family:'Segoe UI',system-ui,-apple-system,sans-serif;
  padding:16px;margin:0;min-height:100vh;display:flex;flex-direction:column;align-items:center;
  transition:background-color var(--transition),color var(--transition);
}
.topbar{display:flex;align-items:center;justify-content:space-between;width:100%;max-width:1100px;margin-bottom:16px;gap:12px;flex-wrap:wrap}
.brand{display:flex;align-items:center;gap:10px;font-weight:700;font-size:16px;letter-spacing:.3px}
.brand svg{width:22px;height:22px;fill:var(--accent)}
.brand span{color:var(--text-dim);font-weight:400;font-size:12px;margin-left:2px}
.theme-toggle{display:flex;align-items:center;gap:8px;background:var(--panel);border:1px solid var(--border);border-radius:999px;padding:6px 12px;cursor:pointer;color:var(--text-dim);font-size:12px;font-weight:600;transition:all var(--transition);box-shadow:0 2px 8px rgba(0,0,0,.15)}
.theme-toggle:hover{border-color:var(--border-hi);color:var(--text)}
.theme-toggle svg{width:16px;height:16px;fill:currentColor;transition:transform .4s}
.theme-toggle:hover svg{transform:rotate(20deg)}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:var(--radius);padding:18px;box-shadow:var(--shadow);width:100%;max-width:1100px;margin-bottom:16px}
.panel-header{display:flex;align-items:center;gap:8px;color:var(--text-dim);font-size:11px;font-weight:700;letter-spacing:1.2px;margin-bottom:14px;text-transform:uppercase;flex-wrap:wrap;justify-content:space-between}
.panel-header .left{display:flex;align-items:center;gap:8px}
.panel-header svg{width:16px;height:16px;fill:var(--yellow)}
.period-tabs{display:flex;gap:6px;background:var(--panel2);padding:4px;border-radius:999px;border:1px solid var(--border)}
.period-tabs button{background:transparent;color:var(--text-dim);border:none;padding:6px 14px;border-radius:999px;cursor:pointer;font-size:11px;font-weight:700;font-family:inherit;transition:all var(--transition);letter-spacing:.3px}
.period-tabs button:hover{color:var(--text)}
.period-tabs button.active{background:var(--accent);color:#000}
.chart-wrap{position:relative;width:100%;height:360px;margin-bottom:8px}
#histChart{display:block;width:100%;height:100%;cursor:crosshair}
#histEmpty{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--text-muted);font-size:13px;pointer-events:none}
.tooltip{position:absolute;pointer-events:none;background:var(--panel2);border:1px solid var(--border-hi);border-radius:8px;padding:8px 10px;font-size:11px;font-family:'JetBrains Mono',Consolas,monospace;color:var(--text);box-shadow:var(--shadow);opacity:0;transition:opacity .15s;z-index:10;min-width:130px}
.tooltip.show{opacity:1}
.tooltip .row{display:flex;justify-content:space-between;gap:10px;margin:2px 0}
.tooltip .row span:first-child{color:var(--text-dim)}
.tooltip .ttl{color:var(--text-dim);font-size:10px;margin-bottom:4px;border-bottom:1px solid var(--border);padding-bottom:4px}
.legend{display:flex;gap:18px;margin-top:10px;font-size:11px;color:var(--text-dim);justify-content:center;flex-wrap:wrap}
.legend span{display:flex;align-items:center;gap:6px}
.legend i{width:14px;height:3px;border-radius:2px;display:inline-block}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;margin-top:6px}
.stat{background:var(--panel2);border-radius:var(--radius-sm);padding:12px;text-align:center;border:1px solid var(--border)}
.stat .label{color:var(--text-muted);font-size:10px;text-transform:uppercase;letter-spacing:.8px;font-weight:600;margin-bottom:4px}
.stat .value{font-size:16px;font-weight:800;font-family:'JetBrains Mono',Consolas,monospace}
.btn{background:var(--panel);color:var(--text);border:1px solid var(--border);padding:9px 16px;border-radius:var(--radius-sm);cursor:pointer;font-size:12px;font-family:inherit;font-weight:600;transition:all var(--transition);display:inline-flex;align-items:center;gap:6px}
.btn:hover{background:var(--panel2);border-color:var(--border-hi);transform:translateY(-1px)}
.btn.primary{background:var(--accent);color:#000;border:none;font-weight:700}
.controls{display:flex;gap:10px;justify-content:center;margin-top:8px;flex-wrap:wrap}
@media(max-width:600px){body{padding:10px}.chart-wrap{height:260px}.stats{grid-template-columns:1fr}.panel-header{flex-direction:column;align-items:flex-start;gap:10px}.period-tabs{width:100%;justify-content:space-between}.period-tabs button{flex:1}}
</style>
</head><body>

<div class="topbar">
  <div class="brand">
    <svg viewBox="0 0 24 24"><path d="M3 13h2v7H3v-7zm4-6h2v13H7V7zm4-4h2v17h-2V3zm4 8h2v9h-2v-9zm4-4h2v13h-2V7z"/></svg>
    Power History <span>PV · INV · BATT</span>
  </div>
  <div class="theme-toggle" onclick="toggleTheme()" title="Змінити тему">
    <svg id="themeIcon" viewBox="0 0 24 24"><path d="M12 3a9 9 0 109 9c0-.46-.04-.92-.1-1.36a5.389 5.389 0 01-4.4 2.26 5.403 5.403 0 01-3.14-9.8c-.44-.06-.9-.1-1.36-.1z"/></svg>
    <span id="themeLabel">Dark</span>
  </div>
</div>

<div class="panel">
  <div class="panel-header">
    <div class="left">
      <svg viewBox="0 0 24 24"><path d="M3 13h2v7H3v-7zm4-6h2v13H7V7zm4-4h2v17h-2V3zm4 8h2v9h-2v-9zm4-4h2v13h-2V7z"/></svg>
      <span>Навантаження за період</span>
    </div>
    <div class="period-tabs">
      <button data-period="1" onclick="setPeriod(1)">24 год</button>
      <button data-period="3" onclick="setPeriod(3)">3 дні</button>
      <button data-period="7" class="active" onclick="setPeriod(7)">7 днів</button>
    </div>
  </div>

  <div class="chart-wrap">
    <canvas id="histChart"></canvas>
    <div id="histEmpty">Збираємо дані…</div>
    <div class="tooltip" id="histTip"></div>
  </div>

  <div class="legend">
    <span><i style="background:var(--yellow)"></i>PV (сонце)</span>
    <span><i style="background:var(--blue)"></i>INV (навантаження / розетка)</span>
    <span><i style="background:var(--accent)"></i>BATT (+ заряд / − розряд)</span>
  </div>

  <div class="stats">
    <div class="stat"><div class="label">AVG PV</div><div class="value yellow" id="statPv">--</div></div>
    <div class="stat"><div class="label">AVG INV</div><div class="value blue" id="statInv">--</div></div>
    <div class="stat"><div class="label">AVG BATT</div><div class="value green" id="statBatt">--</div></div>
    <div class="stat"><div class="label">PV Energy (total)</div><div class="value yellow" id="statEnPv">--</div></div>
    <div class="stat"><div class="label">Grid Energy (total)</div><div class="value blue" id="statEnGrid">--</div></div>
  </div>

  <div class="controls">
    <button class="btn primary" onclick="loadHistory()">
      <svg viewBox="0 0 24 24" style="width:14px;height:14px;fill:currentColor"><path d="M17.65 6.35A7.958 7.958 0 0012 4a8 8 0 108 8h-2a6 6 0 11-6-6c1.66 0 3.14.69 4.22 1.78L13 11h7V4l-2.35 2.35z"/></svg>
      Оновити
    </button>
    <button class="btn" onclick="resetEnergy()" style="background:var(--red);color:#fff;border:none">
      <svg viewBox="0 0 24 24" style="width:14px;height:14px;fill:currentColor"><path d="M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM19 4h-3.5l-1-1h-5l-1 1H5v2h14V4z"/></svg>
      Скинути лічильники
    </button>
    <button class="btn" onclick="location.href='/'">← Дашборд</button>
  </div>
</div>

<script>
function setTheme(t){
  document.documentElement.setAttribute('data-theme', t);
  localStorage.setItem('inv_theme', t);
  const lbl = document.getElementById('themeLabel');
  const icon = document.getElementById('themeIcon');
  if (lbl)  lbl.textContent = (t === 'light') ? 'Light' : 'Dark';
  if (icon) icon.style.transform = (t === 'light') ? 'rotate(180deg)' : 'rotate(0)';
  setTimeout(drawHistory, 60);
}
function toggleTheme(){
  const cur = document.documentElement.getAttribute('data-theme');
  setTheme(cur === 'light' ? 'dark' : 'light');
}
(function initTheme(){
  const saved = localStorage.getItem('inv_theme');
  const sys = window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  setTheme(saved || sys);
})();

const HIST_BUCKET_MS = 30 * 60 * 1000;
const HIST_BUCKETS   = 336;
const PER_DAY        = 48;
let periodDays = 7;
let histData = { n: 0, s: [] };

function setPeriod(d){
  periodDays = d;
  document.querySelectorAll('.period-tabs button').forEach(b=>{
    b.classList.toggle('active', Number(b.dataset.period) === d);
  });
  drawHistory();
}

function renderEnergy(enPv, enGrid){
  const pvEl = document.getElementById('statEnPv');
  const grEl = document.getElementById('statEnGrid');
  if (pvEl) pvEl.textContent = (enPv   != null) ? Number(enPv).toFixed(3)   + ' kWh' : '--';
  if (grEl) grEl.textContent = (enGrid != null) ? Number(enGrid).toFixed(3) + ' kWh' : '--';
}

async function loadHistory(){
  try{
    const r = await fetch('/api_history');
    histData = await r.json();
    renderEnergy(histData.enPv, histData.enGrid);
    drawHistory();
  }catch(e){ console.error('history load', e); }
}

async function resetEnergy(){
  if (!confirm('Скинути лічильники енергії PV та Grid до нуля?')) return;
  try{
    const r = await fetch('/reset_energy', {method:'POST'});
    if (r.status === 200) {
      await loadHistory();
    } else if (r.status === 403) {
      alert('Доступ заборонено (тільки локальна мережа)');
    } else {
      alert('Помилка скидання');
    }
  }catch(e){ alert('Помилка з\'єднання'); }
}

function sliced(){
  const n = histData.n;
  if (!n) return { n: 0, s: [] };
  const need = Math.min(n, periodDays * PER_DAY);
  const start = n - need;
  return {
    n: need,
    s: histData.s.slice(start * 3, (start + need) * 3)
  };
}

function drawHistory(){
  const canvas = document.getElementById('histChart');
  const wrap = canvas.parentElement;
  const dpr = window.devicePixelRatio || 1;
  const W = wrap.clientWidth;
  const H = wrap.clientHeight;
  if (W === 0 || H === 0) return;
  canvas.width  = W * dpr;
  canvas.height = H * dpr;
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,W,H);

  const data = sliced();
  const n = data.n;
  const empty = document.getElementById('histEmpty');
  if (!n) { empty.style.display = 'flex'; return; }
  empty.style.display = 'none';

  const padL = 52, padR = 14, padT = 14, padB = 30;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;

  let minV = 0, maxV = 0;
  for (let k=0;k<n;k++){
    const pv = data.s[k*3], inv = data.s[k*3+1], batt = data.s[k*3+2];
    minV = Math.min(minV, pv, inv, batt);
    maxV = Math.max(maxV, pv, inv, batt);
  }
  const range = Math.max(100, maxV - minV);
  maxV += range * 0.1;
  minV -= range * 0.1;
  if (minV > 0) minV = 0;

  const xStep = plotW / Math.max(1, n - 1);
  const yOf = v => padT + plotH - ((v - minV) / (maxV - minV)) * plotH;
  const xOf = i => padL + i * xStep;

  const csBorder = getComputedStyle(document.documentElement).getPropertyValue('--border').trim() || '#2a3340';
  const csMuted  = getComputedStyle(document.documentElement).getPropertyValue('--text-muted').trim() || '#8a97a8';
  ctx.strokeStyle = csBorder;
  ctx.lineWidth = 1;
  ctx.font = '10px Consolas,monospace';
  ctx.fillStyle = csMuted;

  const gridLines = 5;
  for (let g=0; g<=gridLines; g++){
    const v = minV + (maxV-minV)*g/gridLines;
    const y = yOf(v);
    ctx.beginPath();
    ctx.moveTo(padL, y); ctx.lineTo(padL+plotW, y);
    ctx.stroke();
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    ctx.fillText(Math.round(v)+' W', padL-6, y);
  }

  for (let d=0; d<=periodDays; d++){
    const i = d * PER_DAY;
    if (i > n) break;
    const x = xOf(i - 0.5);
    ctx.strokeStyle = 'rgba(128,128,128,0.15)';
    ctx.beginPath();
    ctx.moveTo(x, padT); ctx.lineTo(x, padT+plotH);
    ctx.stroke();
    if (d < periodDays){
      const cx = xOf(i - 0.5 + PER_DAY/2);
      ctx.fillStyle = csMuted;
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      ctx.fillText('-'+(periodDays-d)+'d', cx, padT+plotH+8);
    }
  }

  if (minV < 0 && maxV > 0){
    const y0 = yOf(0);
    ctx.strokeStyle = 'rgba(200,200,200,0.45)';
    ctx.beginPath(); ctx.moveTo(padL, y0); ctx.lineTo(padL+plotW, y0); ctx.stroke();
  }

  if (n === 1) {
    const cx = padL + plotW / 2;
    const cy1 = yOf(data.s[0]);
    const cy2 = yOf(data.s[1]);
    const cy3 = yOf(data.s[2]);
    ctx.fillStyle = '#ffc857';
    ctx.beginPath(); ctx.arc(cx, cy1, 4, 0, 2*Math.PI); ctx.fill();
    ctx.fillStyle = '#4ea8ff';
    ctx.beginPath(); ctx.arc(cx, cy2, 4, 0, 2*Math.PI); ctx.fill();
    ctx.fillStyle = '#00e0a1';
    ctx.beginPath(); ctx.arc(cx, cy3, 4, 0, 2*Math.PI); ctx.fill();

    document.getElementById('statPv').textContent   = data.s[0] + ' W';
    document.getElementById('statInv').textContent  = data.s[1] + ' W';
    document.getElementById('statBatt').textContent = data.s[2] + ' W';

    canvas._data = { data, minV, maxV, padL, padR, padT, padB, plotW, plotH, n };
    return;
  }

  const drawSeries = (color, idx) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    let started = false;
    for (let k=0;k<n;k++){
      const v = data.s[k*3 + idx];
      const x = xOf(k);
      const y = yOf(v);
      if (!started){ ctx.moveTo(x,y); started = true; }
      else ctx.lineTo(x,y);
    }
    ctx.stroke();
  };
  drawSeries('#ffc857', 0);
  drawSeries('#4ea8ff', 1);
  drawSeries('#00e0a1', 2);

  let sp=0, si=0, sb=0;
  for (let k=0;k<n;k++){ sp+=data.s[k*3]; si+=data.s[k*3+1]; sb+=data.s[k*3+2]; }
  document.getElementById('statPv').textContent   = Math.round(sp/n) + ' W';
  document.getElementById('statInv').textContent  = Math.round(si/n) + ' W';
  document.getElementById('statBatt').textContent = Math.round(sb/n) + ' W';

  canvas._data = { data, minV, maxV, padL, padR, padT, padB, plotW, plotH, n };
}

(function setupTooltip(){
  const canvas = document.getElementById('histChart');
  const tip = document.getElementById('histTip');

  canvas.addEventListener('mousemove', e => {
    const st = canvas._data;
    if (!st || !st.n) { tip.classList.remove('show'); return; }
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const { padL, plotW, n, data } = st;
    if (x < padL || x > padL + plotW) { tip.classList.remove('show'); return; }

    let idx;
    if (n === 1) idx = 0;
    else idx = Math.round((x - padL) / (plotW / Math.max(1, n-1)));
    if (idx < 0 || idx >= n) { tip.classList.remove('show'); return; }

    const pv   = data.s[idx*3];
    const inv  = data.s[idx*3+1];
    const batt = data.s[idx*3+2];
    const bucketAgo = (n - 1 - idx) * (HIST_BUCKET_MS / 60000);
    const hAgo = Math.floor(bucketAgo / 60);
    const mAgo = Math.round(bucketAgo % 60);
    const timeLbl = (n === 1) ? 'зараз' : (hAgo ? `-${hAgo} год ${mAgo} хв` : `-${mAgo} хв`);

    tip.innerHTML =
      `<div class="ttl">${timeLbl}</div>` +
      `<div class="row"><span>PV</span><span style="color:#ffc857">${pv} W</span></div>` +
      `<div class="row"><span>INV</span><span style="color:#4ea8ff">${inv} W</span></div>` +
      `<div class="row"><span>BATT</span><span style="color:#00e0a1">${batt} W</span></div>`;

    const tipW = 150, tipH = 80;
    let tx = x + 14, ty = y - 30;
    if (tx + tipW > rect.width)  tx = x - tipW - 14;
    if (ty < 0) ty = 6;
    if (ty + tipH > rect.height) ty = rect.height - tipH - 6;
    tip.style.left = tx + 'px';
    tip.style.top  = ty + 'px';
    tip.classList.add('show');
  });
  canvas.addEventListener('mouseleave', () => tip.classList.remove('show'));
})();

let resizeTO = null;
window.addEventListener('resize', () => {
  clearTimeout(resizeTO);
  resizeTO = setTimeout(drawHistory, 200);
});

const es = new EventSource('/events');
es.addEventListener('history', e => {
  try{
    histData = JSON.parse(e.data);
    renderEnergy(histData.enPv, histData.enGrid);
    drawHistory();
  }catch(err){ console.error(err); }
});

loadHistory();
</script>
</body></html>
)rawliteral";

// ================== HTML: SERIAL ==================
const char SERIAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Serial Monitor</title>
<style>
:root{
  --bg:#0f0f0f; --panel:#1c1c1c; --border:#333;
  --text:#e6e6e6; --text-dim:#888;
  --accent:#00ff88; --accent-dim:#00cc6a;
  --btn-bg:#555; --btn-text:#fff;
}
html[data-theme="light"]{
  --bg:#eef2f7; --panel:#ffffff; --border:#d6dde6;
  --text:#1a2230; --text-dim:#5a6878;
  --accent:#00a878; --accent-dim:#008a62;
  --btn-bg:#c0cad6; --btn-text:#1a2230;
}
body{background:var(--bg);color:var(--text);font-family:monospace;padding:15px;margin:0}
h1{color:var(--accent);text-align:center;font-size:18px}
#log{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:10px;height:70vh;overflow-y:auto;font-size:12px;line-height:1.6;white-space:pre-wrap;word-break:break-all}
.btn{background:var(--accent);color:#000;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-weight:bold;margin:3px}
.bg{background:var(--btn-bg);color:var(--btn-text)}
.ctr{text-align:center;margin:10px 0}
</style></head><body>
<h1>Serial Monitor</h1>
<div class="ctr">
<button class="btn" onclick="fetchLog()">Refresh</button>
<button class="btn bg" onclick="document.getElementById('log').innerText=''">Clear</button>
<button class="btn bg" id="autoBtn" onclick="toggleAuto()">Auto-scroll: ON</button>
<button class="btn bg" id="dumpBtn" onclick="toggleDump()">Dump: OFF</button>
<button class="btn bg" onclick="toggleTheme()" id="themeBtn">🌓 Тема</button>
<button class="btn bg" onclick="location.href='/'">Back</button>
</div>
<div id="log">Loading...</div>
<script>
function setTheme(t){
  document.documentElement.setAttribute('data-theme', t);
  localStorage.setItem('inv_theme', t);
}
function toggleTheme(){
  const cur = document.documentElement.getAttribute('data-theme');
  setTheme(cur === 'light' ? 'dark' : 'light');
}
(function initTheme(){
  const saved = localStorage.getItem('inv_theme');
  const sys = window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  setTheme(saved || sys);
})();

let autoScroll=true, dumpOn=false;
function toggleAuto(){autoScroll=!autoScroll;document.getElementById("autoBtn").innerText=autoScroll?"Auto-scroll: ON":"Auto-scroll: OFF";}
function updDump(){let b=document.getElementById("dumpBtn");b.innerText=dumpOn?"Dump: ON":"Dump: OFF";b.style.background=dumpOn?"var(--accent)":"var(--btn-bg)";b.style.color=dumpOn?"#000":"var(--btn-text)";}
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
:root{
  --bg:#0f0f0f; --panel:#1c1c1c; --panel2:#141414; --border:#333; --border-hi:#4a4a4a;
  --text:#e6e6e6; --text-dim:#9aa0a6;
  --accent:#00ff88; --accent-dim:#00cc6a;
  --input-bg:#333; --input-border:#555;
  --blue:#4da3ff; --yellow:#ffc857; --red:#ff5c5c; --purple:#b48cff;
  --btn-bg:#555; --btn-text:#fff;
}
html[data-theme="light"]{
  --bg:#eef2f7; --panel:#ffffff; --panel2:#f4f7fb; --border:#d6dde6; --border-hi:#b8c3d1;
  --text:#1a2230; --text-dim:#5a6878;
  --accent:#00a878; --accent-dim:#008a62;
  --input-bg:#f0f3f8; --input-border:#c0cad6;
  --blue:#2b7fd4; --yellow:#e8a317; --red:#e63946; --purple:#8b5cf6;
  --btn-bg:#c0cad6; --btn-text:#1a2230;
}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,-apple-system,sans-serif;padding:16px;margin:0;min-height:100vh}
.wrap{max-width:760px;margin:0 auto}
.topbar{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:18px;flex-wrap:wrap}
.topbar h1{margin:0;color:var(--accent);font-size:20px;display:flex;align-items:center;gap:10px}
.topbar h1 svg{width:22px;height:22px;fill:var(--accent)}
.topbar .actions{display:flex;gap:8px;flex-wrap:wrap}

.section{background:var(--panel);border:1px solid var(--border);border-radius:14px;margin-bottom:16px;overflow:hidden;box-shadow:0 4px 12px rgba(0,0,0,.15)}
.section-head{display:flex;align-items:center;gap:10px;padding:14px 18px;background:var(--panel2);border-bottom:1px solid var(--border);cursor:pointer;user-select:none;transition:background .2s}
.section-head:hover{background:color-mix(in srgb, var(--accent) 4%, var(--panel2))}
.section-head svg.icon{width:20px;height:20px;flex-shrink:0}
.section-head .title{font-weight:700;font-size:14px;letter-spacing:.3px;flex:1}
.section-head .subtitle{font-size:11px;color:var(--text-dim);font-weight:400;margin-left:8px}
.section-head .chev{width:16px;height:16px;fill:var(--text-dim);transition:transform .25s;flex-shrink:0}
.section.collapsed .section-head .chev{transform:rotate(-90deg)}
.section.collapsed .section-body{display:none}
.section-body{padding:6px 18px 18px 18px}
.section.heater .section-head svg.icon{fill:var(--red)}
.section.battery-protect .section-head svg.icon{fill:var(--yellow)}
.section.battery-charge .section-head svg.icon{fill:var(--blue)}
.section.pump .section-head svg.icon{fill:var(--purple)}

.field{margin-top:16px}
.field:first-child{margin-top:10px}
.field label{display:flex;align-items:baseline;gap:8px;font-size:13px;font-weight:700;color:var(--text);margin-bottom:4px}
.field label .unit{color:var(--text-dim);font-weight:400;font-size:11px;font-family:'JetBrains Mono',Consolas,monospace}
.field .desc{color:var(--text-dim);font-size:12px;line-height:1.55;margin:0 0 8px 0;padding:8px 12px;background:var(--panel2);border-left:3px solid var(--border-hi);border-radius:4px}
.field .desc b{color:var(--text);font-weight:600}
.field .desc code{background:var(--bg);padding:1px 5px;border-radius:3px;font-family:'JetBrains Mono',Consolas,monospace;font-size:11px;color:var(--accent)}
.field input{width:100%;padding:11px 12px;background:var(--input-bg);color:var(--text);border:1px solid var(--input-border);border-radius:8px;font-size:14px;font-family:'JetBrains Mono',Consolas,monospace;transition:border-color .2s,box-shadow .2s}
.field input:focus{outline:none;border-color:var(--accent);box-shadow:0 0 0 3px color-mix(in srgb, var(--accent) 20%, transparent)}
.field input.invalid{border-color:var(--red);box-shadow:0 0 0 3px color-mix(in srgb, var(--red) 20%, transparent)}

.btn{background:var(--accent);color:#000;border:none;padding:10px 18px;border-radius:8px;cursor:pointer;font-weight:700;font-family:inherit;font-size:13px;transition:all .2s;display:inline-flex;align-items:center;gap:6px}
.btn:hover{transform:translateY(-1px);filter:brightness(1.08)}
.btn:active{transform:translateY(0)}
.btn.bg{background:var(--btn-bg);color:var(--btn-text)}
.btn svg{width:14px;height:14px;fill:currentColor}

.msg{color:var(--accent);text-align:center;margin:12px 0;padding:10px;background:color-mix(in srgb, var(--accent) 10%, transparent);border:1px solid var(--accent);border-radius:8px;display:none;font-weight:600}
.err{color:var(--red);text-align:center;margin:12px 0;padding:10px;background:color-mix(in srgb, var(--red) 10%, transparent);border:1px solid var(--red);border-radius:8px;display:none;font-weight:600}

.footer-actions{display:flex;gap:10px;justify-content:center;margin-top:20px;flex-wrap:wrap}
@media(max-width:600px){body{padding:10px}.section-head{padding:12px 14px}.section-body{padding:6px 14px 14px 14px}}
</style></head><body>
<div class="wrap">

<div class="topbar">
  <h1>
    <svg viewBox="0 0 24 24"><path d="M19.14 12.94c.04-.3.06-.61.06-.94 0-.32-.02-.64-.07-.94l2.03-1.58a.49.49 0 00.12-.61l-1.92-3.32a.488.488 0 00-.59-.22l-2.39.96c-.5-.38-1.03-.7-1.62-.94l-.36-2.54a.484.484 0 00-.48-.41h-3.84c-.24 0-.43.17-.47.41l-.36 2.54c-.59.24-1.13.57-1.62.94l-2.39-.96c-.22-.08-.47 0-.59.22L2.74 8.87c-.12.21-.08.47.12.61l2.03 1.58c-.05.3-.09.63-.09.94s.02.64.07.94l-2.03 1.58a.49.49 0 00-.12.61l1.92 3.32c.12.22.37.29.59.22l2.39-.96c.5.38 1.03.7 1.62.94l.36 2.54c.05.24.24.41.48.41h3.84c.24 0 .44-.17.47-.41l.36-2.54c.59-.24 1.13-.56 1.62-.94l2.39.96c.22.08.47 0 .59-.22l1.92-3.32c.12-.22.07-.47-.12-.61l-2.01-1.58zM12 15.6A3.6 3.6 0 1112 8.4a3.6 3.6 0 010 7.2z"/></svg>
    Налаштування
  </h1>
  <div class="actions">
    <button class="btn bg" onclick="toggleAll()" id="toggleAllBtn" title="Розгорнути/згорнути всі блоки">Згорнути все</button>
    <button class="btn bg" onclick="toggleTheme()" id="themeBtn">🌓 Тема</button>
  </div>
</div>

<div class="msg" id="msg">✓ Збережено!</div>
<div class="err" id="err">✗ Доступ заборонено (тільки локальна мережа)</div>

<div class="section heater" data-key="heater">
  <div class="section-head" onclick="toggleSection(this)">
    <svg class="icon" viewBox="0 0 24 24"><path d="M13 3h-2v10h2V3zm4.83 2.17l-1.42 1.42C17.99 7.86 19 9.81 19 12c0 3.87-3.13 7-7 7s-7-3.13-7-7c0-2.19 1.01-4.14 2.58-5.42L6.17 5.17C4.23 6.82 3 9.26 3 12c0 4.97 4.03 9 9 9s9-4.03 9-9c0-2.74-1.23-5.18-3.17-6.83z"/></svg>
    <span class="title">ТЕН (нагрівач)</span>
    <span class="subtitle">потужність та ліміти</span>
    <svg class="chev" viewBox="0 0 24 24"><path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z"/></svg>
  </div>
  <div class="section-body">
    <div class="field">
      <label>Номінальна потужність ТЕНа <span class="unit">VA</span></label>
      <div class="desc">
        Повна потужність вашого ТЕНа. Від неї рахується <b>відсоток навантаження</b>.<br>
        <b>Приклад:</b> ТЕН 2000 Вт → введіть <code>2000</code>.
      </div>
      <input type="number" id="heaterVA" min="100" max="15000" step="50" value="1000">
    </div>
    <div class="field">
      <label>Максимальна дозволена потужність <span class="unit">VA</span></label>
      <div class="desc">
        Стеля, вище якої ТЕН <b>ніколи</b> не підніметься.
      </div>
      <input type="number" id="maxAllocVA" min="100" max="15000" step="50" value="500">
    </div>
    <div class="field">
      <label>Інтервал опитування інвертора <span class="unit">сек</span></label>
      <div class="desc">
        Як часто ESP32 надсилає запит <code>QPIGS</code>. Рекомендовано <code>1.0</code>–<code>3.0</code> сек.
      </div>
      <input type="number" id="updIntSec" min="0.5" max="10" step="0.1" value="2">
    </div>
  </div>
</div>

<div class="section battery-protect" data-key="battery-protect">
  <div class="section-head" onclick="toggleSection(this)">
    <svg class="icon" viewBox="0 0 24 24"><path d="M12 2L4 5v6.09c0 5.05 3.41 9.76 8 10.91 4.59-1.15 8-5.86 8-10.91V5l-8-3zm-1 13l-3-3 1.41-1.41L11 12.17l4.59-4.58L17 9l-6 6z"/></svg>
    <span class="title">Захист акумулятора</span>
    <span class="subtitle">відключення по напрузі</span>
    <svg class="chev" viewBox="0 0 24 24"><path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z"/></svg>
  </div>
  <div class="section-body">
    <div class="field">
      <label>Напруга відключення <span class="unit">В</span></label>
      <div class="desc">
        Якщо напруга АКБ падає <b>нижче</b> цього значення — ТЕН і насос вимикаються.<br>
        <b>Для 24В:</b> <code>27.0</code> В.
      </div>
      <input type="number" id="battOffV" min="20" max="80" step="0.1" value="27">
    </div>
    <div class="field">
      <label>Гістерезис <span class="unit">В</span></label>
      <div class="desc">
        Різниця напруги для повторного включення.
      </div>
      <input type="number" id="battHystV" min="0" max="5" step="0.1" value="0.5">
    </div>
  </div>
</div>

<div class="section battery-charge" data-key="battery-charge">
  <div class="section-head" onclick="toggleSection(this)">
    <svg class="icon" viewBox="0 0 24 24"><path d="M15.67 4H14V2h-4v2H8.33C7.6 4 7 4.6 7 5.33v15.33C7 21.4 7.6 22 8.33 22h7.33c.74 0 1.34-.6 1.34-1.33V5.33C17 4.6 16.4 4 15.67 4zM11 20v-5.5H9L13 7v5.5h2L11 20z"/></svg>
    <span class="title">Заряджання акумулятора</span>
    <span class="subtitle">пріоритет заряду над ТЕНом</span>
    <svg class="chev" viewBox="0 0 24 24"><path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z"/></svg>
  </div>
  <div class="section-body">
    <div class="field">
      <label>Напруга повного заряду <span class="unit">В</span></label>
      <div class="desc">
        Коли АКБ досягає цього значення — весь надлишок сонця йде на ТЕН.<br>
        <b>Для 24В AGM:</b> <code>29.0</code> В.
      </div>
      <input type="number" id="fullChgV" min="20" max="80" step="0.1" value="29">
    </div>
    <div class="field">
      <label>Резерв струму зарядки <span class="unit">А</span></label>
      <div class="desc">
        Скільки ампер зарезервувати для АКБ. Додатнє — резерв, від'ємне — дозволити брати з АКБ.
      </div>
      <input type="number" id="chgCurA" min="-100" max="100" step="1" value="0">
    </div>
    <div class="field">
      <label>Дозвіл розряду АКБ <span class="unit">Вт</span></label>
      <div class="desc">
        Скільки ват дозволено брати з АКБ на ТЕН.
      </div>
      <input type="number" id="battDisW" min="0" max="15000" step="50" value="0">
    </div>
  </div>
</div>

<div class="section pump" data-key="pump">
  <div class="section-head" onclick="toggleSection(this)">
    <svg class="icon" viewBox="0 0 24 24"><path d="M12 2c-5.33 4.55-8 8.48-8 11.8 0 4.98 3.8 8.2 8 8.2s8-3.22 8-8.2c0-3.32-2.67-7.25-8-11.8zm0 18c-3.35 0-6-2.57-6-6.2 0-2.34 1.95-5.44 6-9.14 4.05 3.7 6 6.79 6 9.14 0 3.63-2.65 6.2-6 6.2z"/></svg>
    <span class="title">Насос котла</span>
    <span class="subtitle">умови запуску та таймінги</span>
    <svg class="chev" viewBox="0 0 24 24"><path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z"/></svg>
  </div>
  <div class="section-body">
    <div class="field">
      <label>Мінімальна вільна потужність для насоса <span class="unit">Вт</span></label>
      <div class="desc">
        Насос увімкнеться <b>тільки якщо</b> вільна потужність не менша за це значення.<br>
        <b>0</b> — без порогу. Рекомендовано 50–200 Вт.
      </div>
      <input type="number" id="pumpMinW" min="0" max="15000" step="10" value="0">
    </div>
    <div class="field">
      <label>Довибіг насоса (post-run) <span class="unit">сек</span></label>
      <div class="desc">
        Скільки секунд насос працює після вимкнення ТЕНа.<br>
        <b>Рекомендовано:</b> <code>30</code>–<code>60</code> сек.
      </div>
      <input type="number" id="pumpPostSec" min="0" max="600" step="1" value="30">
    </div>
  </div>
</div>

<div class="footer-actions">
  <button class="btn" onclick="save()">
    <svg viewBox="0 0 24 24"><path d="M17 3H5a2 2 0 00-2 2v14a2 2 0 002 2h14a2 2 0 002-2V7l-4-4zm-5 16a3 3 0 110-6 3 3 0 010 6zm3-10H5V5h10v4z"/></svg>
    Зберегти
  </button>
  <button class="btn bg" onclick="location.href='/'">
    <svg viewBox="0 0 24 24"><path d="M20 11H7.83l5.59-5.59L12 4l-8 8 8 8 1.41-1.41L7.83 13H20v-2z"/></svg>
    Назад
  </button>
</div>

</div>
<script>
function setTheme(t){
  document.documentElement.setAttribute('data-theme', t);
  localStorage.setItem('inv_theme', t);
  const btn = document.getElementById('themeBtn');
  if (btn) btn.textContent = (t === 'light') ? '☀️ Світла' : '🌙 Темна';
}
function toggleTheme(){
  const cur = document.documentElement.getAttribute('data-theme');
  setTheme(cur === 'light' ? 'dark' : 'light');
}
(function initTheme(){
  const saved = localStorage.getItem('inv_theme');
  const sys = window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  setTheme(saved || sys);
})();

function toggleSection(head){
  const sec = head.parentElement;
  sec.classList.toggle('collapsed');
  saveCollapsedState();
}
function saveCollapsedState(){
  const state = {};
  document.querySelectorAll('.section').forEach(s=>{
    if (s.classList.contains('collapsed')) state[s.dataset.key] = 1;
  });
  localStorage.setItem('inv_settings_collapsed', JSON.stringify(state));
}
function loadCollapsedState(){
  try{
    const state = JSON.parse(localStorage.getItem('inv_settings_collapsed') || '{}');
    document.querySelectorAll('.section').forEach(s=>{
      if (state[s.dataset.key]) s.classList.add('collapsed');
    });
  }catch(e){}
}
function toggleAll(){
  const anyOpen = [...document.querySelectorAll('.section')].some(s=>!s.classList.contains('collapsed'));
  document.querySelectorAll('.section').forEach(s=>{
    s.classList.toggle('collapsed', anyOpen);
  });
  document.getElementById('toggleAllBtn').textContent = anyOpen ? 'Розгорнути все' : 'Згорнути все';
  saveCollapsedState();
}
loadCollapsedState();

const KEYS = ["heaterVA","maxAllocVA","updIntSec","battOffV","battHystV","chgCurA","fullChgV","battDisW","pumpMinW","pumpPostSec"];

async function load(){
  try{
    let d = await (await fetch("/api_settings")).json();
    KEYS.forEach(k=>{
      const el = document.getElementById(k);
      if (el && d[k] !== undefined) el.value = d[k];
    });
  }catch(e){ console.error('load settings', e); }
}

async function save(){
  const body = KEYS.map(k=>k+"="+encodeURIComponent(document.getElementById(k).value)).join("&");
  try{
    let r = await fetch("/save_settings",{
      method:"POST",
      headers:{"Content-Type":"application/x-www-form-urlencoded"},
      body
    });
    if(r.status==200){
      const m = document.getElementById("msg");
      m.style.display="block";
      document.getElementById("err").style.display="none";
      setTimeout(()=>m.style.display="none", 2500);
    } else if(r.status==403){
      const e = document.getElementById("err");
      e.style.display="block";
      document.getElementById("msg").style.display="none";
      setTimeout(()=>e.style.display="none", 3500);
    } else {
      alert("Помилка збереження (HTTP " + r.status + ")");
    }
  }catch(e){
    alert("Помилка з'єднання з пристроєм");
  }
}

load();
</script></body></html>
)rawliteral";

// ================== HTML: SIGNAL ==================
const char SIGNAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Рівень сигналу</title>
<style>
:root{
  --bg:#0f0f0f; --panel:#1c1c1c; --border:#2a2a2a;
  --text:#e6e6e6; --text-dim:#888;
  --accent:#00ff88; --bar-bg:#333;
  --btn-bg:#555; --btn-text:#fff;
}
html[data-theme="light"]{
  --bg:#eef2f7; --panel:#ffffff; --border:#d6dde6;
  --text:#1a2230; --text-dim:#5a6878;
  --accent:#00a878; --bar-bg:#e0e6ee;
  --btn-bg:#c0cad6; --btn-text:#1a2230;
}
body{background:var(--bg);color:var(--text);font-family:monospace;padding:15px;margin:0}
h1{color:var(--accent);text-align:center;font-size:18px}
.card{background:var(--panel);border-radius:10px;padding:20px;max-width:420px;margin:12px auto;border:1px solid var(--border)}
.big{text-align:center;font-size:36px;font-weight:bold;margin:8px 0}
.small{text-align:center;color:var(--text-dim);font-size:12px;margin-bottom:14px}
.bar{height:14px;background:var(--bar-bg);border-radius:7px;overflow:hidden;margin:10px 0 18px}
.fill{height:100%;width:0;background:var(--accent);transition:.4s}
.row{display:flex;justify-content:space-between;padding:7px 0;border-bottom:1px solid var(--border);font-size:12px}
.row:last-child{border-bottom:none}
.k{color:var(--text-dim)}.v{font-weight:bold;text-align:right;word-break:break-all}
.btn{background:var(--accent);color:#000;border:none;padding:10px 18px;border-radius:6px;cursor:pointer;font-weight:bold;margin:5px}
.bg{background:var(--btn-bg);color:var(--btn-text)}
.center{text-align:center;margin-top:14px}
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
<div class="center">
<button class="btn bg" onclick="toggleTheme()" id="themeBtn">🌓 Тема</button>
<button class="btn bg" onclick="location.href='/'">Назад</button>
</div>
<script>
function setTheme(t){
  document.documentElement.setAttribute('data-theme', t);
  localStorage.setItem('inv_theme', t);
}
function toggleTheme(){
  const cur = document.documentElement.getAttribute('data-theme');
  setTheme(cur === 'light' ? 'dark' : 'light');
}
(function initTheme(){
  const saved = localStorage.getItem('inv_theme');
  const sys = window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  setTheme(saved || sys);
})();

function esc(s){return String(s||'--').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}
async function upd(){
  try{
    let d=await(await fetch('/api_signal')).json();
    let q=d.quality||0;
    document.getElementById('q').innerText=q+'%';
    document.getElementById('rssi').innerText='RSSI: '+((d.rssi!=null&&d.rssi!==0)?d.rssi+' dBm':'--');
    let f=document.getElementById('fill');
    f.style.width=q+'%';
    f.style.background=q>70?'var(--accent)':(q>40?'#ffd166':'#ff4d4d');
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
<style>
:root{
  --bg:#0f0f0f; --panel:#1c1c1c; --border:#555;
  --text:#fff; --text-dim:#888;
  --accent:#00ff88; --input-bg:#333;
}
html[data-theme="light"]{
  --bg:#eef2f7; --panel:#ffffff; --border:#c0cad6;
  --text:#1a2230; --text-dim:#5a6878;
  --accent:#00a878; --input-bg:#f0f3f8;
}
body{background:var(--bg);color:var(--text);font-family:monospace;padding:20px;text-align:center;margin:0}
.card{background:var(--panel);border-radius:12px;padding:20px;max-width:360px;margin:0 auto;border:1px solid var(--border)}
select,input,button{width:100%;box-sizing:border-box;padding:12px;margin:8px 0;background:var(--input-bg);color:var(--text);border:1px solid var(--border);border-radius:8px;font-size:14px}
button{background:var(--accent);color:#000;font-weight:bold;cursor:pointer;border:none}
.theme-link{background:transparent;color:var(--text-dim);font-size:12px;padding:6px;border:none;cursor:pointer;text-decoration:underline;width:auto;margin:0 auto;display:inline-block}
</style></head><body>
<div class="card">
<h2>WiFi Setup</h2>
<form action="/save_wifi" method="POST">
<select name="ssid" id="ss" required><option>Scanning...</option></select>
<input type="password" name="password" placeholder="Password">
<button type="submit">Save</button>
</form>
<button class="theme-link" onclick="toggleTheme()" id="themeBtn">🌓 Змінити тему</button>
</div>
<script>
function setTheme(t){
  document.documentElement.setAttribute('data-theme', t);
  localStorage.setItem('inv_theme', t);
}
function toggleTheme(){
  const cur = document.documentElement.getAttribute('data-theme');
  setTheme(cur === 'light' ? 'dark' : 'light');
}
(function initTheme(){
  const saved = localStorage.getItem('inv_theme');
  const sys = window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  setTheme(saved || sys);
})();

fetch("/scan_wifi").then(r=>r.json()).then(n=>{let s=document.getElementById("ss");s.innerHTML="";
if(!n.length){s.innerHTML="<option value=''>No networks</option>";return}
n.forEach(x=>{let o=document.createElement("option");o.value=x.ssid;o.text=x.ssid+" ("+x.rssi+")";s.appendChild(o)})});
</script></body></html>
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
  events.onConnect([](AsyncEventSourceClient *client){
    if (client->lastId()) {
      char b[48];
      snprintf(b, sizeof(b), "SSE reconnect, lastId=%u\n", client->lastId());
      serialLog(b);
    }
    client->send("hello", NULL, millis(), 1000);
  });
  server.addHandler(&events);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) { r->send_P(200, "text/html", INDEX_HTML); });
  server.on("/history", HTTP_GET, [](AsyncWebServerRequest *r) { r->send_P(200, "text/html", HISTORY_HTML); });

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
    doc["cp"]  = (int)(((int32_t)cfgHeaterPowerVA * actualTriacDuty) / 100);
    doc["cpp"] = (int)currentPower;
    doc["pph"] = (int)PpowerHeater;
    doc["hpct"]= actualTriacDuty;
    doc["hpct_t"] = currentHeaterPercent;
    doc["hva"] = (unsigned)cfgHeaterPowerVA;
    doc["maVA"]= (unsigned)cfgMaxAllocatedVA;
    doc["sw"]  = switch_state ? 1 : 0;
    doc["pump"]= pumpRunning ? 1 : 0;
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
    doc["ver"]     = dataVersion;
    doc["ver_sketch"] = SKETCH_VERSION;
    doc["stale"]   = locked ? 0 : 1;

    JsonArray raw = doc.createNestedArray("raw");
    for (int i = 0; i < inv.f_count; i++) raw.add(inv.f_raw[i]);

    auto* res = r->beginResponseStream("application/json; charset=utf-8");
    serializeJson(doc, *res);
    r->send(res);
    if (locked) xSemaphoreGive(dataMutex);
  });

  server.on("/api_history", HTTP_GET, [](AsyncWebServerRequest *r) {
    uint16_t liveCount = (histAccN > 0) ? 1 : 0;
    uint16_t total = histCount + liveCount;

    auto* res = r->beginResponseStream("application/json; charset=utf-8");
    res->printf("{\"n\":%u,\"enPv\":%.3f,\"enGrid\":%.3f,\"s\":[",
                (unsigned)total, energyPvWh / 1000.0, energyGridWh / 1000.0);

    for (uint16_t k = 0; k < histCount; k++) {
      uint16_t idx = (histHead + HIST_BUCKETS - histCount + k) % HIST_BUCKETS;
      if (k) res->print(',');
      res->printf("%d,%d,%d", histBuf[idx].pv, histBuf[idx].inv, histBuf[idx].batt);
    }
    if (liveCount) {
      if (histCount) res->print(',');
      int32_t avgPv   = histAccPv   / (int32_t)histAccN;
      int32_t avgInv  = histAccInv  / (int32_t)histAccN;
      int32_t avgBatt = histAccBatt / (int32_t)histAccN;
      res->printf("%d,%d,%d", (int)avgPv, (int)avgInv, (int)avgBatt);
    }

    res->print("]}");
    r->send(res);
  });

  server.on("/reset_energy", HTTP_POST, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        energyPvWh   = 0.0;
        energyGridWh = 0.0;
        lastEnergyMs = millis();
        xSemaphoreGive(dataMutex);
      }
      lastSavedEnergyPv   = -1.0;
      lastSavedEnergyGrid = -1.0;
      saveEnergy();
      req->send(200, "text/plain", "OK");
    });
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
    doc["pumpPostSec"]= cfgPumpPostrunMs  / 1000.0f;
    doc["pumpMinW"]   = cfgPumpMinFreeW;
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
      uint32_t newPumpPost = cfgPumpPostrunMs;
      int32_t  newPumpMinW = cfgPumpMinFreeW;

      if (req->hasParam("heaterVA",   true)) newHeaterVA  = constrain(req->getParam("heaterVA",   true)->value().toInt(),   100,    15000);
      if (req->hasParam("maxAllocVA", true)) newMaxAlloc  = constrain(req->getParam("maxAllocVA", true)->value().toInt(),   100,    15000);
      if (req->hasParam("updIntSec",  true)) newUpdInt    = constrain(req->getParam("updIntSec",  true)->value().toFloat(), 0.5f,   10.0f);
      if (req->hasParam("battOffV",   true)) newBattOffV  = constrain(req->getParam("battOffV",   true)->value().toFloat(), 20.0f,  80.0f);
      if (req->hasParam("battHystV",  true)) newBattHystV = constrain(req->getParam("battHystV",  true)->value().toFloat(), 0.0f,   10.0f);
      if (req->hasParam("chgCurA",    true)) newChgCurA   = constrain(req->getParam("chgCurA",    true)->value().toFloat(), -100.0f, 100.0f);
      if (req->hasParam("fullChgV",   true)) newFullChgV  = constrain(req->getParam("fullChgV",   true)->value().toFloat(), 20.0f,  80.0f);
      if (req->hasParam("battDisW",   true)) newBattDisW  = constrain(req->getParam("battDisW",   true)->value().toFloat(), 0.0f,   15000.0f);
      if (req->hasParam("pumpPostSec",true)) newPumpPost  = (uint32_t)constrain(req->getParam("pumpPostSec", true)->value().toFloat() * 1000.0f, 0.0f, 600000.0f);
      if (req->hasParam("pumpMinW",   true)) newPumpMinW  = constrain(req->getParam("pumpMinW",   true)->value().toInt(),   0,     15000);

      if (dataMutex && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        cfgHeaterPowerVA       = newHeaterVA;
        cfgMaxAllocatedVA      = newMaxAlloc;
        cfgUpdateIntervalSec   = newUpdInt;
        cfgBattOffVoltage      = newBattOffV;
        cfgBattHysteresisV     = newBattHystV;
        cfgFullChargeCurrentA  = newChgCurA;
        cfgFullChargeVoltage   = newFullChgV;
        cfgBattDischargeAllowW = newBattDisW;
        cfgPumpPostrunMs       = newPumpPost;
        cfgPumpMinFreeW        = newPumpMinW;
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
      dataVersion++;
      pushDataToClients();
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

void setup() {
  Serial.begin(115200);

  pinMode(TRIAC_PIN, OUTPUT);
  digitalWrite(TRIAC_PIN, LOW);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);

  Inv.begin(INVERTER_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  vTaskDelay(pdMS_TO_TICKS(300));

  pi30_crcSelfTest();

  WiFi.persistent(false);

  nvsMutex = xSemaphoreCreateMutex();

  preferences.begin("wifi_config", true);
  wifiSsid     = preferences.getString("ssid", "");
  wifiPassword = preferences.getString("password", "");
  preferences.end();
  loadSettings();
  loadHistoryFromNVS();

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

  serialLog("=== System started (PI30 QPIGS ECO II, v8.6.9 SSE + PUMP(power-gated, no-pre-start) + ACTUAL TRIAC(smooth) + HISTORY(NVS split) + ENERGY + THEME + RAW-BUF) ===\n");
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

  static uint32_t lastEnergySaveMs = 0;
  if (now - lastEnergySaveMs > ENERGY_SAVE_INTERVAL_MS) {
    lastEnergySaveMs = now;
    saveEnergy();
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}
