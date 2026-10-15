#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <Update.h>
#include <functional>
#include <ArduinoJson.h>

#define RX2_PIN 16
#define TX2_PIN 17
#define RS485_CTRL 4
#define LED_PIN 2
#define SKETCH_VERSION "7.2.5"
#define SERIAL_BUF_SIZE 4096

// ========================================================================
// КЕРУВАННЯ СІМІСТОРОМ — BURST CONTROL (MOC3063 + BTA41 + активне навантаження)
// ========================================================================
#define TRIAC_PIN 19

constexpr uint32_t TRIAC_WINDOW_MS = 1000;
constexpr uint32_t TRIAC_TICK_MS   = 10;

volatile uint8_t currentTriacDuty     = 0;
volatile uint8_t currentHeaterPercent = 0;

TaskHandle_t triacTaskHandle = nullptr;
TaskHandle_t modbusTaskHandle = nullptr;

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
constexpr uint32_t RESTART_DELAY_MS = 2000;
constexpr uint32_t WEB_UPDATE_TIMEOUT_MS = 30000;
constexpr float DEFAULT_UPDATE_INTERVAL_SEC = 2.0f;

SemaphoreHandle_t dataMutex = nullptr;
SemaphoreHandle_t modbusMutex = nullptr;
volatile bool webUpdateActive = false;
volatile unsigned long lastWebUpdateActivityMs = 0;
volatile bool pendingCounterReset = false;
volatile bool dumpEnabled = false;

static uint16_t invBuf[80], pvBuf[20];

uint16_t cfgHeaterPowerVA = 100;
uint16_t cfgMaxAllocatedVA = 100;
float cfgUpdateIntervalSec = DEFAULT_UPDATE_INTERVAL_SEC;
float cfgBattOffVoltage = 27.00f;
float cfgBattHysteresisV = 0.5f;
float cfgFullChargeCurrentA = 0.00f;
float cfgFullChargeVoltage = 29.00f;
float cfgBattDischargeAllowW = 0.00f;

float battV = 0.00f;
float pvV = 0.00f;
float pvI = 0.00f;
float invV = 0.00f;
float invI = 0.00f;
float gridFreq = 0.00f;
float systemV = 0.00f;

int32_t freePower = 0;
int32_t currentPower = 0;
int32_t pvPower = 0;
int32_t invPower = 0;
int32_t PpowerHeater = 0;
float battI = 0.00f;
float battP = 0.00f;

volatile bool switch_state = false;
static char targMessageGlobal[128] = "--";

static char serialBuf[SERIAL_BUF_SIZE];
static volatile uint16_t serialHead = 0;
static volatile uint16_t serialTail = 0;
static portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

// ========================================================================
// КОНФІГУРАЦІЯ ТИПІВ ІНВЕРТОРІВ
// ========================================================================
struct RegAddr {
  uint16_t addr;
  int16_t divisor;
  int16_t maxPorog;
  int16_t divisortwo;
};

struct InverterConfig {
  const char* id;
  const char* name;

  uint16_t invBlockAddr;
  uint8_t  invBlockLen;
  uint16_t pvBlockAddr;
  uint8_t  pvBlockLen;

  RegAddr pvWork;
  RegAddr mppt;
  RegAddr chg;
  RegAddr pvV;
  RegAddr pvI;
  RegAddr pvPower;
  RegAddr systemV;
  RegAddr pvEnergyHi;
  RegAddr pvEnergyLo;

  RegAddr invWorkState;
  RegAddr invV_0;
  RegAddr invV_1;
  RegAddr battV;
  RegAddr invI;
  RegAddr invPower;
  RegAddr gridFreq;
  RegAddr acChgHi;
  RegAddr acChgLo;

  RegAddr battI;
  RegAddr battP;

  uint16_t resetCounter1Addr;
  uint16_t resetCounter2Addr;
  uint16_t machineTypeAddr;
  uint16_t machinePowerAddr;
  uint8_t  slaveId;
  uint32_t baudRate;
};

const InverterConfig inverterConfigs[] = {
  {
    .id   = "SRNE",
    .name = "Must (Standard)/ SRNE / EASUN",
    .invBlockAddr = 25201,
    .invBlockLen = 80,
    .pvBlockAddr  = 15201,
    .pvBlockLen  = 18,
    .pvWork       = {15201, 1},
    .mppt = {15202, 1},
    .chg = {15203, 1},
    .pvV          = {15206, 10},
    .pvI = {15207, 10},
    .pvPower = {15208, 1},
    .systemV      = {15215, 1},
    .pvEnergyHi = {15217, 1},
    .pvEnergyLo = {15218, 1},
    .invWorkState = {25201, 1},
    .invV_0       = {25206, 10},
    .invV_1 = {25202, 1},
    .battV = {25205, 10},
    .invI = {25212, 10},
    .invPower = {25219, 1},
    .gridFreq     = {25225, 100, 0 , 0},
    .acChgHi      = {25255, 1},
    .acChgLo = {25256, 1},
    .battI  = {25274, -1, 50, -10},
    .battP    = {25273, -1},
    .resetCounter1Addr = 20213,
    .resetCounter2Addr = 10112,
    .machineTypeAddr   = 20000,
    .machinePowerAddr  = 20001,
    .slaveId = 0x04,
    .baudRate = 19200
  },
  {
    .id = "GROWATT",
    .name = "Growatt (Заготовка)",
    .invBlockAddr = 25201,
    .invBlockLen = 80,
    .pvBlockAddr  = 15201,
    .pvBlockLen  = 18,
    .pvWork       = {15201, 1},
    .mppt = {15202, 1},
    .chg = {15203, 1},
    .pvV          = {15206, 10},
    .pvI = {15207, 10},
    .pvPower = {15208, 1},
    .systemV      = {15215, 1},
    .pvEnergyHi = {15217, 1},
    .pvEnergyLo = {15218, 1},
    .invWorkState = {25201, 1},
    .invV_0       = {25206, 10},
    .invV_1 = {25202, 1},
    .battV = {25205, 10},
    .invI = {25212, 10},
    .invPower = {25219, 1},
    .gridFreq     = {25225, 100, 0 , 0},
    .acChgHi      = {25255, 1},
    .acChgLo = {25256, 1},
    .battI  = {25274, -1, 50, -10},
    .battP    = {25273, -1},
    .resetCounter1Addr = 20213,
    .resetCounter2Addr = 10112,
    .machineTypeAddr   = 20000,
    .machinePowerAddr  = 20001,
    .slaveId = 0x04,
    .baudRate = 19200
  },
  {
    .id = "VOLTRONIC",
    .name = "Voltronic / Axpert (Заготовка)",
    .invBlockAddr = 25201,
    .invBlockLen = 80,
    .pvBlockAddr  = 15201,
    .pvBlockLen  = 18,
    .pvWork       = {15201, 1},
    .mppt = {15202, 1},
    .chg = {15203, 1},
    .pvV          = {15206, 10},
    .pvI = {15207, 10},
    .pvPower = {15208, 1},
    .systemV      = {15215, 1},
    .pvEnergyHi = {15217, 1},
    .pvEnergyLo = {15218, 1},
    .invWorkState = {25201, 1},
    .invV_0       = {25206, 10},
    .invV_1 = {25202, 1},
    .battV = {25205, 10},
    .invI = {25212, 10},
    .invPower = {25219, 1},
    .gridFreq     = {25225, 100, 0 , 0},
    .acChgHi      = {25255, 1},
    .acChgLo = {25256, 1},
    .battI  = {25274, -1, 50, -10},
    .battP    = {25273, -1},
    .resetCounter1Addr = 20213,
    .resetCounter2Addr = 10112,
    .machineTypeAddr   = 20000,
    .machinePowerAddr  = 20001,
    .slaveId = 0x04,
    .baudRate = 19200
  }
};
const int NUM_INVERTER_MODELS = sizeof(inverterConfigs) / sizeof(inverterConfigs[0]);

int cfgInverterModelIdx = 0;

// ========================================================================
// RAW MODBUS RTU
// ========================================================================
#define MB_SUCCESS           0x00
#define MB_INVALID_SLAVE_ID  0xE0
#define MB_INVALID_FUNCTION  0xE1
#define MB_RESPONSE_TIMEOUT  0xE2
#define MB_INVALID_CRC       0xE3

static uint16_t modbusCRC16(const uint8_t* buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void mbFlushRx() {
  while (Serial1.available()) Serial1.read();
}

static uint8_t modbusReadHolding(uint16_t addr, uint8_t qty,
                                 uint16_t* outRegs,
                                 uint32_t timeoutMs = 1500) {
  if (qty < 1 || qty > 125) return MB_INVALID_FUNCTION;

  const InverterConfig& invCfg = inverterConfigs[cfgInverterModelIdx];
  uint8_t slave = invCfg.slaveId;

  uint8_t frame[8];
  frame[0] = slave;
  frame[1] = 0x03;
  frame[2] = (addr >> 8) & 0xFF;
  frame[3] = addr & 0xFF;
  frame[4] = 0;
  frame[5] = qty;
  uint16_t crc = modbusCRC16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  mbFlushRx();

  digitalWrite(RS485_CTRL, HIGH);
  delayMicroseconds(100);
  Serial1.write(frame, 8);
  Serial1.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_CTRL, LOW);

  uint8_t rx[260];
  uint16_t rxLen = 0;
  uint16_t expected = 5;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (Serial1.available() && rxLen < sizeof(rx)) {
      rx[rxLen++] = Serial1.read();
      if (rxLen >= 3 && rx[1] == 0x03) expected = 3 + rx[2] + 2;
      if (rxLen >= 5 && rx[1] == 0x83) expected = 5;
    }
    if (rxLen >= expected) break;
    delay(1);
  }

  if (rxLen == 0)    return MB_RESPONSE_TIMEOUT;
  if (rxLen < 5)     return MB_RESPONSE_TIMEOUT;

  uint16_t calc = modbusCRC16(rx, rxLen - 2);
  uint16_t recv = rx[rxLen - 2] | ((uint16_t)rx[rxLen - 1] << 8);
  if (calc != recv)  return MB_INVALID_CRC;

  if (rx[1] == 0x83) return MB_INVALID_FUNCTION;
  if (rx[0] != slave || rx[1] != 0x03) return MB_INVALID_SLAVE_ID;

  uint8_t byteCount = rx[2];
  if (rxLen < (uint16_t)(3 + byteCount + 2)) return MB_RESPONSE_TIMEOUT;
  if ((byteCount / 2) < qty) return MB_RESPONSE_TIMEOUT;

  for (uint16_t i = 0; i < qty; i++) {
    outRegs[i] = ((uint16_t)rx[3 + 2 * i] << 8) | rx[3 + 2 * i + 1];
  }
  return MB_SUCCESS;
}

static uint8_t modbusWriteSingle(uint16_t addr, uint16_t value,
                                 uint8_t* excCode = nullptr,
                                 uint32_t timeoutMs = 1500) {
  if (excCode) *excCode = 0;
  const InverterConfig& invCfg = inverterConfigs[cfgInverterModelIdx];
  uint8_t slave = invCfg.slaveId;

  uint8_t frame[8];
  frame[0] = slave;
  frame[1] = 0x06;
  frame[2] = (addr >> 8) & 0xFF;
  frame[3] = addr & 0xFF;
  frame[4] = (value >> 8) & 0xFF;
  frame[5] = value & 0xFF;
  uint16_t crc = modbusCRC16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  mbFlushRx();

  digitalWrite(RS485_CTRL, HIGH);
  delayMicroseconds(100);
  Serial1.write(frame, 8);
  Serial1.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_CTRL, LOW);

  uint8_t rx[16];
  uint16_t rxLen = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (Serial1.available() && rxLen < sizeof(rx)) rx[rxLen++] = Serial1.read();
    if (rxLen >= 8 || (rxLen >= 5 && (rx[1] & 0x80))) break;
    delay(1);
  }

  if (rxLen < 5) return MB_RESPONSE_TIMEOUT;

  uint16_t calc = modbusCRC16(rx, rxLen - 2);
  uint16_t recv = rx[rxLen - 2] | ((uint16_t)rx[rxLen - 1] << 8);
  if (calc != recv) return MB_INVALID_CRC;

  if (rx[1] & 0x80) {
    if (excCode) *excCode = rx[2];
    return MB_INVALID_FUNCTION;
  }

  if (rxLen < 8) return MB_RESPONSE_TIMEOUT;
  if (rx[0] != slave) return MB_INVALID_SLAVE_ID;
  return MB_SUCCESS;
}

static uint8_t modbusWriteMultiple(uint16_t addr, const uint16_t* values, uint8_t qty,
                                   uint8_t* excCode = nullptr,
                                   uint32_t timeoutMs = 1500) {
  if (excCode) *excCode = 0;
  if (qty < 1 || qty > 123) return MB_INVALID_FUNCTION;

  const InverterConfig& invCfg = inverterConfigs[cfgInverterModelIdx];
  uint8_t slave = invCfg.slaveId;

  uint8_t frame[260];
  frame[0] = slave;
  frame[1] = 0x10;
  frame[2] = (addr >> 8) & 0xFF;
  frame[3] = addr & 0xFF;
  frame[4] = 0;
  frame[5] = qty;
  frame[6] = qty * 2;
  for (uint8_t i = 0; i < qty; i++) {
    frame[7 + 2 * i] = (values[i] >> 8) & 0xFF;
    frame[8 + 2 * i] = values[i] & 0xFF;
  }
  uint16_t len = 7 + qty * 2;
  uint16_t crc = modbusCRC16(frame, len);
  frame[len]     = crc & 0xFF;
  frame[len + 1] = crc >> 8;
  len += 2;

  mbFlushRx();

  digitalWrite(RS485_CTRL, HIGH);
  delayMicroseconds(100);
  Serial1.write(frame, len);
  Serial1.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_CTRL, LOW);

  uint8_t rx[16];
  uint16_t rxLen = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (Serial1.available() && rxLen < sizeof(rx)) rx[rxLen++] = Serial1.read();
    if (rxLen >= 8 || (rxLen >= 5 && (rx[1] & 0x80))) break;
    delay(1);
  }

  if (rxLen < 5) return MB_RESPONSE_TIMEOUT;

  uint16_t calc = modbusCRC16(rx, rxLen - 2);
  uint16_t recv = rx[rxLen - 2] | ((uint16_t)rx[rxLen - 1] << 8);
  if (calc != recv) return MB_INVALID_CRC;

  if (rx[1] & 0x80) {
    if (excCode) *excCode = rx[2];
    return MB_INVALID_FUNCTION;
  }

  if (rxLen < 8) return MB_RESPONSE_TIMEOUT;
  if (rx[0] != slave) return MB_INVALID_SLAVE_ID;
  return MB_SUCCESS;
}

// ========================================================================
// ДОПОМІЖНІ ФУНКЦІЇ
// ========================================================================
inline int regIndex(uint16_t blockAddr, uint8_t blockLen, uint16_t regAddr) {
  if (regAddr < blockAddr) return -1;
  int idx = (int)(regAddr - blockAddr);
  if (idx >= blockLen) return -1;
  return idx;
}

inline uint16_t getReg(const uint16_t* buf, uint16_t blockAddr, uint8_t blockLen, uint16_t regAddr) {
  int idx = regIndex(blockAddr, blockLen, regAddr);
  if (idx < 0) return 0xFFFF;
  return buf[idx];
}

inline int32_t safeDivisor(int16_t d) {
  return (d == 0) ? 1 : (int32_t)d;
}

inline int32_t absInt16(int16_t v) {
  int32_t x = (int32_t)v;
  return (x < 0) ? -x : x;
}

float readScaled(const uint16_t* buf, uint16_t blockAddr, uint8_t blockLen, const RegAddr& r) {
  if (r.addr == 0) return 0.0f;
  uint16_t rawU = getReg(buf, blockAddr, blockLen, r.addr);
  if (rawU == 0xFFFF) return 0.0f;

  int16_t raw    = (int16_t)rawU;
  int32_t absVal = absInt16(raw);
  int32_t mp     = absInt16(r.maxPorog);

  int16_t divt = (mp > 0 && absVal > mp) ? r.divisortwo : r.divisor;

  float v = (float)raw / (float)safeDivisor(divt);
  if (v == 0.0f) v = 0.0f;
  return v;
}

int32_t readRaw(const uint16_t* buf, uint16_t blockAddr, uint8_t blockLen, const RegAddr& r) {
  if (r.addr == 0) return 0;
  uint16_t raw = getReg(buf, blockAddr, blockLen, r.addr);
  if (raw == 0xFFFF) return 0;
  return (int32_t)raw;
}

int32_t readScaledInt(const uint16_t* buf, uint16_t blockAddr, uint8_t blockLen, const RegAddr& r) {
  if (r.addr == 0) return 0;
  uint16_t rawU = getReg(buf, blockAddr, blockLen, r.addr);
  if (rawU == 0xFFFF) return 0;

  int16_t raw    = (int16_t)rawU;
  int32_t absVal = absInt16(raw);
  int32_t mp     = absInt16(r.maxPorog);
  int16_t divt   = (mp > 0 && absVal > mp) ? r.divisortwo : r.divisor;

  return (int32_t)raw / (int32_t)safeDivisor(divt);
}

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

bool isLocalIP(IPAddress ip) {
  if (ip[0] == 10) return true;
  if (ip[0] == 172 && ip[1] >= 16 && ip[1] <= 31) return true;
  if (ip[0] == 192 && ip[1] == 168) return true;
  IPAddress localIP = WiFi.localIP();
  IPAddress softIP = WiFi.softAPIP();
  if (localIP[0] != 0 && ip == localIP) return true;
  if (softIP[0] != 0 && ip == softIP) return true;
  return false;
}

void requireLocalIP(AsyncWebServerRequest *request, std::function<void(AsyncWebServerRequest*)> handler) {
  if (!request->client() || !isLocalIP(request->client()->remoteIP())) {
    request->send(403, "text/plain", "Access denied: Local network only");
    return;
  }
  handler(request);
}

// ========================================================================
// InverterData
// ========================================================================
struct InverterData {
  int workState = 0;
  String machineType, machinePower;

  uint16_t pvWork = 0;
  uint16_t mppt = 0;
  uint16_t chg = 0;
  uint16_t pvEnergyHi = 0;
  uint16_t pvEnergyLo = 0;
  uint16_t acChgHi = 0;
  uint16_t acChgLo = 0;
};

AsyncWebServer server(80);
DNSServer dnsServer;
Preferences preferences;
InverterData inv;
String wifiSsid, wifiPassword;
bool setupMode = false;
unsigned long lastWifiReconnectMs = 0;
bool needRestart = false;
unsigned long restartRequestedMs = 0;

template<typename T>
bool readModbusBlock(uint16_t addr, uint8_t qty, T* buf) {
  if (addr == 0 || qty == 0) return false;
  if (qty > 125) return false;
  uint16_t tmp[125];
  if (modbusReadHolding(addr, qty, tmp) != MB_SUCCESS) return false;
  for (uint8_t i = 0; i < qty; i++) buf[i] = (T)tmp[i];
  return true;
}

uint16_t readSingleRegisterWithRetry(uint16_t addr, uint8_t retries = 3) {
  if (addr == 0) return 0xFFFF;
  uint16_t tmp = 0;
  for (uint8_t i = 0; i < retries; i++) {
    if (modbusReadHolding(addr, 1, &tmp) == MB_SUCCESS) return tmp;
    vTaskDelay(pdMS_TO_TICKS(150));
  }
  return 0xFFFF;
}

// ========================================================================
// НАЛАШТУВАННЯ
// ========================================================================
void loadSettings() {
  preferences.begin("settings", true);
  cfgInverterModelIdx = preferences.getInt("invModel", 0);
  if (cfgInverterModelIdx < 0 || cfgInverterModelIdx >= NUM_INVERTER_MODELS) cfgInverterModelIdx = 0;

  cfgHeaterPowerVA        = preferences.getUShort("heaterVA",  100);
  cfgMaxAllocatedVA       = preferences.getUShort("maxAllocVA", 100);
  cfgUpdateIntervalSec    = constrain(preferences.getFloat("updIntSec", DEFAULT_UPDATE_INTERVAL_SEC), 0.5f, 10.0f);
  cfgBattOffVoltage       = constrain(preferences.getFloat("battOffV", 27.0f), 20.0f, 80.0f);
  cfgBattHysteresisV      = constrain(preferences.getFloat("battHystV", 0.5f), 0.0f, 10.0f);
  cfgFullChargeCurrentA   = constrain(preferences.getFloat("chgCurA", 0.0f), -100.0f, 100.0f);
  cfgFullChargeVoltage    = constrain(preferences.getFloat("fullChgV", 29.0f), 20.0f, 80.0f);
  cfgBattDischargeAllowW  = constrain(preferences.getFloat("battDisW", 0.0f), 0.0f, 15000.0f);
  switch_state            = preferences.getBool("swState", false);
  preferences.end();
}

void saveSettings() {
  preferences.begin("settings", false);
  preferences.putInt("invModel", cfgInverterModelIdx);
  preferences.putUShort("heaterVA", cfgHeaterPowerVA);
  preferences.putUShort("maxAllocVA", cfgMaxAllocatedVA);
  preferences.putFloat("updIntSec", cfgUpdateIntervalSec);
  preferences.putFloat("battOffV", cfgBattOffVoltage);
  preferences.putFloat("battHystV", cfgBattHysteresisV);
  preferences.putFloat("chgCurA", cfgFullChargeCurrentA);
  preferences.putFloat("fullChgV", cfgFullChargeVoltage);
  preferences.putFloat("battDisW", cfgBattDischargeAllowW);
  preferences.putBool("swState", switch_state);
  preferences.end();
}

// ========================================================================
// ЗАСТОСУВАННЯ КОНФІГУРАЦІЇ ІНВЕРТОРА
// ========================================================================
void applyInverterConfig() {
  const InverterConfig& invCfg = inverterConfigs[cfgInverterModelIdx];

  Serial1.end();
  delay(100);
  Serial1.begin(invCfg.baudRate, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(500);

  if (invCfg.machineTypeAddr > 0 && invCfg.machinePowerAddr > 0) {
    uint16_t r0 = readSingleRegisterWithRetry(invCfg.machineTypeAddr);
    uint16_t r1 = readSingleRegisterWithRetry(invCfg.machinePowerAddr);
    if (r0 != 0xFFFF && r1 != 0xFFFF) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      inv.machineType = String((char)((r0 >> 8) & 0xFF)) + String((char)(r0 & 0xFF));
      inv.machinePower = String(r1);
      xSemaphoreGive(dataMutex);
    } else {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      inv.machineType = String(invCfg.name);
      inv.machinePower = "—";
      xSemaphoreGive(dataMutex);
    }
  } else {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    inv.machineType = String(invCfg.name);
    inv.machinePower = "—";
    xSemaphoreGive(dataMutex);
  }

  char logLine[128];
  snprintf(logLine, sizeof(logLine),
           ">>> Inverter config switched to: %s (slaveId=0x%02X, baud=%lu)\n",
           invCfg.name, invCfg.slaveId, (unsigned long)invCfg.baudRate);
  serialLog(logLine);
}

// ========================================================================
// ЗАДАЧА КЕРУВАННЯ СІМІСТОРОМ — BURST CONTROL
// ========================================================================
void triacTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TRIAC_TICK_MS);
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
        if ((int32_t)(now - windowStart) >= (int32_t)TRIAC_WINDOW_MS) {
          windowStart = now;
        }
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

// ========================================================================
// ОПИТУВАННЯ MODBUS
// ========================================================================
void pollModbusOnce() {
  bool ok = true;

  digitalWrite(LED_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(50));

  const InverterConfig& invCfg = inverterConfigs[cfgInverterModelIdx];

  if (invCfg.invBlockAddr > 0 && invCfg.invBlockLen > 0) {
    digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);
    if (!readModbusBlock(invCfg.invBlockAddr, invCfg.invBlockLen, invBuf)) ok = false;
    digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);
  }

  if (invCfg.pvBlockAddr > 0 && invCfg.pvBlockLen > 0) {
    digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);
    if (!readModbusBlock(invCfg.pvBlockAddr, invCfg.pvBlockLen, pvBuf)) ok = false;
    digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(30)); digitalWrite(LED_PIN, LOW);
  }

  if (!ok) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  if (dumpEnabled) {
    char line[256];

    if (invCfg.invBlockLen > 0) {
      int n = invCfg.invBlockLen;
      for (int row = 0; row < n; row += 8) {
        int pos = snprintf(line, sizeof(line), "inv[%04X]:",
                           (unsigned)(invCfg.invBlockAddr + row));
        for (int i = row; i < row + 8 && i < n; i++) {
          pos += snprintf(line + pos, sizeof(line) - pos, " %04X", invBuf[i]);
          if (pos >= (int)sizeof(line) - 8) break;
        }
        pos += snprintf(line + pos, sizeof(line) - pos, "\n");
        serialLog(line);
      }
    }

    if (invCfg.pvBlockLen > 0) {
      int n = invCfg.pvBlockLen;
      for (int row = 0; row < n; row += 8) {
        int pos = snprintf(line, sizeof(line), "pv [%04X]:",
                           (unsigned)(invCfg.pvBlockAddr + row));
        for (int i = row; i < row + 8 && i < n; i++) {
          pos += snprintf(line + pos, sizeof(line) - pos, " %04X", pvBuf[i]);
          if (pos >= (int)sizeof(line) - 8) break;
        }
        pos += snprintf(line + pos, sizeof(line) - pos, "\n");
        serialLog(line);
      }
    }
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  inv.workState = readRaw(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.invWorkState);

  pvV      = readScaled(pvBuf, invCfg.pvBlockAddr, invCfg.pvBlockLen, invCfg.pvV);
  pvI      = readScaled(pvBuf, invCfg.pvBlockAddr, invCfg.pvBlockLen, invCfg.pvI);
  pvPower  = readScaledInt(pvBuf, invCfg.pvBlockAddr, invCfg.pvBlockLen, invCfg.pvPower);
  systemV  = readScaled(pvBuf, invCfg.pvBlockAddr, invCfg.pvBlockLen, invCfg.systemV);

  invV     = readScaled(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.invV_0);
  if (invV == 0.0f) invV = readScaled(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.invV_1);
  invI     = readScaled(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.invI);
  invPower = readScaledInt(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.invPower);
  battV    = readScaled(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.battV);
  gridFreq = readScaled(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.gridFreq);
  battI    = readScaled(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.battI);
  battP    = readScaled(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.battP);

  inv.pvWork     = (uint16_t)readRaw(pvBuf,  invCfg.pvBlockAddr,  invCfg.pvBlockLen,  invCfg.pvWork);
  inv.mppt       = (uint16_t)readRaw(pvBuf,  invCfg.pvBlockAddr,  invCfg.pvBlockLen,  invCfg.mppt);
  inv.chg        = (uint16_t)readRaw(pvBuf,  invCfg.pvBlockAddr,  invCfg.pvBlockLen,  invCfg.chg);
  inv.pvEnergyHi = (uint16_t)readRaw(pvBuf,  invCfg.pvBlockAddr,  invCfg.pvBlockLen,  invCfg.pvEnergyHi);
  inv.pvEnergyLo = (uint16_t)readRaw(pvBuf,  invCfg.pvBlockAddr,  invCfg.pvBlockLen,  invCfg.pvEnergyLo);
  inv.acChgHi    = (uint16_t)readRaw(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.acChgHi);
  inv.acChgLo    = (uint16_t)readRaw(invBuf, invCfg.invBlockAddr, invCfg.invBlockLen, invCfg.acChgLo);

  if (!switch_state) {
    freePower = 0;
    currentPower = 0;
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
    freePower = 0;
    currentPower = 0;
    targMessage = "Споживання вимкнено";
  }
  else if (inv.workState != 2) {
    freePower = 0;
    currentPower = 0;
    targMessage = "Інвертер споживає з розетки";
  }
  else if (isBattLow) {
    freePower = 0;
    currentPower = 0;
    targMessage = "Напруга акумулятора менше встановленого мінімуму";
  }
  else if (freePower < 0) {
    currentPower += freePower;
    if (currentPower < 0) currentPower = 0;
    freePower = 0;
    targMessage = "Недостатньо потужності";
  }
  else if (battI < cfgFullChargeCurrentA && battV < cfgFullChargeVoltage) {
    // Струм АКБ нижче мінімуму — не відключаємо, а зменшуємо потужність ТЕНа,
    // щоб дати батареї більше заряду. Крок пропорційний дефіциту струму.
    int32_t deficitW = (int32_t)((cfgFullChargeCurrentA - battI) * battV);
    if (deficitW < cfgMaxAllocatedVA / 20) deficitW = cfgMaxAllocatedVA / 20;   // мінімальний крок, щоб гарантовано зменшити
    currentPower -= deficitW;
    if (currentPower < 0) currentPower = 0;
    freePower = 0;
    targMessage = "Струм акумулятора менше мінімуму — зменшую потужність на 5% від потужності.";
  }
  else if (freePower == 0) {
    targMessage = "Потужність на максимумі";
  }
  else {
    if (currentPower >= cfgMaxAllocatedVA) {
      currentPower = cfgMaxAllocatedVA;
      targMessage = "Потужність обрізано до максимуму";
    }
    else if (freePower > cfgMaxAllocatedVA / 20) {
      currentPower += cfgMaxAllocatedVA / 20;
      if (currentPower > cfgMaxAllocatedVA) currentPower = cfgMaxAllocatedVA;
      targMessage = "Потужність збільшена на 5% від потужності";
    }
    else {
      currentPower += freePower;
      if (currentPower > cfgMaxAllocatedVA) currentPower = cfgMaxAllocatedVA;
      targMessage = "Потужність збільшено";
    }
  }

  PpowerHeater = currentPower;

  if (cfgHeaterPowerVA > 0) {
    currentHeaterPercent = constrain((PpowerHeater * 100) / cfgHeaterPowerVA, 0, 100);
  } else {
    currentHeaterPercent = 0;
  }

  strncpy(targMessageGlobal, targMessage, sizeof(targMessageGlobal) - 1);
  targMessageGlobal[sizeof(targMessageGlobal) - 1] = '\0';

  char logLine[512];
  snprintf(logLine, sizeof(logLine),
           "[%lu] ws=%d pvV=%.1f pvI=%.1f sysV=%.1f invV=%.0f invI=%.1f freq=%.1f "
           "battV=%.1f battI=%.1f battPwr=%.0f pvP=%d invP=%d free=%d cur=%d pph=%d "
           "hVA=%u hpct=%d out=%d%% sw=%d msg=%s\n",
           millis(), inv.workState, pvV, pvI, systemV, invV, invI, gridFreq,
           battV, battI, battP, (int)pvPower, (int)invPower, (int)freePower,
           (int)currentPower, (int)PpowerHeater, (unsigned)cfgHeaterPowerVA,
           (int)currentHeaterPercent, (int)currentTriacDuty,
           switch_state ? 1 : 0, targMessage);
  serialLog(logLine);

  xSemaphoreGive(dataMutex);
  digitalWrite(LED_PIN, HIGH);
}

// ========================================================================
// ЗАДАЧА MODBUS
// ========================================================================
void modbusTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(2000));
  for (;;) {
    if (!webUpdateActive) {
      if (xSemaphoreTake(modbusMutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
        pollModbusOnce();
        xSemaphoreGive(modbusMutex);
      }
    }
    uint32_t intervalMs = (uint32_t)(cfgUpdateIntervalSec * 1000.0f);
    if (intervalMs < 500) intervalMs = 500;
    vTaskDelay(pdMS_TO_TICKS(intervalMs));
  }
}

// ========================================================================
// HTML ШАБЛОНИ
// ========================================================================
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
.current-power-box .label{color:#666;font-size:10px;margin-bottom:2px;text-transform:uppercase;letter-spacing:1px}
.current-power-box .value{font-size:18px;font-weight:bold;color:#00ff88;font-family:monospace}
.status-text{text-align:center;font-size:12px;font-weight:bold;line-height:1.6}
.status-text .machine{color:#4da3ff}
.status-text .state{color:#ffd166}
.alert-box{border:2px solid #ff4d4d;border-radius:6px;padding:10px;background:rgba(255,77,77,.05);min-height:60px;box-sizing:border-box;display:flex;align-items:center;justify-content:center;text-align:center}
.alert-box .value{color:#ff4d4d;font-size:12px;font-weight:bold;line-height:1.4;word-wrap:break-word;overflow-wrap:break-word;word-break:break-word;max-width:100%}
.free-power-box{background:#161616;border-radius:6px;padding:8px;text-align:center;border:1px solid #2a2a2a}
.free-power-box .label{color:#666;font-size:10px;margin-bottom:2px;text-transform:uppercase;letter-spacing:1px}
.free-power-box .value{font-size:18px;font-weight:bold;color:#00ff88;font-family:monospace}
.heater-slider-box{background:#161616;border-radius:6px;padding:8px;text-align:center;border:1px solid #2a2a2a;margin-top:6px}
.heater-slider-box .label{color:#666;font-size:10px;margin-bottom:2px;text-transform:uppercase;letter-spacing:1px}
.slider-track{background:#333;border-radius:4px;height:12px;width:100%;margin-top:4px;overflow:hidden;position:relative}
.slider-fill{height:100%;border-radius:4px;transition:width 0.4s ease, background-color 0.4s ease;width:0%}
.slider-text{color:#888;font-size:10px;margin-top:4px;font-family:monospace}
.switch-row{display:flex;align-items:center;justify-content:center;gap:12px;margin-top:6px;padding:10px 14px;background:#161616;border-radius:6px;border:1px solid #2a2a2a}
.toggle-switch{position:relative;width:96px;height:56px;background-color:#333;border-radius:28px;cursor:pointer;transition:background-color .3s,border-color .3s;border:2px solid #444;flex-shrink:0;box-sizing:border-box}
.toggle-switch.active{background-color:rgba(0,255,136,.15);border-color:#00ff88}
.toggle-knob{position:absolute;top:4px;left:4px;width:44px;height:44px;background-color:#888;border-radius:50%;transition:transform .3s cubic-bezier(.4,0,.2,1),background-color .3s;box-shadow:0 2px 5px rgba(0,0,0,.4)}
.toggle-switch.active .toggle-knob{transform:translateX(40px);background-color:#00ff88}
.switch-label{color:#888;font-size:12px;font-weight:bold;text-transform:uppercase;letter-spacing:.5px}
.controls{margin-top:15px;display:flex;flex-wrap:wrap;gap:8px;justify-content:center;max-width:960px}
.btn{background:#333;color:#fff;border:1px solid #444;padding:7px 12px;border-radius:5px;cursor:pointer;font-size:11px;font-family:inherit;transition:background .2s}
.btn:hover{background:#444}
.btn.primary{background:#00ff88;color:#000;border:none;font-weight:bold}
.btn.warn{background:#ffd166;color:#000;border:none;font-weight:bold}
.btn.danger{background:#ff4d4d;color:#fff;border:none;font-weight:bold}
.update-section{background:#1e1e1e;padding:16px;border-radius:10px;margin-top:12px;width:100%;max-width:500px;box-sizing:border-box;display:none;box-shadow:0 4px 6px rgba(0,0,0,.3)}
.update-section.active{display:block}
@media(max-width:768px){.dashboard{grid-template-columns:1fr}}
</style></head><body>
<div class="dashboard">
  <div class="col">
    <div class="panel">
      <div class="panel-header"><svg viewBox="0 0 24 24"><path d="M12 7c-2.76 0-5 2.24-5 5s2.24 5 5 5 5-2.24 5-5-2.24-5-5-5zM2 13h2c.55 0 1-.45 1-1s-.45-1-1-1H2c-.55 0-1 .45-1 1s.45 1 1 1zm18 0h2c.55 0 1-.45 1-1s-.45-1-1-1h-2c-.55 0-1 .45-1 1s.45 1 1 1zM11 2v2c0 .55.45 1 1 1s1-.45 1-1V2c0-.55-.45-1-1-1s-1 .45-1 1zm0 18v2c0 .55.45 1 1 1s1-.45 1-1v-2c0-.55-.45-1-1-1s-1 .45-1 1zM5.99 4.58c-.39-.39-1.03-.39-1.41 0-.39.39-.39 1.03 0 1.41l1.06 1.06c.39.39 1.03.39 1.41 0 .39-.39.39-1.03 0-1.41L5.99 4.58zm12.37 12.37c-.39-.39-1.03-.39-1.41 0-.39.39-.39 1.03 0 1.41l1.06 1.06c.39.39 1.03.39 1.41 0 .39-.39.39-1.03 0-1.41l-1.06-1.06zm1.06-10.96c-.39-.39.39-1.03 0-1.41-.39-.39-1.03-.39-1.41 0l-1.06 1.06c-.39.39-.39 1.03 0 1.41.39.39 1.03.39 1.41 0l1.06-1.06zM7.05 18.36c-.39-.39.39-1.03 0-1.41-.39-.39-1.03-.39-1.41 0l-1.06 1.06c-.39.39-.39 1.03 0 1.41.39.39 1.03.39 1.41 0l1.06-1.06z"/></svg>SOLAR POWER</div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">PV VOLTAGE</div><div class="value yellow" id="pvV">--</div></div>
        <div class="metric"><div class="label">PV CURRENT</div><div class="value yellow" id="pvI">--</div></div>
      </div>
      <div class="big-box"><div class="label">PV POWER (NOW)</div><div class="value yellow" id="pvPwr">--</div></div>
    </div>
    <div class="panel">
      <div class="panel-header blue"><svg viewBox="0 0 24 24"><path d="M13 3h-2v10h2V3zm4.83 2.17l-1.42 1.42C17.99 7.86 19 9.81 19 12c0 3.87-3.13 7-7 7s-7-3.13-7-7c0-2.19 1.01-4.14 2.58-5.42L6.17 5.17C4.23 6.82 3 9.26 3 12c0 4.97 4.03 9 9 9s9-4.03 9-9c0-2.74-1.23-5.18-3.17-6.83z"/></svg>INVERTOR POWER</div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">INV VOLTAGE</div><div class="value blue" id="invV">--</div></div>
        <div class="metric"><div class="label">INV CURRENT</div><div class="value blue" id="invI">--</div></div>
      </div>
      <div class="big-box"><div class="label">INV POWER</div><div class="value red" id="invPwr">--</div></div>
    </div>
  </div>
  <div class="col">
    <div class="panel center-panel">
      <div class="current-power-box"><div class="label">CURRENT HEATER POWER</div><div class="value" id="curP">0 VA</div></div>
      <div class="heater-slider-box">
        <div class="label">HEATER LOAD <span id="hNominal" style="color:#666"></span></div>
        <div class="slider-track"><div id="hSlider" class="slider-fill" style="width: 0%; background: #00ff88;"></div></div>
        <div id="hSliderText" class="slider-text">0%</div>
      </div>
      <div class="switch-row">
        <span class="switch-label">СПОЖИВАННЯ</span>
        <div class="toggle-switch" id="switchBtn" onclick="toggleSwitch()"><div class="toggle-knob"></div></div>
        <span class="switch-label" id="switchStatus" style="color:#888">ВИМК</span>
      </div>
      <div class="status-text"><span class="machine" id="machineInfo">--</span><br><span class="state" id="stateInfo">--</span></div>
      <div class="alert-box" id="alertBox"><div class="value" id="targMsg">--</div></div>
      <div class="free-power-box"><div class="label">FREE POWER</div><div class="value" id="freeP">0 VA</div></div>
    </div>
  </div>
  <div class="col">
    <div class="panel">
      <div class="panel-header blue"><svg viewBox="0 0 24 24"><path d="M15.67 4H14V2h-4v2H8.33C7.6 4 7 4.6 7 5.33v15.33C7 21.4 7.6 22 8.33 22h7.33c.74 0 1.34-.6 1.34-1.33V5.33C17 4.6 16.4 4 15.67 4zM11 20v-5.5H9L13 7v5.5h2L11 20z"/></svg>BATTERY</div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">BATTERY VOLTAGE</div><div class="value blue" id="battV">--</div></div>
        <div class="metric"><div class="label">BATTERY CURRENT</div><div class="value blue" id="battI">--</div></div>
      </div>
      <div class="big-box"><div class="label">BATTERY POWER</div><div class="value" id="battPwr">--</div></div>
    </div>
    <div class="panel">
      <div class="panel-header"><svg viewBox="0 0 24 24"><path d="M11 21h-1l1-7H7.5c-.58 0-.57-.32-.38-.66.19-.34.05-.08.07-.12C8.48 10.94 10.42 7.54 13 3h1l-1 7h3.5c.49 0 .56.33.47.51l-.07.15C12.96 17.55 11 21 11 21z"/></svg>POWER INFO</div>
      <div class="stack">
        <div class="metric"><div class="label">GRID FREQ</div><div class="value blue" id="gridFreq">--</div></div>
        <div class="metric"><div class="label">PV ENERGY TOTAL</div><div class="value yellow" id="pvAccum">--</div></div>
        <div class="metric"><div class="label">ACCUM SELF USE</div><div class="value green" id="accChg">--</div></div>
      </div>
    </div>
  </div>
</div>
<div class="controls">
<button class="btn primary" onclick="update()">Refresh</button>
<button class="btn" onclick="location.href='/settings'">Settings</button>
<button class="btn" id="signalBtn" onclick="location.href='/signal'">Сигнал</button>
<button class="btn warn" onclick="resetCounters()">Reset counters</button>
<button class="btn" onclick="resetWifi()">Reset WiFi</button>
<button class="btn danger" onclick="restartEsp()">Restart ESP</button>
<button class="btn" onclick="location.href='/serial'">Serial Monitor</button>
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
function wsStr(v){switch(v){case 0:return"POWER ON";case 1:return"SELFTEST";case 2:return"OFF GRID";case 3:return"GRID TIE";case 4:return"BYPASS";case 5:return"STOP";case 6:return"GRID CHRG";default:return"UNKNOWN"}}
function mpptStr(v){switch(v){case 0:return"Stop";case 1:return"MPPT";case 2:return"Current limit";default:return"--"}}
function chgStr(v){switch(v){case 0:return"Stop";case 1:return"Charging";case 2:return"Float";default:return"--"}}
function isBad(v){return v===null||v===undefined||v===65535}
function fmt1(v,unit){if(isBad(v))return"--";return Number(v).toFixed(1)+(unit?" "+unit:"")}
function fmt0(v,unit){if(isBad(v))return"--";return Math.round(Number(v))+(unit?" "+unit:"")}
function fmtKwh(hi,lo){if(isBad(hi)||isBad(lo))return"--";return(hi*1000+lo/10).toFixed(1)+" kWh"}
function toggleUpdate(){document.getElementById("updateSection").classList.toggle("active")}
async function resetWifi(){if(confirm("Reset WiFi?")){await fetch("/reset_wifi");location.reload()}}
async function resetCounters(){
  if(!confirm("Reset all counters?"))return;
  try{
    let r = await fetch("/reset_counters");
    if(r.status===200) alert("Reset command sent");
    else if(r.status===403) alert("Access denied: Local network only");
    else alert("Error: "+r.status);
  }catch(e){ alert("Connection error"); }
}
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
x.send(fd)}
async function toggleSwitch(){
try{let resp=await fetch("/toggle_switch",{method:"POST"});
if(resp.status==403){alert("Access denied: Local network only");return}
let data=await resp.json();updateSwitchUI(data.state);}catch(e){alert("Connection error!")}}
function updateSwitchUI(state){
let switchEl=document.getElementById("switchBtn");let status=document.getElementById("switchStatus");
if(state){switchEl.classList.add("active");status.innerText="УВІМК";status.style.color="#00ff88";}
else{switchEl.classList.remove("active");status.innerText="ВИМК";status.style.color="#888";}}
async function update(){
try{let d=await(await fetch("/api")).json();

let modelName = ((d.mt && d.mt !== "UNKNOWN") ? d.mt + " " : "") + ((d.mp && d.mp !== "—") ? d.mp : "");
let sysVText = (d.systemV !== null && d.systemV !== undefined && d.systemV > 0) ? " | " + d.systemV.toFixed(1) : "";
document.getElementById("machineInfo").innerText = modelName + sysVText;

document.getElementById("stateInfo").innerText=wsStr(d.ws)+" | MPPT: "+mpptStr(d.mppt)+" | CHG: "+chgStr(d.chg);
document.getElementById("pvV").innerText=fmt1(d.pvV,"V");document.getElementById("pvI").innerText=fmt1(d.pvI,"A");
document.getElementById("pvPwr").innerText = fmt0(d.pvPower, "VA");
document.getElementById("pvAccum").innerText=fmtKwh(d.pvEnergyHi,d.pvEnergyLo);
document.getElementById("invV").innerText=fmt1(d.invV,"V");document.getElementById("invI").innerText=fmt1(d.invI,"A");
document.getElementById("invPwr").innerText=fmt0(d.invPower,"VA");
document.getElementById("gridFreq").innerText=(!isBad(d.freq)&&d.freq>0)?d.freq.toFixed(1)+" Hz":"--";
document.getElementById("battV").innerText=(d.battV!=null&&d.battV>0)?d.battV.toFixed(1)+" V":"--";
let battIEl=document.getElementById("battI");
battIEl.innerText=d.battI.toFixed(1)+" A";
battIEl.className="value "+(d.battI>=0?"blue":"red");
let battPwrEl=document.getElementById("battPwr");
if(!isBad(d.battP)){battPwrEl.innerText=Math.round(d.battP)+" VA";battPwrEl.className="value "+(d.battP>=0?"blue":"red");}
else{battPwrEl.innerText="--";battPwrEl.className="value blue";}
document.getElementById("accChg").innerText=fmtKwh(d.acChgHi,d.acChgLo);
document.getElementById("curP").innerText=fmt0(d.cp,"VA");document.getElementById("freeP").innerText=fmt0(d.fp,"VA");
let hpct = d.hpct || 0;let hSlider = document.getElementById("hSlider");
hSlider.style.width = hpct + "%";
let hColor = hpct < 33 ? "#00ff88" : (hpct < 66 ? "#ffd166" : "#ff4d4d");
hSlider.style.background = hColor;
document.getElementById("hSliderText").innerText = hpct + "%";document.getElementById("hSliderText").style.color = hColor;
if(d.hva)document.getElementById("hNominal").innerText="("+d.hva+" VA nominal)";
updateSwitchUI(d.sw);
if(d.isLocal === 0){
  let fb=document.getElementById("fwBtn"); if(fb) fb.style.display="none";
  let us=document.getElementById("updateSection"); if(us){us.style.display="none";us.classList.remove("active");}
}
let sigBtn=document.getElementById("signalBtn");if(sigBtn){sigBtn.innerText="Сигнал: "+((d.rssi!=null&&d.rssi!==0)?d.rssi+" dBm":"--");}
let tm=d.tm||"--";let tmEl=document.getElementById("targMsg");let box=document.getElementById("alertBox");
tmEl.innerText=tm;
if(tm==="Все добре"){tmEl.style.color="#00ff88";box.style.borderColor="#00ff88";box.style.background="rgba(0,255,136,0.05)";}
else{tmEl.style.color="#ff4d4d";box.style.borderColor="#ff4d4d";box.style.background="rgba(255,77,77,0.05)";}
}catch(e){console.error(e)}}
setInterval(update,1000);update();
</script></body></html>
)rawliteral";

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
<button class="btn bg" id="autoBtn" onclick="toggleAutoScroll()">Auto-scroll: ON</button>
<button class="btn bg" id="dumpBtn" onclick="toggleDump()">Dump: OFF</button>
<button class="btn bg" onclick="location.href='/'">Back</button>
</div>
<div id="log">Loading...</div>
<script>
let autoScroll = true;
let dumpOn = false;

function toggleAutoScroll(){
  autoScroll = !autoScroll;
  let b = document.getElementById("autoBtn");
  b.innerText = autoScroll ? "Auto-scroll: ON" : "Auto-scroll: OFF";
}

function updateDumpBtn(){
  let b = document.getElementById("dumpBtn");
  if(!b) return;
  b.innerText = dumpOn ? "Dump: ON" : "Dump: OFF";
  b.style.background = dumpOn ? "#00ff88" : "#555";
  b.style.color = dumpOn ? "#000" : "#fff";
}

async function loadDumpState(){
  try{
    let r = await fetch("/api_dump");
    let d = await r.json();
    dumpOn = (d.dump === 1);
  }catch(e){}
  updateDumpBtn();
}

async function toggleDump(){
  dumpOn = !dumpOn;
  try{
    let r = await fetch("/api_dump?state=" + (dumpOn ? "1" : "0"));
    let d = await r.json();
    dumpOn = (d.dump === 1);
  }catch(e){}
  updateDumpBtn();
}

async function fetchLog(){
  try{
    let r = await fetch("/api_serial");
    let t = await r.text();
    let el = document.getElementById("log");
    el.innerText = t;
    if(autoScroll) el.scrollTop = el.scrollHeight;
  }catch(e){
    document.getElementById("log").innerText = "Error: " + e;
  }
}

loadDumpState();
fetchLog();
setInterval(fetchLog, 2000);
</script></body></html>
)rawliteral";

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
input, select{width:100%;box-sizing:border-box;padding:10px;margin:5px 0;background:#333;color:#fff;border:1px solid #555;border-radius:6px;font-size:14px}
.btn{background:#00ff88;color:#000;border:none;padding:10px 20px;border-radius:6px;cursor:pointer;font-weight:bold;margin:5px}
.bg{background:#555;color:#fff}.msg{color:#00ff88;text-align:center;margin:10px 0;display:none}
.err{color:#ff4d4d;text-align:center;margin:10px 0;display:none}
</style></head><body>
<h1>Settings</h1>
<div class="msg" id="msg">Saved!</div><div class="err" id="err">Access denied: Local network only</div>

<h2>Тип інвертора</h2><div class="card">
<label>Модель інвертора</label>
<div class="desc">Оберіть модель вашого інвертора. Від цього залежить, за якими адресами Modbus читатимуться напруги, струми, потужності та лічильники енергії. Якщо обрано неправильну модель — дані будуть некоректними або нульовими.</div>
<div class="hint">Доступні моделі: SRNE / Must / EASUN — повна підтримка. Growatt, Voltronic / Axpert — заготовки (потребують заповнення адрес).</div>
<select id="invModel"></select>
</div>

<h2>General</h2><div class="card">
<label>Heater power (VA)</label>
<div class="desc">Номінальна потужність вашого ТЕНа (нагрівача) у вольт-амперах. Від цього значення залежить відсоток навантаження: якщо виділена потужність 250 ВА, а ТЕН на 1000 ВА — буде показано 25%. Якщо вказати завищене значення, ТЕН ніколи не вийде на 100%, якщо занижене — регулювання буде грубим.</div>
<div class="hint">Приклад: ТЕН 1 кВт → вкажіть 1000. ТЕН 500 Вт → вкажіть 500.</div>
<input type="number" id="heaterVA" min="100" max="15000" value="1000">

<label>Max allocated power (VA)</label>
<div class="desc">Максимальна потужність, яку система дозволяє віддати на ТЕН. Це «стеля» регулювання — потужність ніколи не перевищить це значення, навіть якщо є надлишок енергії. Захищає інвертор і акумулятор від перевантаження.</div>
<div class="hint">Має бути ≤ номіналу інвертора. Для інвертора 1 кВт розумно ставити 500–800 ВА.</div>
<input type="number" id="maxAllocVA" min="100" max="15000" value="500">

<label>Update interval (sec)</label>
<div class="desc">Як часто ESP32 опитує інвертор через Modbus. Менший інтервал — швидша реакція на зміну сонця чи навантаження, але більше навантаження на шину RS-485. Занадто малий інтервал може спричинити помилки CRC або таймаути.</div>
<div class="hint">Рекомендовано: 1.0–3.0 сек. Мінімум 0.5, максимум 10.</div>
<input type="number" id="updIntSec" min="0.5" max="10" step="0.1" value="2">
</div>

<h2>Load shutdown</h2><div class="card">
<label>Hysteresis (V)</label>
<div class="desc">Гістерезис — різниця між порогом вимкнення і порогом повторного вмикання. Запобігає «блиманню» реле, коли напруга коливається біля порогу. Наприклад, при shutdown=27 В і hysteresis=0.5 В: вимкнеться при 27.0 В, увімкнеться назад лише при 27.5 В.</div>
<div class="hint">Рекомендовано: 0.3–1.0 В. Менше — швидше реакція, але більше «блимання».</div>
<input type="number" id="battHystV" min="0" max="5" step="0.1" value="0.5">

<label>Shutdown voltage (V)</label>
<div class="desc">Критична напруга акумулятора, при якій навантаження (ТЕН) вимикається повністю, щоб запобігти глибокому розряду та пошкодженню батареї. Поки напруга нижче цього порогу — ТЕН не вмикається, навіть якщо є надлишок сонця.</div>
<div class="hint">Для 24 В свинцево-кислотної АКБ — 24.0–25.0 В. Для 48 В — 48.0–50.0 В. Для LiFePO4 — вище (напр. 25.5 для 24 В).</div>
<input type="number" id="battOffV" min="20" max="80" step="0.1" value="27">

<h2>Battery charging</h2><div class="card">
<label>Full charge voltage (V)</label>
<div class="desc">Напруга повного заряду АКБ. Коли напруга батареї досягає цього значення — резервування струму зарядки (поле вище) припиняється, і вся надлишкова потужність іде на ТЕН. Це дозволяє використати надлишок, коли АКБ вже повністю заряджена.</div>
<div class="hint">Приклад: 28.8 В для свинцево-кислотної 24 В, 29.2 В для LiFePO4 24 В.</div>
<input type="number" id="fullChgV" min="20" max="80" step="1" value="29">

<label>Charge current (A)</label>
<div class="desc">Струм, який резервується для зарядки акумулятора. Додатне значення — скільки ампер «забронювати» для АКБ перед тим, як решта піде на ТЕН. Від'ємне значення — навпаки, додає бюджет ТЕНу: фактично дозволяє розряджати АКБ цим струмом на нагрів.</div>
<div class="hint">Приклад: +10 = зарезервувати 10 А на заряд АКБ. -5 = дозволити ТЕНу брати 5 А з АКБ. 0 = не резервувати й не розряджати.</div>
<input type="number" id="chgCurA" min="-100" max="100" step="1" value="0">

<label>Battery discharge allow (W)</label>
<div class="desc">Дозвіл брати енергію з акумулятора на ТЕН. За замовчуванням 0 — ТЕН працює лише на надлишок сонця, АКБ не розряджається. Якщо вказати, наприклад, 300 Вт — система дозволить ТЕНу споживати до 300 Вт з АКБ понад те, що дає сонце. Це дозволяє утилізувати запас батареї для нагріву.</div>
<div class="hint">Обережно: великі значення швидко розряджають АКБ. Захист «Shutdown voltage» і «Shutdown current» продовжують діяти.</div>
<div class="hint">УВАГА: якщо ви встановлюєте це значення — ОБОВ'ЯЗКОВО ЗМЕНШІТЬ значення поля <b>Charge current (A)</b> на величину розрахованого струму. Розрахунок: Battery discharge allow (W) / напруга АКБ = струм розряду (A). Для 24 В: 240 W / 24 V = 10 A; 480 W / 24 V = 20 A. Для 48 В: 240 W / 48 V = 5 A; 480 W / 48 V = 10 A.</div>
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
    ["heaterVA","maxAllocVA","updIntSec","battOffV","battHystV","chgCurA","fullChgV","battDisW"].forEach(k=>document.getElementById(k).value=d[k]);
    let invSelect = document.getElementById("invModel");
    invSelect.innerHTML = "";
    if (d.models) {
      d.models.forEach((m, idx) => {
        let opt = document.createElement("option");
        opt.value = idx; opt.text = m.name;
        if (idx == d.currentModel) opt.selected = true;
        invSelect.appendChild(opt);
      });
    }
  }catch(e){}
}
async function save(){
let body=["heaterVA","maxAllocVA","updIntSec","battOffV","battHystV","chgCurA","fullChgV","battDisW","invModel"].map(k=>k+"="+document.getElementById(k).value).join("&");
try{let resp=await fetch("/save_settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body});
if(resp.status==200){document.getElementById("msg").style.display="block";document.getElementById("err").style.display="none";setTimeout(()=>document.getElementById("msg").style.display="none",2000);}
else if(resp.status==403){document.getElementById("err").style.display="block";document.getElementById("msg").style.display="none";setTimeout(()=>document.getElementById("err").style.display="none",3000);}
else{alert("Save error!");}}catch(e){alert("Connection error!");}}
load();
</script></body></html>
)rawliteral";

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
.fill{height:100%;width:0;background:#00ff88;transition:width .4s,background .4s}
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
try{let d=await(await fetch('/api_signal')).json();let q=d.quality||0;
document.getElementById('q').innerText=q+'%';
document.getElementById('rssi').innerText='RSSI: '+((d.rssi!=null&&d.rssi!==0)?d.rssi+' dBm':'--');
let fill=document.getElementById('fill');fill.style.width=q+'%';
fill.style.background=q>70?'#00ff88':(q>40?'#ffd166':'#ff4d4d');
let st='--';if(d.status==='CONNECTED') st='Підключено';else if(d.status==='DISCONNECTED') st='Не підключено';
document.getElementById('status').innerText=st;
document.getElementById('ssid').innerText=esc(d.ssid);document.getElementById('ip').innerText=esc(d.ip);
document.getElementById('mac').innerText=esc(d.mac);
}catch(e){document.getElementById('q').innerText='Помилка'}}
upd();setInterval(upd,2000);
</script></body></html>
)rawliteral";

// ========================================================================
// СИСТЕМНІ ФУНКЦІЇ
// ========================================================================
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

// ========================================================================
// ОНОВЛЕННЯ ПРОШИВКИ — ЛИШЕ З ЛОКАЛЬНОЇ МЕРЕЖІ
// ========================================================================
void onUpdateUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
  if (!request->client() || !isLocalIP(request->client()->remoteIP())) {
    if (!index) {
      Update.abort();
      webUpdateActive = false;
    }
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

    DynamicJsonDocument doc(2048);
    doc["ws"] = inv.workState;
    doc["mt"] = inv.machineType;
    doc["mp"] = inv.machinePower;
    doc["fp"] = (int)freePower;
    doc["cp"] = (int)currentPower;
    doc["pph"] = (int)PpowerHeater;
    doc["hpct"] = currentHeaterPercent;
    doc["hva"] = (unsigned)cfgHeaterPowerVA;
    doc["maVA"] = (unsigned)cfgMaxAllocatedVA;
    doc["sw"] = switch_state ? 1 : 0;
    doc["tm"] = targCopy;
    doc["pvV"] = pvV; doc["pvI"] = pvI;
    doc["pvPower"] = (int)pvPower;
    doc["systemV"] = systemV;
    doc["invV"] = invV; doc["invI"] = invI;
    doc["invPower"] = (int)invPower;
    doc["battV"] = battV; doc["battI"] = battI; doc["battP"] = battP;
    doc["freq"] = gridFreq;
    doc["pvEnergyHi"] = (unsigned)inv.pvEnergyHi;
    doc["pvEnergyLo"] = (unsigned)inv.pvEnergyLo;
    doc["acChgHi"]    = (unsigned)inv.acChgHi;
    doc["acChgLo"]    = (unsigned)inv.acChgLo;
    doc["pvWork"]     = (unsigned)inv.pvWork;
    doc["mppt"]       = (unsigned)inv.mppt;
    doc["chg"]        = (unsigned)inv.chg;
    doc["rssi"] = WiFi.RSSI();
    doc["isLocal"] = (r->client() && isLocalIP(r->client()->remoteIP())) ? 1 : 0;
    auto* res = r->beginResponseStream("application/json; charset=utf-8");
    serializeJson(doc, *res);
    r->send(res);
    if (locked) xSemaphoreGive(dataMutex);
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *r) { r->send_P(200, "text/html", SETTINGS_HTML); });

  server.on("/api_settings", HTTP_GET, [](AsyncWebServerRequest *r) {
    DynamicJsonDocument doc(1024);
    doc["currentModel"] = cfgInverterModelIdx;
    doc["heaterVA"] = (unsigned)cfgHeaterPowerVA;
    doc["maxAllocVA"] = (unsigned)cfgMaxAllocatedVA;
    doc["updIntSec"] = cfgUpdateIntervalSec;
    doc["battOffV"] = cfgBattOffVoltage;
    doc["battHystV"] = cfgBattHysteresisV;
    doc["chgCurA"] = cfgFullChargeCurrentA;
    doc["fullChgV"] = cfgFullChargeVoltage;
    doc["battDisW"] = cfgBattDischargeAllowW;
    JsonArray models = doc.createNestedArray("models");
    for (int i = 0; i < NUM_INVERTER_MODELS; i++) {
      JsonObject m = models.createNestedObject();
      m["id"] = inverterConfigs[i].id;
      m["name"] = inverterConfigs[i].name;
    }
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
    int quality = wifiQualityPercent(rssi);
    const char* status = (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "DISCONNECTED";
    DynamicJsonDocument doc(256);
    doc["rssi"] = rssi;
    doc["quality"] = quality;
    doc["status"] = status;
    doc["ssid"] = WiFi.SSID();
    doc["ip"] = WiFi.localIP().toString();
    doc["mac"] = WiFi.macAddress();
    auto* res = r->beginResponseStream("application/json; charset=utf-8");
    serializeJson(doc, *res);
    r->send(res);
  });

  server.on("/save_settings", HTTP_POST, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      int newModelToApply = -1;
      if (req->hasParam("invModel", true)) {
        int newModel = req->getParam("invModel", true)->value().toInt();
        if (newModel >= 0 && newModel < NUM_INVERTER_MODELS && newModel != cfgInverterModelIdx) {
          newModelToApply = newModel;
        }
      }

      uint16_t newHeaterVA   = cfgHeaterPowerVA;
      uint16_t newMaxAllocVA = cfgMaxAllocatedVA;
      float    newUpdInt     = cfgUpdateIntervalSec;
      float    newBattOffV   = cfgBattOffVoltage;
      float    newBattHystV  = cfgBattHysteresisV;
      float    newChgCurA    = cfgFullChargeCurrentA;
      float    newFullChgV   = cfgFullChargeVoltage;
      float    newBattDisW   = cfgBattDischargeAllowW;

      if (req->hasParam("heaterVA",   true)) newHeaterVA   = constrain(req->getParam("heaterVA",   true)->value().toInt(),   100,   15000);
      if (req->hasParam("maxAllocVA", true)) newMaxAllocVA = constrain(req->getParam("maxAllocVA", true)->value().toInt(),   100,   15000);
      if (req->hasParam("updIntSec",  true)) newUpdInt     = constrain(req->getParam("updIntSec",  true)->value().toFloat(), 0.5f,  10.0f);
      if (req->hasParam("battOffV",   true)) newBattOffV   = constrain(req->getParam("battOffV",   true)->value().toFloat(), 20.0f, 80.0f);
      if (req->hasParam("battHystV",  true)) newBattHystV  = constrain(req->getParam("battHystV",  true)->value().toFloat(), 0.0f,  10.0f);
      if (req->hasParam("chgCurA",    true)) newChgCurA    = constrain(req->getParam("chgCurA",    true)->value().toFloat(), -100.0f, 100.0f);
      if (req->hasParam("fullChgV",   true)) newFullChgV   = constrain(req->getParam("fullChgV",   true)->value().toFloat(), 20.0f, 80.0f);
      if (req->hasParam("battDisW",   true)) newBattDisW   = constrain(req->getParam("battDisW",   true)->value().toFloat(), 0.0f, 15000.0f);

      if (dataMutex && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        cfgHeaterPowerVA        = newHeaterVA;
        cfgMaxAllocatedVA       = newMaxAllocVA;
        cfgUpdateIntervalSec    = newUpdInt;
        cfgBattOffVoltage       = newBattOffV;
        cfgBattHysteresisV      = newBattHystV;
        cfgFullChargeCurrentA   = newChgCurA;
        cfgFullChargeVoltage    = newFullChgV;
        cfgBattDischargeAllowW  = newBattDisW;
        xSemaphoreGive(dataMutex);
      }

      if (newModelToApply >= 0) {
        if (xSemaphoreTake(modbusMutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
          cfgInverterModelIdx = newModelToApply;
          applyInverterConfig();
          xSemaphoreGive(modbusMutex);
        }
      }
      saveSettings();
      req->send(200, "text/plain", "OK");
    });
  });

  server.on("/toggle_switch", HTTP_POST, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      switch_state = !switch_state;

      if (!switch_state && dataMutex &&
          xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        freePower = 0;
        currentPower = 0;
        PpowerHeater = 0;
        currentHeaterPercent = 0;
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

  server.on("/reset_counters", HTTP_GET, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) { pendingCounterReset = true; req->send(200, "text/plain", "OK"); });
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

// ========================================================================
// Расшифровка кодов статуса
// ========================================================================
const char* mbErrStr(uint8_t st) {
  switch (st) {
    case MB_SUCCESS:          return "SUCCESS";
    case MB_INVALID_SLAVE_ID: return "INVALID_SLAVE_ID";
    case MB_INVALID_FUNCTION: return "INVALID_FUNCTION";
    case MB_RESPONSE_TIMEOUT: return "RESPONSE_TIMEOUT";
    case MB_INVALID_CRC:      return "INVALID_CRC";
    default:                  return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIAC_PIN, OUTPUT);
  digitalWrite(TRIAC_PIN, LOW);
  currentTriacDuty = 0;

  pinMode(RS485_CTRL, OUTPUT); digitalWrite(RS485_CTRL, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  WiFi.persistent(false);

  preferences.begin("wifi_config", true);
  wifiSsid = preferences.getString("ssid", "");
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
  dataMutex = xSemaphoreCreateMutex();
  modbusMutex = xSemaphoreCreateMutex();

  applyInverterConfig();
  delay(1000);

  setupWebServer();

  xTaskCreatePinnedToCore(triacTask,  "triacTask",  2048, nullptr, 2, &triacTaskHandle,  0);
  xTaskCreatePinnedToCore(modbusTask, "modbusTask", 4096, nullptr, 1, &modbusTaskHandle, 0);

  serialLog("=== System started (Burst Control v7.2.5, raw Modbus) ===\n");
}

void loop() {
  unsigned long now = millis();
  if (needRestart && now - restartRequestedMs >= RESTART_DELAY_MS) ESP.restart();

  if (setupMode) {
    dnsServer.processNextRequest();
    vTaskDelay(pdMS_TO_TICKS(5));
    return;
  }

  if (webUpdateActive && (now - lastWebUpdateActivityMs) > WEB_UPDATE_TIMEOUT_MS) {
    webUpdateActive = false;
    Update.abort();
    serialLog("OTA timeout — webUpdateActive cleared\n");
  }

  if (WiFi.status() != WL_CONNECTED && now - lastWifiReconnectMs >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWifiReconnectMs = now;
    WiFi.reconnect();
  }

  if (pendingCounterReset) {
    pendingCounterReset = false;

    if (xSemaphoreTake(modbusMutex, portMAX_DELAY) == pdTRUE) {
      const InverterConfig& invCfg = inverterConfigs[cfgInverterModelIdx];

      if (invCfg.resetCounter1Addr > 0) {
        uint8_t excCode = 0;
        uint8_t st = modbusWriteSingle(invCfg.resetCounter1Addr, 1, &excCode);

        if (st != MB_SUCCESS) {
          vTaskDelay(pdMS_TO_TICKS(100));
          uint16_t v1 = 1;
          st = modbusWriteMultiple(invCfg.resetCounter1Addr, &v1, 1, &excCode);
        }

        if (st != MB_SUCCESS) {
          vTaskDelay(pdMS_TO_TICKS(100));
          uint16_t v0 = 0;
          st = modbusWriteMultiple(invCfg.resetCounter1Addr, &v0, 1, &excCode);
        }

        char l[192];
        if (st == MB_INVALID_FUNCTION) {
          snprintf(l, sizeof(l),
                   ">>> RESET1 0x%04X status=EXCEPTION exc=0x%02X\n",
                   invCfg.resetCounter1Addr, excCode);
        } else {
          snprintf(l, sizeof(l),
                   ">>> RESET1 0x%04X final_status=0x%02X (%s)\n",
                   invCfg.resetCounter1Addr, st, mbErrStr(st));
        }
        serialLog(l);
        vTaskDelay(pdMS_TO_TICKS(150));
      }

      if (invCfg.resetCounter2Addr > 0) {
        uint8_t excCode = 0;
        uint8_t st = modbusWriteSingle(invCfg.resetCounter2Addr, 1, &excCode);

        char l[192];
        snprintf(l, sizeof(l),
         ">>> RESET2 0x%04X status=0x%02X (%s)\n",
         invCfg.resetCounter2Addr, st, mbErrStr(st));
        serialLog(l);
        vTaskDelay(pdMS_TO_TICKS(150));
      }

      xSemaphoreGive(modbusMutex);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}
