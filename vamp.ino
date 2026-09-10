#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ModbusMaster.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <Update.h>
#include <functional>

#define RX2_PIN 16
#define TX2_PIN 17
#define RS485_CTRL 4
#define SLAVE_ID 0x04
#define LED_PIN 2
#define SKETCH_VERSION "6.3.22" 
#define SERIAL_BUF_SIZE 4096

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
constexpr uint32_t RESTART_DELAY_MS = 2000;
constexpr float DEFAULT_UPDATE_INTERVAL_SEC = 2.0f;

SemaphoreHandle_t dataMutex = nullptr;
volatile bool webUpdateActive = false;
volatile bool pendingCounterReset = false;

static uint16_t invBuf[80], pvBuf[20];

uint16_t cfgHeaterPowerVA = 0;
uint16_t cfgMaxAllocatedVA = 0;
float cfgUpdateIntervalSec = DEFAULT_UPDATE_INTERVAL_SEC;
float cfgBattOffVoltage = 0.0f;
float cfgBattHysteresisV = 0.25f;
float cfgBattOffCurrent = 0.0f;
float cfgFullChargeCurrentA = 0.0f;
float cfgFullChargeVoltage = 0.0f;
float battV = 0.0f;

int32_t freePower = 0;
int32_t currentPower = 0;
int32_t pvPower = 0;
int32_t invPower = 0;
int32_t PpowerHeater = 0;
float battI = 0.0f;
float battPower = 0.0f;

bool switch_state = false;

static char targMessageGlobal[128] = "--";

static char serialBuf[SERIAL_BUF_SIZE];
static volatile uint16_t serialHead = 0;
static volatile uint16_t serialTail = 0;
static portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;

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

struct InverterData {
  uint16_t inv[80];
  uint16_t pv[20];
  int workState = 0;
  const char* stateStr = "OFF";
  String machineType, machinePower;
};

AsyncWebServer server(80);
DNSServer dnsServer;
Preferences preferences;
ModbusMaster node;
InverterData inv;
String wifiSsid, wifiPassword;
bool setupMode = false;
unsigned long lastUpdateMs = 0, lastWifiReconnectMs = 0;
bool needRestart = false;
unsigned long restartRequestedMs = 0;

void preTrans() {
  digitalWrite(RS485_CTRL, HIGH);
  delayMicroseconds(100);
}
void postTrans() {
  delayMicroseconds(100);
  Serial1.flush();
  digitalWrite(RS485_CTRL, LOW);
}

template<typename T>
bool readModbusBlock(uint16_t addr, uint8_t qty, T* buf) {
  node.clearResponseBuffer();
  if (node.readHoldingRegisters(addr, qty) == node.ku8MBSuccess) {
    for (uint8_t i = 0; i < qty; i++) buf[i] = node.getResponseBuffer(i);
    return true;
  }
  return false;
}

uint16_t readSingleRegisterWithRetry(uint16_t addr, uint8_t retries = 3) {
  for (uint8_t i = 0; i < retries; i++) {
    node.clearResponseBuffer();
    if (node.readHoldingRegisters(addr, 1) == node.ku8MBSuccess) return node.getResponseBuffer(0);
    delay(150);
  }
  return 0xFFFF;
}

void loadSettings() {
  preferences.begin("settings", true);
  cfgHeaterPowerVA = preferences.getUShort("heaterVA", 0);
  cfgMaxAllocatedVA = preferences.getUShort("maxAllocVA", 0);
  cfgUpdateIntervalSec = constrain(preferences.getFloat("updIntSec", DEFAULT_UPDATE_INTERVAL_SEC), 0.5f, 10.0f);
  cfgBattOffVoltage = preferences.getFloat("battOffV", 0.0f);
  cfgBattHysteresisV = constrain(preferences.getFloat("battHystV", 0.25f), 0.0f, 5.0f);
  cfgBattOffCurrent = preferences.getFloat("battOffI", 0.0f);
  cfgFullChargeCurrentA = constrain(preferences.getFloat("chgCurA", 0.0f), 0.0f, 100.0f);
  cfgFullChargeVoltage = constrain(preferences.getFloat("fullChgV", 0.0f), 20.0f, 30.0f);
  switch_state = preferences.getBool("swState", false);
  preferences.end();
}

void saveSettings() {
  preferences.begin("settings", false);
  preferences.putUShort("heaterVA", cfgHeaterPowerVA);
  preferences.putUShort("maxAllocVA", cfgMaxAllocatedVA);
  preferences.putFloat("updIntSec", cfgUpdateIntervalSec);
  preferences.putFloat("battOffV", cfgBattOffVoltage);
  preferences.putFloat("battHystV", cfgBattHysteresisV);
  preferences.putFloat("battOffI", cfgBattOffCurrent);
  preferences.putFloat("chgCurA", cfgFullChargeCurrentA);
  preferences.putFloat("fullChgV", cfgFullChargeVoltage);
  preferences.putBool("swState", switch_state);
  preferences.end();
}

void pollModbusOnce() {
  bool ok = true;
  digitalWrite(LED_PIN, HIGH);
  if (!readModbusBlock(25201, 80, invBuf)) ok = false; delay(40);
  if (!readModbusBlock(15201, 20, pvBuf)) ok = false; delay(40);
  if (!ok) return;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(inv.inv, invBuf, sizeof(inv.inv));
  memcpy(inv.pv, pvBuf, sizeof(inv.pv));
  inv.workState = inv.inv[0];
  switch (inv.workState) {
    case 0: inv.stateStr = "POWER ON"; break;
    case 1: inv.stateStr = "SELFTEST"; break;
    case 2: inv.stateStr = "OFF GRID"; break;
    case 3: inv.stateStr = "GRID TIE"; break;
    case 4: inv.stateStr = "BYPASS"; break;
    case 5: inv.stateStr = "STOP"; break;
    case 6: inv.stateStr = "GRID CHRG"; break;
    default: inv.stateStr = "UNKNOWN"; break;
  }

  float pvV   = (inv.pv[4] != 65535) ? inv.pv[4] / 10.0f : 0.0f;
  float pvI   = (inv.pv[6] != 65535) ? inv.pv[6] / 10.0f : 0.0f;
  float invV  = (inv.inv[5] != 65535) ? inv.inv[5] / 10.0f : 0.0f;
  float invI  = (inv.inv[11] != 65535) ? inv.inv[11] / 10.0f : 0.0f;

  if (inv.inv[4] != 0 && inv.inv[4] != 65535) {
    battV = inv.inv[4] / 10.0f;
  } else {
    battV = (inv.pv[5] != 65535) ? inv.pv[5] / 10.0f : 0.0f;
  }

  pvPower = pvV * pvI;
  invPower = (int32_t)round((inv.inv[18] != 65535) ? inv.inv[18] : 0.0);
  freePower = pvPower - invPower;

  battI = (battV > 0) ? ((float)freePower / battV) : 0.0f;
  battPower = battV * battI;

  if (battV < cfgFullChargeVoltage || pvI < cfgFullChargeCurrentA) {
    freePower -= (int32_t)(pvV * cfgFullChargeCurrentA);
  }
  
  static bool isBattLow = false;
  if (cfgBattOffVoltage > 0.0f) {
    if (battV < cfgBattOffVoltage) isBattLow = true;
    else if (battV >= cfgBattOffVoltage + cfgBattHysteresisV) isBattLow = false;
  } else {
    isBattLow = false;
  }

  const char* targMessage = "Все добре";

  if (inv.inv[0] != 2) {
    freePower = 0; currentPower = 0;
    targMessage = "Інвертер споживає з розетки";
  } else if (freePower <= 0) {
    freePower = 0; currentPower = 0;
    targMessage = "Вільна потужність відсутня";
  } else if (isBattLow) {
    freePower = 0; currentPower = 0;
    targMessage = "Напруга акамулятора менше встановленого мінімуму";
  } else if (pvI < cfgBattOffCurrent) {
    freePower = 0; currentPower = 0;
    targMessage = "Струм акамулятора менше встановленого мінімуму";
  } else {
    if (freePower < 0) {
      currentPower -= freePower;
      targMessage = "Недостатньо потужності";
    } else if (freePower == 0) {
      targMessage = "Потужність на максимумі";
    } else {
      if (freePower > 19) {
        currentPower += 20;
        targMessage = "Потужність збільшена на 20 ВА";
      } else {
        currentPower += freePower;
        targMessage = "Потужність збільшено";
      }
    }
    if (currentPower >= cfgMaxAllocatedVA) {
      currentPower = cfgMaxAllocatedVA;
      targMessage = "Потужність обрізано до максимуму.";
    }
  }

  PpowerHeater = currentPower;

  strncpy(targMessageGlobal, targMessage, sizeof(targMessageGlobal) - 1);
  targMessageGlobal[sizeof(targMessageGlobal) - 1] = '\0';

  char logLine[256];
  snprintf(logLine, sizeof(logLine), "[%lu] ws=%d pvV=%.1f pvI=%.1f invV=%.0f invI=%.1f battV=%.1f battI=%.1f battPwr=%.0f pvP=%d invP=%d free=%d cur=%d pph=%d sw=%d msg=%s\n",
           millis(), inv.workState, pvV, pvI, invV, invI, battV, battI, battPower, pvPower, invPower, freePower, currentPower, PpowerHeater, switch_state ? 1 : 0, targMessage);
  serialLog(logLine);

  xSemaphoreGive(dataMutex);
  digitalWrite(LED_PIN, LOW);
}

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
.switch-row{display:flex;align-items:center;justify-content:center;gap:8px;margin-top:6px;padding:6px 10px;background:#161616;border-radius:6px;border:1px solid #2a2a2a}
.switch-btn{background:none;border:none;cursor:pointer;padding:0;display:flex;align-items:center;gap:6px;transition:transform 0.15s}
.switch-btn:hover{transform:scale(1.08)}
.switch-btn:active{transform:scale(0.95)}
.switch-btn svg{width:28px;height:28px;transition:fill 0.3s}
.switch-label{color:#888;font-size:11px;font-weight:bold;text-transform:uppercase;letter-spacing:0.5px}
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
      <div class="panel-header">
        <svg viewBox="0 0 24 24"><path d="M12 7c-2.76 0-5 2.24-5 5s2.24 5 5 5 5-2.24 5-5-2.24-5-5-5zM2 13h2c.55 0 1-.45 1-1s-.45-1-1-1H2c-.55 0-1 .45-1 1s.45 1 1 1zm18 0h2c.55 0 1-.45 1-1s-.45-1-1-1h-2c-.55 0-1 .45-1 1s.45 1 1 1zM11 2v2c0 .55.45 1 1 1s1-.45 1-1V2c0-.55-.45-1-1-1s-1 .45-1 1zm0 18v2c0 .55.45 1 1 1s1-.45 1-1v-2c0-.55-.45-1-1-1s-1 .45-1 1zM5.99 4.58c-.39-.39-1.03-.39-1.41 0-.39.39-.39 1.03 0 1.41l1.06 1.06c.39.39 1.03.39 1.41 0 .39-.39.39-1.03 0-1.41L5.99 4.58zm12.37 12.37c-.39-.39-1.03-.39-1.41 0-.39.39-.39 1.03 0 1.41l1.06 1.06c.39.39 1.03.39 1.41 0 .39-.39.39-1.03 0-1.41l-1.06-1.06zm1.06-10.96c-.39-.39.39-1.03 0-1.41-.39-.39-1.03-.39-1.41 0l-1.06 1.06c-.39.39-.39 1.03 0 1.41.39.39 1.03.39 1.41 0l1.06-1.06zM7.05 18.36c-.39-.39.39-1.03 0-1.41-.39-.39-1.03-.39-1.41 0l-1.06 1.06c-.39.39-.39 1.03 0 1.41.39.39 1.03.39 1.41 0l1.06-1.06z"/></svg>
        SOLAR POWER
      </div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">PV VOLTAGE (15205)</div><div class="value yellow" id="pvV">--</div></div>
        <div class="metric"><div class="label">PV CURRENT (15207)</div><div class="value yellow" id="pvI">--</div></div>
      </div>
      <div class="big-box">
        <div class="label">PV POWER (NOW)</div>
        <div class="value yellow" id="pvPwr">--</div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-header blue">
        <svg viewBox="0 0 24 24"><path d="M13 3h-2v10h2V3zm4.83 2.17l-1.42 1.42C17.99 7.86 19 9.81 19 12c0 3.87-3.13 7-7 7s-7-3.13-7-7c0-2.19 1.01-4.14 2.58-5.42L6.17 5.17C4.23 6.82 3 9.26 3 12c0 4.97 4.03 9 9 9s9-4.03 9-9c0-2.74-1.23-5.18-3.17-6.83z"/></svg>
        INVERTOR POWER
      </div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">INV VOLTAGE (25206)</div><div class="value blue" id="invV">--</div></div>
        <div class="metric"><div class="label">INV CURRENT (25212)</div><div class="value blue" id="invI">--</div></div>
      </div>
      <div class="big-box">
        <div class="label">INV POWER (25219)</div>
        <div class="value red" id="invPwr">--</div>
      </div>
    </div>
  </div>

  <div class="col">
    <div class="panel center-panel">
      <div class="current-power-box">
        <div class="label">CURRENT HEATER POWER</div>
        <div class="value" id="curP">0 VA</div>
      </div>
      <div class="heater-slider-box">
        <div class="label">HEATER LOAD</div>
        <div class="slider-track">
          <div id="hSlider" class="slider-fill" style="width: 0%; background: #00ff88;"></div>
        </div>
        <div id="hSliderText" class="slider-text">0%</div>
      </div>
      <div class="switch-row">
        <span class="switch-label">ПЕРЕМИКАЧ СПОЖИВАННЯ</span>
        <button class="switch-btn" id="switchBtn" onclick="toggleSwitch()">
          <svg id="switchIcon" viewBox="0 0 24 24" fill="#555">
            <path d="M17 7H7c-2.76 0-5 2.24-5 5s2.24 5 5 5h10c2.76 0 5-2.24 5-5s-2.24-5-5-5zm0 8c-1.66 0-3-1.34-3-3s1.34-3 3-3 3 1.34 3 3-1.34 3-3 3z"/>
          </svg>
        </button>
        <span class="switch-label" id="switchStatus" style="color:#888">OFF</span>
      </div>
      <div class="status-text">
        <span class="machine" id="machineInfo">--</span><br>
        <span class="state" id="stateInfo">--</span>
      </div>
      <div class="alert-box" id="alertBox">
        <div class="value" id="targMsg">--</div>
      </div>
      <div class="free-power-box">
        <div class="label">FREE POWER</div>
        <div class="value" id="freeP">0 VA</div>
      </div>
    </div>
  </div>

  <div class="col">
    <div class="panel">
      <div class="panel-header blue">
        <svg viewBox="0 0 24 24"><path d="M15.67 4H14V2h-4v2H8.33C7.6 4 7 4.6 7 5.33v15.33C7 21.4 7.6 22 8.33 22h7.33c.74 0 1.34-.6 1.34-1.33V5.33C17 4.6 16.4 4 15.67 4zM11 20v-5.5H9L13 7v5.5h2L11 20z"/></svg>
        BATTERY
      </div>
      <div class="grid-2x2">
        <div class="metric"><div class="label">BATTERY VOLTAGE</div><div class="value blue" id="battV">--</div></div>
        <div class="metric"><div class="label">BATTERY CURRENT</div><div class="value blue" id="battI">--</div></div>
      </div>
      <div class="big-box">
        <div class="label">BATTERY POWER</div>
        <div class="value" id="battPwr">--</div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-header">
        <svg viewBox="0 0 24 24"><path d="M11 21h-1l1-7H7.5c-.58 0-.57-.32-.38-.66.19-.34.05-.08.07-.12C8.48 10.94 10.42 7.54 13 3h1l-1 7h3.5c.49 0 .56.33.47.51l-.07.15C12.96 17.55 11 21 11 21z"/></svg>
        POWER INFO
      </div>
      <div class="stack">
        <div class="metric"><div class="label">PV ENERGY TOTAL</div><div class="value yellow" id="pvAccum">--</div></div>
        <div class="metric"><div class="label">ACCUM LOAD</div><div class="value red" id="accDis">--</div></div>
        <div class="metric"><div class="label">ACCUM SELF USE</div><div class="value green" id="accChg">--</div></div>
      </div>
    </div>
  </div>
</div>

<div class="controls">
<button class="btn primary" onclick="update()">Refresh</button>
<button class="btn" onclick="location.href='/settings'">Settings</button>
<button class="btn warn" onclick="resetCounters()">Reset counters</button>
<button class="btn" onclick="resetWifi()">Reset WiFi</button>
<button class="btn danger" onclick="restartEsp()">Restart ESP</button>
<button class="btn" onclick="location.href='/serial'">Serial Monitor</button>
<button class="btn" onclick="toggleUpdate()">Firmware Update</button>
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
async function resetCounters(){if(!confirm("Reset all counters?"))return;await fetch("/reset_counters");alert("Done")}
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
try{
let resp=await fetch("/toggle_switch",{method:"POST"});
if(resp.status==403){alert("Access denied: Local network only");return}
let data=await resp.json();
updateSwitchUI(data.state);
}catch(e){alert("Connection error!")}}

function updateSwitchUI(state){
let icon=document.getElementById("switchIcon");
let status=document.getElementById("switchStatus");
if(state){
icon.setAttribute("fill","#00ff88");
status.innerText="ON";
status.style.color="#00ff88";
}else{
icon.setAttribute("fill","#555");
status.innerText="OFF";
status.style.color="#888";
}}

async function update(){
try{
let d=await(await fetch("/api")).json();

document.getElementById("machineInfo").innerText=((d.mt&&d.mt!=="UNKNOWN")?d.mt+" ":"")+((d.mp&&d.mp!=="—")?d.mp:"");
document.getElementById("stateInfo").innerText=wsStr(d.ws)+" | MPPT: "+mpptStr(d.mppt)+" | CHG: "+chgStr(d.chg);

document.getElementById("pvV").innerText=fmt1(d.pvV,"V");
document.getElementById("pvI").innerText=fmt1(d.pvI,"A");
document.getElementById("pvPwr").innerText=fmt0(d.pvPower,"VA");
document.getElementById("pvAccum").innerText=fmtKwh(d.pvEnergyHi,d.pvEnergyLo);

document.getElementById("invV").innerText=fmt1(d.invV,"V");
document.getElementById("invI").innerText=fmt1(d.invI,"A");
document.getElementById("invPwr").innerText=fmt0(d.invPower,"VA");

document.getElementById("battV").innerText=(d.battV!=null&&d.battV>0)?d.battV.toFixed(1)+" V":"--";

let battIEl=document.getElementById("battI");
if(!isBad(d.battI)){
battIEl.innerText=d.battI.toFixed(1)+" A";
battIEl.className="value "+(d.battI>=0?"blue":"red");
}else{
battIEl.innerText="--";
battIEl.className="value blue";
}

let battPwrEl=document.getElementById("battPwr");
if(!isBad(d.battPower)){
battPwrEl.innerText=Math.round(d.battPower)+" VA";
battPwrEl.className="value "+(d.battPower>=0?"blue":"red");
}else{
battPwrEl.innerText="--";
battPwrEl.className="value blue";
}

document.getElementById("accDis").innerText=fmtKwh(d.acDisHi,d.acDisLo);
document.getElementById("accChg").innerText=fmtKwh(d.acChgHi,d.acChgLo);

document.getElementById("curP").innerText=fmt0(d.cp,"VA");
document.getElementById("freeP").innerText=fmt0(d.fp,"VA");

let hpct = d.hpct || 0;
let hSlider = document.getElementById("hSlider");
hSlider.style.width = hpct + "%";
let hColor = hpct < 33 ? "#00ff88" : (hpct < 66 ? "#ffd166" : "#ff4d4d");
hSlider.style.background = hColor;
document.getElementById("hSliderText").innerText = hpct + "%";
document.getElementById("hSliderText").style.color = hColor;

updateSwitchUI(d.sw);

let tm=d.tm||"--";
let tmEl=document.getElementById("targMsg");
let box=document.getElementById("alertBox");
tmEl.innerText=tm;
if(tm==="Все добре"){
tmEl.style.color="#00ff88";
box.style.borderColor="#00ff88";
box.style.background="rgba(0,255,136,0.05)";
}else{
tmEl.style.color="#ff4d4d";
box.style.borderColor="#ff4d4d";
box.style.background="rgba(255,77,77,0.05)";
}
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
.bg{background:#555;color:#fff}
.ctr{text-align:center;margin:10px 0}
</style></head><body>
<h1>Serial Monitor</h1>
<div class="ctr">
<button class="btn" onclick="fetchLog()">Refresh</button>
<button class="btn bg" onclick="document.getElementById('log').innerText=''">Clear</button>
<button class="btn bg" onclick="autoScroll=!autoScroll;this.innerText=autoScroll?'Auto-scroll: ON':'Auto-scroll: OFF'">Auto-scroll: ON</button>
<button class="btn bg" onclick="location.href='/'">Back</button>
</div>
<div id="log">Loading...</div>
<script>
let autoScroll=true;
async function fetchLog(){
try{let r=await fetch("/api_serial");let t=await r.text();
let el=document.getElementById("log");
el.innerText=t;
if(autoScroll)el.scrollTop=el.scrollHeight;
}catch(e){document.getElementById("log").innerText="Error: "+e}}
fetchLog();
setInterval(fetchLog,2000);
</script></body></html>
)rawliteral";

const char SETTINGS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Settings</title>
<style>
body{background:#0f0f0f;color:#e6e6e6;font-family:monospace;padding:15px;margin:0}
h1{color:#00ff88;text-align:center;font-size:18px}
h2{color:#4da3ff;font-size:14px;margin-top:20px}
.card{background:#1c1c1c;border-radius:10px;padding:15px;margin:10px 0}
label{color:#888;font-size:12px;display:block;margin-top:10px}
.desc{color:#666;font-size:10px;margin:2px 0 6px 0;line-height:1.3}
input{width:100%;box-sizing:border-box;padding:10px;margin:5px 0;background:#333;color:#fff;border:1px solid #555;border-radius:6px;font-size:14px}
.btn{background:#00ff88;color:#000;border:none;padding:10px 20px;border-radius:6px;cursor:pointer;font-weight:bold;margin:5px}
.bg{background:#555;color:#fff}
.msg{color:#00ff88;text-align:center;margin:10px 0;display:none}
.err{color:#ff4d4d;text-align:center;margin:10px 0;display:none}
</style></head><body>
<h1>Settings</h1>
<div class="msg" id="msg">Saved!</div>
<div class="err" id="err">Access denied: Local network only</div>
<h2>General</h2>
<div class="card">
<label>Heater power (VA)</label>
<div class="desc">Номінальна потужність вашого приладу (навантаження), наприклад ТЭНа, у вольт-амперах (ВА). Це базове значення, яке використовується системою для розрахунку допустимої потужності. Вказуйте потужність приладу, яка зазначена на шильдику або в паспорті. Якщо встановлено 0, система не використовує це обмеження.</div>
<input type="number" id="heaterVA" min="0" max="99999" value="0">
<label>Max allocated power (VA)</label>
<div class="desc">Максимальна потужність у ВА, яку система може виділити на ваш прилад. Це жорстке обмеження, яке запобігає перевантаженню інвертора або проводки. Навіть якщо вільна потужність від сонячних панелей вища, потужність приладу не перевищить це значення. Рекомендується встановлювати не більше номінальної потужності інвертора та потужності приладу. Якщо встановлено 0, обмеження не діє.</div>
<input type="number" id="maxAllocVA" min="0" max="99999" value="0">
<label>Update interval (sec)</label>
<div class="desc">Інтервал опитування інвертора по Modbus у секундах. Визначає, як часто система зчитує дані про напругу, струм та потужність. Занадто малий інтервал (менше 1 сек) може перевантажити інтерфейс або інвертор, занадто великий призведе до запізнень у реакції на зміни. Рекомендований діапазон: 1-3 сек. За замовчуванням 2 сек.</div>
<input type="number" id="updIntSec" min="0.5" max="10" step="0.5" value="2">
</div>
<h2>Load shutdown</h2>
<div class="card">
<label>Shutdown voltage (V)</label>
<div class="desc">Критична напруга акумулятора, при якій навантаження примусово вимикається для захисту акумулятора від глибокого розряду. Вимірюється на клемах акумулятора. Рекомендується встановлювати на рівні напруги, нижче якої розряд акумулятора стає небезпечним. Наприклад, для LiFePO4 12В це ~11.5-12.0В, для AGM/GEL ~11.8-12.2В. Також враховуйте налаштування інвертора (зазвичай Floating voltage +0.2-0.5 В). Якщо встановлено 0, захист по напрузі вимкнено.</div>
<input type="number" id="battOffV" min="0" max="99.9" step="0.1" value="0">
<label>Hysteresis (V)</label>
<div class="desc">Гістерезис напруги (В). Різниця між напругою відключення і напругою повторного включення навантаження. Запобігає частому перемиканню (дребезгу) реле при коливаннях напруги. Наприклад, якщо встановлено 22.0 В і гістерезис 0.3 В, прилад вимкнеться при 22.0 В, а увімкнеться назад тільки коли напруга підніметься до 22.3 В. Рекомендовано 0.2 - 0.5 В. Якщо встановлено 0, гістерезис відсутній (може призвести до частих перемикань).</div>
<input type="number" id="battHystV" min="0" max="5" step="0.1" value="0.25">
<label>Shutdown current (A)</label>
<div class="desc">Мінімальний струм акумулятора, при якому дозволяється робота навантаження. Якщо струм акумулятора нижчий за це значення (наприклад, сонячні панелі не дають достатньо енергії або акумулятор розряджений), прилад вимикається. Може використовуватися для захисту від роботи при дуже низькому зарядному струмі. Якщо встановлено 0, захист по струму вимкнено.</div>
<input type="number" id="battOffI" min="0" max="999.9" step="0.1" value="0">
</div>
<h2>Battery charging</h2>
<div class="card">
<label>Charge current (A)</label>
<div class="desc">Струм, який резервується для зарядки акумулятора під час роботи приладу. Система віднімає цю потужність (напруга акумулятора × зарядний струм) від вільної потужності сонячних панелей, щоб акумулятор отримував заряд. Наприклад, якщо вільна потужність 500 ВА, а зарядний струм 10 А при напрузі 24 В, на прилад піде лише 500 - 240 = 260 ВА. Якщо встановлено 0, вся вільна потужність іде на прилад, акумулятор не заряджається.</div>
<input type="number" id="chgCurA" min="0" max="100" step="1" value="0">
<label>Full charge voltage (V)</label>
<div class="desc">Напруга, при якій акумулятор вважається повністю зарядженим. Якщо напруга акумулятора вища за це значення, система перестає резервувати потужність на зарядку і вся вільна енергія йде на ваш прилад. Це запобігає перезаряду акумулятора. Встановлюйте відповідно до типу акумулятора: для LiFePO4 12В ~14.2-14.6 В, для AGM ~14.4-14.8 В, для GEL ~14.1-14.4 В. Працює в парі з параметром «Струм зарядки». Якщо встановлено 0, функція не діє.</div>
<input type="number" id="fullChgV" min="20" max="30" step="0.1" value="0">
</div>
<div style="text-align:center;margin-top:15px">
<button class="btn" onclick="save()">Save</button>
<button class="btn bg" onclick="location.href='/'">Back</button>
</div>
<script>
async function load(){try{let d=await(await fetch("/api_settings")).json();
["heaterVA","maxAllocVA","updIntSec","battOffV","battHystV","battOffI","chgCurA","fullChgV"].forEach(k=>document.getElementById(k).value=d[k])}catch(e){}}
async function save(){
let body=["heaterVA","maxAllocVA","updIntSec","battOffV","battHystV","battOffI","chgCurA","fullChgV"].map(k=>k+"="+document.getElementById(k).value).join("&");
try{
let resp=await fetch("/save_settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body});
if(resp.status==200){
document.getElementById("msg").style.display="block";
document.getElementById("err").style.display="none";
setTimeout(()=>document.getElementById("msg").style.display="none",2000);
}else if(resp.status==403){
document.getElementById("err").style.display="block";
document.getElementById("msg").style.display="none";
setTimeout(()=>document.getElementById("err").style.display="none",3000);
}else{
alert("Save error!");
}
}catch(e){
alert("Connection error!");
}
}
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

void triggerReboot() { needRestart = true; restartRequestedMs = millis(); }

void startSetupAP() {
  setupMode = true;
  WiFi.mode(WIFI_AP); WiFi.softAP("Inverter-Setup");
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) { r->send_P(200, "text/html", SETUP_HTML); });
  server.on("/scan_wifi", HTTP_GET, [](AsyncWebServerRequest* r) {
    int n = WiFi.scanNetworks();
    auto* res = r->beginResponseStream("application/json");
    res->print("[");
    for (int i = 0; i < n; i++) { if (i) res->print(","); res->printf("{\"ssid\":\"%s\",\"rssi\":%d}", WiFi.SSID(i).c_str(), WiFi.RSSI(i)); }
    res->print("]"); WiFi.scanDelete(); r->send(res);
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

void onUpdateUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
  if (!index) {
    if (!request->client() || !isLocalIP(request->client()->remoteIP())) {
      request->send(403, "text/plain", "Access denied");
      Update.abort();
      return;
    }
    webUpdateActive = true; 
    if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) Update.printError(Serial); 
  }
  if (!Update.hasError() && Update.write(data, len) != len) Update.printError(Serial);
  if (final) { 
    if (Update.end(true)) Serial.printf("Update OK: %u bytes\n", (unsigned)(index + len)); 
    else { Update.printError(Serial); webUpdateActive = false; } 
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) { 
    r->send_P(200, "text/html", INDEX_HTML); 
  });

  server.on("/api", HTTP_GET, [](AsyncWebServerRequest *r) {
    bool locked = dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE;

    float api_pvV = (inv.pv[4] != 65535) ? inv.pv[4] / 10.0f : 0.0f;
    float api_pvI = (inv.pv[6] != 65535) ? inv.pv[6] / 10.0f : 0.0f;
    float api_invV = (inv.inv[5] != 65535) ? inv.inv[5] / 10.0f : 0.0f;
    float api_invI = (inv.inv[11] != 65535) ? inv.inv[11] / 10.0f : 0.0f;

    int32_t api_pvPower = (int32_t)round((inv.pv[7] != 65535) ? (float)inv.pv[7] : 0.0f);
    int32_t api_invPower = (int32_t)round((inv.inv[18] != 65535) ? (float)inv.inv[18] : 0.0f);

    int heaterPercent = (cfgHeaterPowerVA > 0) ? (PpowerHeater * 100 / cfgHeaterPowerVA) : 0;
    if (heaterPercent > 100) heaterPercent = 100;
    if (heaterPercent < 0) heaterPercent = 0;

    auto* res = r->beginResponseStream("application/json; charset=utf-8");

    res->printf(
      "{\"ws\":%d,\"mt\":\"%s\",\"mp\":\"%s\",\"fp\":%d,\"cp\":%d,\"pph\":%d,\"hpct\":%d,\"hva\":%u,\"maVA\":%u,\"sw\":%d,\"tm\":\"%s\",",
      inv.workState,
      inv.machineType.c_str(),
      inv.machinePower.c_str(),
      (int)freePower,
      (int)currentPower,
      (int)PpowerHeater,
      heaterPercent,
      (unsigned)cfgHeaterPowerVA,
      (unsigned)cfgMaxAllocatedVA,
      switch_state ? 1 : 0,
      targMessageGlobal
    );

    res->printf("\"pvV\":%.1f,\"pvI\":%.1f,\"pvPower\":%d,", api_pvV, api_pvI, (int)api_pvPower);
    res->printf("\"invV\":%.1f,\"invI\":%.1f,\"invPower\":%d,", api_invV, api_invI, (int)api_invPower);
    res->printf("\"battV\":%.1f,\"battI\":%.1f,\"battPower\":%.1f,", battV, battI, battPower);
    res->printf("\"pvEnergyHi\":%u,\"pvEnergyLo\":%u,", (unsigned)inv.pv[16], (unsigned)inv.pv[17]);
    res->printf("\"acDisHi\":%u,\"acDisLo\":%u,", (unsigned)inv.inv[52], (unsigned)inv.inv[53]);
    res->printf("\"acChgHi\":%u,\"acChgLo\":%u,", (unsigned)inv.inv[54], (unsigned)inv.inv[55]);
    res->printf("\"pvWork\":%u,\"mppt\":%u,\"chg\":%u}", (unsigned)inv.pv[0], (unsigned)inv.pv[1], (unsigned)inv.pv[2]);

    r->send(res);
    if (locked) xSemaphoreGive(dataMutex);
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *r) { 
    r->send_P(200, "text/html", SETTINGS_HTML); 
  });

  server.on("/api_settings", HTTP_GET, [](AsyncWebServerRequest *r) {
    auto* res = r->beginResponseStream("application/json");
    res->printf("{\"heaterVA\":%u,\"maxAllocVA\":%u,\"updIntSec\":%.1f,\"battOffV\":%.1f,\"battHystV\":%.1f,\"battOffI\":%.1f,\"chgCurA\":%.1f,\"fullChgV\":%.1f}",
                cfgHeaterPowerVA, cfgMaxAllocatedVA, cfgUpdateIntervalSec, cfgBattOffVoltage, cfgBattHysteresisV, cfgBattOffCurrent, cfgFullChargeCurrentA, cfgFullChargeVoltage);
    r->send(res);
  });

  server.on("/serial", HTTP_GET, [](AsyncWebServerRequest *r) { 
    r->send_P(200, "text/html", SERIAL_HTML); 
  });

  server.on("/api_serial", HTTP_GET, [](AsyncWebServerRequest *r) {
    char tmp[SERIAL_BUF_SIZE];
    portENTER_CRITICAL(&serialMux);
    uint16_t h = serialHead, t = serialTail;
    portEXIT_CRITICAL(&serialMux);
    uint16_t len = 0;
    while (t != h && len < SERIAL_BUF_SIZE - 1) {
      tmp[len++] = serialBuf[t];
      t = (t + 1) % SERIAL_BUF_SIZE;
    }
    tmp[len] = '\0';
    r->send(200, "text/plain; charset=utf-8", tmp);
  });

  server.on("/save_settings", HTTP_POST, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      if (req->hasParam("heaterVA", true)) cfgHeaterPowerVA = req->getParam("heaterVA", true)->value().toInt();
      if (req->hasParam("maxAllocVA", true)) cfgMaxAllocatedVA = req->getParam("maxAllocVA", true)->value().toInt();
      if (req->hasParam("updIntSec", true)) cfgUpdateIntervalSec = constrain(req->getParam("updIntSec", true)->value().toFloat(), 0.5f, 10.0f);
      if (req->hasParam("battOffV", true)) cfgBattOffVoltage = req->getParam("battOffV", true)->value().toFloat();
      if (req->hasParam("battHystV", true)) cfgBattHysteresisV = constrain(req->getParam("battHystV", true)->value().toFloat(), 0.0f, 5.0f);
      if (req->hasParam("battOffI", true)) cfgBattOffCurrent = req->getParam("battOffI", true)->value().toFloat();
      if (req->hasParam("chgCurA", true)) cfgFullChargeCurrentA = constrain(req->getParam("chgCurA", true)->value().toFloat(), 0.0f, 100.0f);
      if (req->hasParam("fullChgV", true)) cfgFullChargeVoltage = constrain(req->getParam("fullChgV", true)->value().toFloat(), 20.0f, 30.0f);
      saveSettings(); req->send(200, "text/plain", "OK");
    });
  });

  server.on("/toggle_switch", HTTP_POST, [](AsyncWebServerRequest *r) {
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      switch_state = !switch_state;
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
    requireLocalIP(r, [](AsyncWebServerRequest *req) {
      if (Update.hasError()) { webUpdateActive = false; req->send(500, "text/plain", "FAIL"); }
      else { req->send(200, "text/plain", "OK"); triggerReboot(); }
    });
  }, onUpdateUpload);

  server.onNotFound([](AsyncWebServerRequest *r) { 
    r->redirect("/"); 
  });
  
  server.begin();
}

void setup() {
  Serial.begin(115200);
  pinMode(RS485_CTRL, OUTPUT); digitalWrite(RS485_CTRL, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  WiFi.persistent(false);
  preferences.begin("wifi_config", true);
  wifiSsid = preferences.getString("ssid", "");
  wifiPassword = preferences.getString("password", "");
  preferences.end();
  loadSettings();
  if (wifiSsid.length()) {
    WiFi.mode(WIFI_STA); WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < WIFI_CONNECT_TIMEOUT_MS) delay(300);
  }
  if (WiFi.status() != WL_CONNECTED) { startSetupAP(); return; }
  MDNS.begin("inverter");
  Serial1.begin(19200, SERIAL_8N1, RX2_PIN, TX2_PIN);
  node.begin(SLAVE_ID, Serial1);
  node.preTransmission(preTrans); node.postTransmission(postTrans);
  delay(1500);
  uint16_t r0 = readSingleRegisterWithRetry(20000), r1 = readSingleRegisterWithRetry(20001);
  if (r0 != 0xFFFF && r1 != 0xFFFF) {
    inv.machineType = String((char)((r0 >> 8) & 0xFF)) + String((char)(r0 & 0xFF));
    inv.machinePower = (r1 == 1800 || r1 == 3000) ? String(r1) : "—";
  } else { inv.machineType = "UNKNOWN"; inv.machinePower = "—"; }
  dataMutex = xSemaphoreCreateMutex();
  setupWebServer(); 
  lastUpdateMs = millis();
  serialLog("=== System started ===\n");
}

void loop() {
  unsigned long now = millis();
  if (needRestart && now - restartRequestedMs >= RESTART_DELAY_MS) ESP.restart();
  if (setupMode) { dnsServer.processNextRequest(); return; }
  if (WiFi.status() != WL_CONNECTED && now - lastWifiReconnectMs >= WIFI_RECONNECT_INTERVAL_MS) { lastWifiReconnectMs = now; WiFi.reconnect(); }
  
  if (pendingCounterReset) { pendingCounterReset = false; node.writeSingleRegister(20213, 1); delay(100); node.writeSingleRegister(10112, 1); delay(100); }
  if (!webUpdateActive && now - lastUpdateMs >= (uint32_t)(cfgUpdateIntervalSec * 1000.0f)) { lastUpdateMs = now; pollModbusOnce(); }
}
