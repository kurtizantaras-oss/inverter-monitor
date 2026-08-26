#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ModbusMaster.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <Update.h>

#define RX2_PIN 16
#define TX2_PIN 17
#define RS485_CTRL 4
#define SLAVE_ID 0x04
#define LED_PIN 2
#define SKETCH_VERSION "6.3.6"
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
float cfgBattOffCurrent = 0.0f;
float cfgFullChargeCurrentA = 0.0f;
float cfgFullChargeVoltage = 0.0f;

int32_t freePower = 0;
int32_t currentPower = 0;
int32_t pvPower = 0;
int32_t invPower = 0;

static char targMessageGlobal[64] = "--";

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

void preTrans() { digitalWrite(RS485_CTRL, HIGH); delayMicroseconds(100); }
void postTrans() { delayMicroseconds(100); Serial1.flush(); digitalWrite(RS485_CTRL, LOW); }

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
  cfgBattOffCurrent = preferences.getFloat("battOffI", 0.0f);
  cfgFullChargeCurrentA = constrain(preferences.getFloat("chgCurA", 0.0f), 0.0f, 100.0f);
  cfgFullChargeVoltage = constrain(preferences.getFloat("fullChgV", 0.0f), 20.0f, 30.0f);
  preferences.end();
}

void saveSettings() {
  preferences.begin("settings", false);
  preferences.putUShort("heaterVA", cfgHeaterPowerVA);
  preferences.putUShort("maxAllocVA", cfgMaxAllocatedVA);
  preferences.putFloat("updIntSec", cfgUpdateIntervalSec);
  preferences.putFloat("battOffV", cfgBattOffVoltage);
  preferences.putFloat("battOffI", cfgBattOffCurrent);
  preferences.putFloat("chgCurA", cfgFullChargeCurrentA);
  preferences.putFloat("fullChgV", cfgFullChargeVoltage);
  preferences.end();
}

void pollModbusOnce() {
  bool ok = true;
  digitalWrite(LED_PIN, HIGH);
  if (!readModbusBlock(25201, 80, invBuf)) ok = false; delay(40);
  if (!readModbusBlock(15201, 20, pvBuf)) ok = false; delay(40);
  if (!ok) {
    return;
  }

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
  float invV  = (inv.inv[1] != 65535) ? (float)inv.inv[1] : 0.0f;
  float invI  = (inv.inv[11] != 65535) ? inv.inv[11] / 10.0f : 0.0f;

  float battV;
  if (inv.inv[4] != 0 && inv.inv[4] != 65535) {
    battV = inv.inv[4] / 10.0f;
  } else {
    battV = (inv.pv[5] != 65535) ? inv.pv[5] / 10.0f : 0.0f;
  }

  pvPower = (int32_t)round(pvV * pvI);
  invPower = (int32_t)round(invV * invI);
  freePower = pvPower - invPower;

  const char* targMessage = "Все добре";

  if (inv.inv[0] != 2) {
    freePower = 0;
    currentPower = 0;
    targMessage = "Інвертер споживає з розетки";
  } else if (freePower <= 0) {
    freePower = 0;
    currentPower = 0;
    targMessage = "Вільна потужність відсутня";
  } else if (battV < cfgBattOffVoltage) {
    freePower = 0;
    currentPower = 0;
    targMessage = "Напруга акамулятора менше встановленого мінімуму";
  } else if (pvI < cfgBattOffCurrent) {
    freePower = 0;
    currentPower = 0;
    targMessage = "Струм акамулятора менше встановленого мінімуму";
  } else {
    // otnimaem zapas zaryadki na  akb
    if (battV < cfgFullChargeVoltage || pvI < cfgFullChargeCurrentA) {
      freePower -= (int32_t)(pvV * cfgFullChargeCurrentA);
    }
    if (freePower < 0) {
      currentPower -= freePower;
      targMessage = "Недостатньо потужності";
    } else if (freePower == 0) {
      targMessage = "Потужність на максимумі";
    // esli freepower > 0 
    } else {
      if (freePower  > 19) {
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

  strncpy(targMessageGlobal, targMessage, sizeof(targMessageGlobal) - 1);
  targMessageGlobal[sizeof(targMessageGlobal) - 1] = '\0';

  char logLine[200];
  snprintf(logLine, sizeof(logLine), "[%lu] ws=%d pvV=%.1f pvI=%.1f invV=%.0f invI=%.1f battV=%.1f pvP=%d invP=%d free=%d cur=%d msg=%s\n",
           millis(), inv.workState, pvV, pvI, invV, invI, battV, pvPower, invPower, freePower, currentPower, targMessage);
  serialLog(logLine);

  xSemaphoreGive(dataMutex);
  digitalWrite(LED_PIN, LOW);
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Inverter</title>
<style>
body{background:#0f0f0f;color:#e6e6e6;font-family:monospace;padding:15px;margin:0}
h1{color:#00ff88;text-align:center;font-size:18px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin:15px 0}
.card{background:#1c1c1c;border-radius:10px;padding:12px;text-align:center}
.lb{color:#888;font-size:11px;margin-bottom:6px}
.vl{font-size:20px;font-weight:bold;white-space:pre-line}
.vl.g{color:#00ff88}.vl.b{color:#4da3ff}.vl.r{color:#ff4d4d}.vl.y{color:#ffd166}
.btn{background:#00ff88;color:#000;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-weight:bold;margin:3px}
.bw{background:#ffd166}.bg{background:#555;color:#fff}.br{background:#ff4d4d;color:#fff}
.ctr{text-align:center;margin:10px 0}
.st{text-align:center;color:#4da3ff;font-size:14px;margin:10px 0}
.full-width{grid-column:1/-1}
</style></head><body>
<h1>Inverter Monitor</h1>
<div class="st" id="state">--</div>

<div class="grid">
<div class="card"><div class="lb">BATT VOLTAGE</div><div class="vl y" id="battV">--</div></div>
<div class="card"><div class="lb">PV VOLTAGE (15205)</div><div class="vl y" id="pvV">--</div></div>
<div class="card"><div class="lb">PV CURRENT (15207)</div><div class="vl g" id="pvI">--</div></div>
<div class="card"><div class="lb">PV POWER</div><div class="vl y" id="pvPwr">--</div></div>
<div class="card"><div class="lb">INV VOLTAGE (25202)</div><div class="vl b" id="invV">--</div></div>
<div class="card"><div class="lb">INV CURRENT (25212)</div><div class="vl b" id="invI">--</div></div>
<div class="card"><div class="lb">INV POWER</div><div class="vl r" id="invPwr">--</div></div>
<div class="card"><div class="lb">FREE POWER</div><div class="vl y" id="freeP">--</div></div>
<div class="card"><div class="lb">CURRENT POWER</div><div class="vl b" id="curP">--</div></div>
<div class="card"><div class="lb">Solar gen.</div><div class="vl y" id="pvAccum">--</div></div>
<div class="card"><div class="lb">Accum. disch.</div><div class="vl r" id="accDis">--</div></div>
<div class="card full-width"><div class="lb">Heater shutdown reason</div><div class="vl" id="targMsg">--</div></div>
</div>

<div class="ctr">
<button class="btn" onclick="update()">Refresh</button>
<button class="btn" onclick="location.href='/settings'">Settings</button>
<button class="btn bw" onclick="resetCounters()">Reset counters</button>
<button class="btn bg" onclick="resetWifi()">Reset WiFi</button>
<button class="btn br" onclick="restartEsp()">Restart ESP</button>
</div>
<div class="ctr">
<button class="btn bg" onclick="location.href='/serial'">Serial Monitor</button>
</div>
<div style="background:#1c1c1c;padding:12px;border-radius:8px;margin-top:15px">
<b style="color:#00ff88">FIRMWARE UPDATE</b><br><br>
<input type="file" id="fwfile" accept=".bin" style="width:100%;padding:8px;background:#333;color:#fff;border:1px solid #555;border-radius:6px;box-sizing:border-box">
<button class="btn" onclick="doUpdate()" style="width:100%;margin-top:8px">Update firmware</button>
<div id="prog" style="height:6px;background:#333;border-radius:3px;margin-top:8px;overflow:hidden;display:none"><div id="pbar" style="height:100%;width:0;background:#00ff88"></div></div>
<div id="ust" style="font-size:11px;color:#888;margin-top:5px">Select .bin file</div>
</div>
<script>
function wsStr(v){switch(v){case 0:return"POWER ON";case 1:return"SELFTEST";case 2:return"OFF GRID";case 3:return"GRID TIE";case 4:return"BYPASS";case 5:return"STOP";case 6:return"GRID CHRG";default:return"UNKNOWN"}}
function mpptStr(v){switch(v){case 0:return"Stop";case 1:return"MPPT";case 2:return"Current limit";default:return"--"}}
function chgStr(v){switch(v){case 0:return"Stop";case 1:return"Charging";case 2:return"Float";default:return"--"}}
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
async function update(){
try{let d=await(await fetch("/api")).json();
document.getElementById("state").innerText="State: "+wsStr(d.ws)+" | "+d.mt+" "+d.mp+" | MPPT: "+mpptStr(d.pv[0])+" | CHG: "+chgStr(d.pv[1]);
let rawInvV=d.inv[2],rawInvI=d.inv[3],rawPvV=d.pv[4],rawPvI=d.pv[6];
let invV=(rawInvV!=null&&rawInvV!==65535)?String(rawInvV):"--";
let invI=(rawInvI!=null&&rawInvI!==65535)?(rawInvI/10).toFixed(1):"--";
let pvV=(rawPvV!=null&&rawPvV!==65535)?(rawPvV/10).toFixed(1):"--";
let pvI=(rawPvI!=null&&rawPvI!==65535)?(rawPvI/10).toFixed(1):"--";
let pvP=(pvV!=="--"&&pvI!=="--")?Math.round(rawPvV/10*rawPvI/10):"--";
let invP=(invV!=="--"&&invI!=="--")?Math.round(rawInvV*rawInvI/10):"--";
document.getElementById("pvV").innerText=pvV==="--"?"--":pvV+" V";
document.getElementById("pvI").innerText=pvI==="--"?"--":pvI+" A";
document.getElementById("pvPwr").innerText=pvP==="--"?"--":pvP+" VA";
document.getElementById("invV").innerText=invV==="--"?"--":invV+" V";
document.getElementById("invI").innerText=invI==="--"?"--":invI+" A";
document.getElementById("invPwr").innerText=invP==="--"?"--":invP+" VA";
document.getElementById("freeP").innerText=d.fp+" VA";
document.getElementById("curP").innerText=d.cp+" VA";

let rawBv=d.bv;
let rawPvBatt=d.pv[5];
let battV;
if(rawBv!=null&&rawBv!==0&&rawBv!==65535){
  battV=(rawBv/10).toFixed(1);
}else if(rawPvBatt!=null&&rawPvBatt!==65535){
  battV=(rawPvBatt/10).toFixed(1);
}else{
  battV="--";
}
document.getElementById("battV").innerText=battV==="--"?"--":battV+" V";

let pvHi=d.pv[7],pvLo=d.pv[8];
if(pvHi!=null&&pvHi!==65535&&pvLo!=null&&pvLo!==65535){
  document.getElementById("pvAccum").innerText=(pvHi*1000+pvLo/10).toFixed(1)+" kWh";
}else{document.getElementById("pvAccum").innerText="--"}

let acHi=d.inv[4],acLo=d.inv[5];
if(acHi!=null&&acHi!==65535&&acLo!=null&&acLo!==65535){
  document.getElementById("accDis").innerText=(acHi*1000+acLo/10).toFixed(1)+" kWh";
}else{document.getElementById("accDis").innerText="--"}

let tm=d.tm||"--";
let tmEl=document.getElementById("targMsg");
tmEl.innerText=tm;
tmEl.className="vl "+(tm==="Все добре"?"g":"r");
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
</style></head><body>
<h1>Settings</h1>
<div class="msg" id="msg">Saved!</div>
<h2>General</h2>
<div class="card">
<label>Heater power (VA)</label>
<div class="desc">Потужність приладу, що використовується</div>
<input type="number" id="heaterVA" min="0" max="99999" value="0">
<label>Max allocated power (VA)</label>
<div class="desc">Максимальна потужність, яку можна виділити, але не більше ніж потужність інвертора</div>
<input type="number" id="maxAllocVA" min="0" max="99999" value="0">
<label>Update interval (sec)</label>
<div class="desc">Період оновлення даних з інвертора, рекомендується 1 сек.</div>
<input type="number" id="updIntSec" min="0.5" max="10" step="0.5" value="2">
</div>
<h2>Load shutdown</h2>
<div class="card">
<label>Shutdown voltage (V)</label>
<div class="desc">Напруга акамулятора нижче якої вимикається ваш прилад (Налаштування інвертора Floating voltage +0.2-0.5 V)</div>
<input type="number" id="battOffV" min="0" max="99.9" step="0.1" value="0">
<label>Shutdown current (A)</label>
<div class="desc">Струм на акамуляторі нижче якого вимикається ваш прилад</div>
<input type="number" id="battOffI" min="0" max="999.9" step="0.1" value="0">
</div>
<h2>Battery charging</h2>
<div class="card">
<label>Charge current (A)</label>
<div class="desc">Струм зарядки акамулятора при роботі вашого приладу</div>
<input type="number" id="chgCurA" min="0" max="100" step="1" value="0">
<label>Full charge voltage (V)</label>
<div class="desc">Максимальна напруга зарядки акамулятора, якщо напруга вище, то зарядка акамулятора зупиняється (все буде йти на ваш прилад)</div>
<input type="number" id="fullChgV" min="20" max="30" step="0.1" value="0">
</div>
<div style="text-align:center;margin-top:15px">
<button class="btn" onclick="save()">Save</button>
<button class="btn bg" onclick="location.href='/'">Back</button>
</div>
<script>
async function load(){try{let d=await(await fetch("/api_settings")).json();
["heaterVA","maxAllocVA","updIntSec","battOffV","battOffI","chgCurA","fullChgV"].forEach(k=>document.getElementById(k).value=d[k])}catch(e){}}
async function save(){
let body=["heaterVA","maxAllocVA","updIntSec","battOffV","battOffI","chgCurA","fullChgV"].map(k=>k+"="+document.getElementById(k).value).join("&");
if((await fetch("/save_settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body})).status==200){
document.getElementById("msg").style.display="block";setTimeout(()=>document.getElementById("msg").style.display="none",2000)}
else alert("Save error!")}
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

void onUpdateUpload(AsyncWebServerRequest*, String filename, size_t index, uint8_t* data, size_t len, bool final) {
  if (!index) { webUpdateActive = true; if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) Update.printError(Serial); }
  if (!Update.hasError() && Update.write(data, len) != len) Update.printError(Serial);
  if (final) { if (Update.end(true)) Serial.printf("Update OK: %u bytes\n", (unsigned)(index + len)); else { Update.printError(Serial); webUpdateActive = false; } }
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) { r->send_P(200, "text/html", INDEX_HTML); });
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest* r) { r->send_P(200, "text/html", SETTINGS_HTML); });
  server.on("/serial", HTTP_GET, [](AsyncWebServerRequest* r) { r->send_P(200, "text/html", SERIAL_HTML); });
  server.on("/api_serial", HTTP_GET, [](AsyncWebServerRequest* r) {
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
  server.on("/api_settings", HTTP_GET, [](AsyncWebServerRequest* r) {
    auto* res = r->beginResponseStream("application/json");
    res->printf("{\"heaterVA\":%u,\"maxAllocVA\":%u,\"updIntSec\":%.1f,\"battOffV\":%.1f,\"battOffI\":%.1f,\"chgCurA\":%.1f,\"fullChgV\":%.1f}",
                cfgHeaterPowerVA, cfgMaxAllocatedVA, cfgUpdateIntervalSec, cfgBattOffVoltage, cfgBattOffCurrent, cfgFullChargeCurrentA, cfgFullChargeVoltage);
    r->send(res);
  });
  server.on("/save_settings", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (r->hasParam("heaterVA", true)) cfgHeaterPowerVA = r->getParam("heaterVA", true)->value().toInt();
    if (r->hasParam("maxAllocVA", true)) cfgMaxAllocatedVA = r->getParam("maxAllocVA", true)->value().toInt();
    if (r->hasParam("updIntSec", true)) cfgUpdateIntervalSec = constrain(r->getParam("updIntSec", true)->value().toFloat(), 0.5f, 10.0f);
    if (r->hasParam("battOffV", true)) cfgBattOffVoltage = r->getParam("battOffV", true)->value().toFloat();
    if (r->hasParam("battOffI", true)) cfgBattOffCurrent = r->getParam("battOffI", true)->value().toFloat();
    if (r->hasParam("chgCurA", true)) cfgFullChargeCurrentA = constrain(r->getParam("chgCurA", true)->value().toFloat(), 0.0f, 100.0f);
    if (r->hasParam("fullChgV", true)) cfgFullChargeVoltage = constrain(r->getParam("fullChgV", true)->value().toFloat(), 20.0f, 30.0f);
    saveSettings(); r->send(200, "text/plain", "OK");
  });
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest* r) { r->send(200, "text/plain", "OK"); triggerReboot(); });
  server.on("/reset_wifi", HTTP_GET, [](AsyncWebServerRequest* r) { preferences.begin("wifi_config", false); preferences.clear(); preferences.end(); r->send(200, "text/plain", "OK"); triggerReboot(); });
  server.on("/reset_counters", HTTP_GET, [](AsyncWebServerRequest* r) { pendingCounterReset = true; r->send(200, "text/plain", "OK"); });
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (Update.hasError()) { webUpdateActive = false; r->send(500, "text/plain", "FAIL"); }
    else { r->send(200, "text/plain", "OK"); triggerReboot(); }
  }, onUpdateUpload);
  server.on("/api", HTTP_GET, [](AsyncWebServerRequest* r) {
    bool locked = dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE;
    auto* res = r->beginResponseStream("application/json");
    res->printf("{\"ws\":%d,\"mt\":\"%s\",\"mp\":\"%s\",\"fp\":%d,\"cp\":%d,\"maVA\":%u,\"tm\":\"%s\","
                "\"bv\":%u,"
                "\"inv\":[%u,%u,%u,%u,%u,%u],"
                "\"pv\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]}",
                inv.workState, inv.machineType.c_str(), inv.machinePower.c_str(), freePower, currentPower, cfgMaxAllocatedVA, targMessageGlobal,
                inv.inv[4],
                inv.inv[0], inv.inv[2], inv.inv[1], inv.inv[11], inv.inv[46], inv.inv[47],
                inv.pv[0], inv.pv[1], inv.pv[2], inv.pv[3], inv.pv[4], inv.pv[5], inv.pv[6], inv.pv[16], inv.pv[17], inv.pv[18]);
    r->send(res);
    if (locked) xSemaphoreGive(dataMutex);
  });
  server.onNotFound([](AsyncWebServerRequest* r) { r->redirect("/"); });
  server.begin();
}

void setupOTA() {
  ArduinoOTA.setHostname("inverter");
  ArduinoOTA.onStart([]() { webUpdateActive = true; digitalWrite(LED_PIN, HIGH); });
  ArduinoOTA.onEnd([]() { digitalWrite(LED_PIN, LOW); });
  ArduinoOTA.onError([](ota_error_t e) { webUpdateActive = false; digitalWrite(LED_PIN, LOW); Serial.printf("OTA Error[%u]\n", e); });
  ArduinoOTA.begin();
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
  setupWebServer(); setupOTA();
  lastUpdateMs = millis();
  serialLog("=== System started ===\n");
}

void loop() {
  unsigned long now = millis();
  if (needRestart && now - restartRequestedMs >= RESTART_DELAY_MS) ESP.restart();
  if (setupMode) { dnsServer.processNextRequest(); ArduinoOTA.handle(); return; }
  if (WiFi.status() != WL_CONNECTED && now - lastWifiReconnectMs >= WIFI_RECONNECT_INTERVAL_MS) { lastWifiReconnectMs = now; WiFi.reconnect(); }
  ArduinoOTA.handle();
  if (pendingCounterReset) { pendingCounterReset = false; node.writeSingleRegister(20213, 1); delay(100); node.writeSingleRegister(10112, 1); delay(100); }
  if (!webUpdateActive && now - lastUpdateMs >= (uint32_t)(cfgUpdateIntervalSec * 1000.0f)) { lastUpdateMs = now; pollModbusOnce(); }
}
