#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <vector>

// ============================================================
//                       НАСТРОЙКИ
// ============================================================

const char* WIFI_SSID = "homewifi";
const char* WIFI_PASS = "homewifi1234567890";

const char* MDNS_NAME = "must-modbus";

const int RX_PIN = 16;
const int TX_PIN = 17;
const int DE_PIN = 4;

const uint32_t MODBUS_BAUD = 19200;

// ============================================================
//                    ОБЪЕКТЫ И ПЕРЕМЕННЫЕ
// ============================================================

HardwareSerial MBSerial(2);
AsyncWebServer server(80);

volatile bool otaSuccess = false;
volatile bool otaRestartPending = false;
volatile bool otaInProgress = false;
uint32_t otaRestartAt = 0;

// >>> SCAN: состояние автопоиска
struct ScanState {
  volatile bool running   = false;
  volatile bool done      = false;
  volatile bool found     = false;

  volatile uint32_t currentBaud  = 0;
  volatile uint8_t  currentSlave = 0;
  volatile int      baudIdx      = 0;
  volatile int      totalBauds   = 0;
  volatile int      maxSlave     = 0;
  volatile uint32_t attemptNum   = 0;
  volatile uint32_t totalAttempts = 0;

  volatile uint32_t foundBaud  = 0;
  volatile uint8_t  foundSlave = 0;
  volatile uint8_t  foundFunc  = 0;
  volatile uint8_t  foundExc   = 0;
  volatile uint16_t foundValue = 0;

  volatile uint32_t durationMs = 0;
  uint32_t startedAt = 0;

  char log[3072];
  volatile uint16_t logLen = 0;

  char foundResp[128];
} scanState;

portMUX_TYPE scanMux = portMUX_INITIALIZER_UNLOCKED;

// >>> SCAN: добавление строки в лог (безопасно из любой задачи)
void scanLog(const char* msg) {
  portENTER_CRITICAL(&scanMux);
  size_t len = strlen(msg);
  size_t space = sizeof(scanState.log) - scanState.logLen - 1;
  if (len > space) len = space;
  memcpy(scanState.log + scanState.logLen, msg, len);
  scanState.logLen += len;
  scanState.log[scanState.logLen] = 0;
  portEXIT_CRITICAL(&scanMux);
}

void scanLogf(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  scanLog(buf);
}

void scanLogReset() {
  portENTER_CRITICAL(&scanMux);
  scanState.logLen = 0;
  scanState.log[0] = 0;
  portEXIT_CRITICAL(&scanMux);
}

// ============================================================
//                       ВСПОМОГАТЕЛЬНЫЕ
// ============================================================

String hex4(uint16_t v) {
  char buf[8];
  snprintf(buf, sizeof(buf), "0x%04X", v);
  return String(buf);
}

uint32_t parseNumber(const String& s) {
  String t = s;
  t.trim();
  if (t.length() == 0) return 0;
  if (t.startsWith("0x") || t.startsWith("0X")) return strtoul(t.c_str(), nullptr, 16);
  return t.toInt();
}

uint16_t crc16(const uint8_t* buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; }
      else { crc >>= 1; }
    }
  }
  return crc;
}

String bytesToHex(const std::vector<uint8_t>& data) {
  String s;
  s.reserve(data.size() * 3);
  char buf[4];
  for (size_t i = 0; i < data.size(); i++) {
    if (i > 0) s += " ";
    snprintf(buf, sizeof(buf), "%02X", data[i]);
    s += buf;
  }
  return s;
}

void mbFlushRx() {
  while (MBSerial.available()) MBSerial.read();
}

// ============================================================
//                       MODBUS 0x03
// ============================================================

bool modbusReadHolding(
  uint8_t slave, uint16_t addr, uint16_t qty, uint32_t timeoutMs,
  String& reqHex, String& respHex, String& err,
  std::vector<uint16_t>& regs, uint16_t& respLen
) {
  reqHex = ""; respHex = ""; err = ""; regs.clear(); respLen = 0;

  if (qty < 1 || qty > 125) { err = "count must be 1..125"; return false; }

  uint8_t frame[8];
  frame[0] = slave;
  frame[1] = 0x03;
  frame[2] = (addr >> 8) & 0xFF;
  frame[3] = addr & 0xFF;
  frame[4] = (qty >> 8) & 0xFF;
  frame[5] = qty & 0xFF;
  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  std::vector<uint8_t> tx;
  tx.assign(frame, frame + 8);
  reqHex = bytesToHex(tx);

  mbFlushRx();
  if (DE_PIN >= 0) digitalWrite(DE_PIN, HIGH);
  MBSerial.write(frame, 8);
  MBSerial.flush();
  if (DE_PIN >= 0) digitalWrite(DE_PIN, LOW);

  std::vector<uint8_t> rx;
  rx.reserve(300);
  uint32_t start = millis();
  uint16_t expected = 5;

  while (millis() - start < timeoutMs) {
    while (MBSerial.available()) {
      rx.push_back(MBSerial.read());
      if (rx.size() >= 3 && rx[1] == 0x03) expected = 3 + rx[2] + 2;
      if (rx.size() >= 5 && rx[1] == 0x83) expected = 5;
      if (rx.size() >= expected) break;
    }
    if (rx.size() >= expected) break;
    delay(1);
  }

  respLen = rx.size();
  respHex = bytesToHex(rx);

  if (rx.empty())  { err = "no response"; return false; }
  if (rx.size() < 5) { err = "response too short"; return false; }

  if (rx.size() >= 4) {
    uint16_t calc = crc16(rx.data(), rx.size() - 2);
    uint16_t recv = rx[rx.size() - 2] | ((uint16_t)rx[rx.size() - 1] << 8);
    if (calc != recv) { err = "CRC error"; return false; }
  }

  if (rx[1] == 0x83) { err = "Modbus exception code " + String(rx[2]); return false; }
  if (rx[0] != slave || rx[1] != 0x03) { err = "bad slave/function"; return false; }

  uint8_t byteCount = rx[2];
  if (rx.size() < (size_t)(3 + byteCount + 2)) { err = "data incomplete"; return false; }

  for (uint16_t i = 0; i < byteCount / 2; i++) {
    uint16_t v = ((uint16_t)rx[3 + 2 * i] << 8) | rx[3 + 2 * i + 1];
    regs.push_back(v);
  }
  err = "";
  return true;
}

// ============================================================
//  >>> SCAN: низкоуровневая проба — принимает любой валидный
//  Modbus-ответ, включая exception 0x83.
// ============================================================
bool probeModbus(
  uint8_t slave, uint16_t addr, uint32_t timeoutMs,
  String& respHex, uint8_t& func, uint8_t& excCode
) {
  respHex = ""; func = 0; excCode = 0;

  uint8_t frame[8];
  frame[0] = slave;
  frame[1] = 0x03;
  frame[2] = (addr >> 8) & 0xFF;
  frame[3] = addr & 0xFF;
  frame[4] = 0;
  frame[5] = 1;
  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  mbFlushRx();
  if (DE_PIN >= 0) digitalWrite(DE_PIN, HIGH);
  MBSerial.write(frame, 8);
  MBSerial.flush();
  if (DE_PIN >= 0) digitalWrite(DE_PIN, LOW);

  std::vector<uint8_t> rx;
  rx.reserve(300);
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (MBSerial.available()) {
      rx.push_back(MBSerial.read());
      if (rx.size() >= 300) break;
    }
    bool done = false;
    if (rx.size() >= 5) {
      if ((rx[1] & 0x80) != 0) done = true;
      else if (rx[1] == 0x03 && rx.size() >= 3) {
        uint16_t exp = 3 + rx[2] + 2;
        if (rx.size() >= exp) done = true;
      }
    }
    if (done) break;
    delay(1);
  }

  respHex = bytesToHex(rx);
  if (rx.size() < 5) return false;

  uint16_t calc = crc16(rx.data(), rx.size() - 2);
  uint16_t recv = rx[rx.size() - 2] | ((uint16_t)rx[rx.size() - 1] << 8);
  if (calc != recv) return false;
  if (rx[0] != slave) return false;

  func = rx[1];
  excCode = (rx[1] & 0x80) ? rx[2] : 0;
  return true;
}

// ============================================================
//  >>> SCAN: задача сканирования — выполняется в фоне, чтобы
//  веб-сервер мог отдавать /api/scan_progress.
// ============================================================
struct ScanParams {
  uint8_t  maxSlave;
  uint16_t testAddr;
  uint32_t perTimeout;
};

void scanTask(void* pvParameters) {
  ScanParams* p = (ScanParams*)pvParameters;
  uint8_t  maxSlave   = p->maxSlave;
  uint16_t testAddr   = p->testAddr;
  uint32_t perTimeout = p->perTimeout;
  delete p;

  const uint32_t bauds[] = { 9600, 19200, 38400, 4800, 2400, 57600, 115200, 1200 };
  const int NUM_BAUDS = sizeof(bauds) / sizeof(bauds[0]);

  uint32_t originalBaud = MODBUS_BAUD;
  uint32_t t0 = millis();

  scanLogReset();
  scanLog("=== Auto-scan started ===\n");
  scanLogf("Test addr: %s, maxSlave: %d, per-timeout: %lu ms\n",
           hex4(testAddr).c_str(), maxSlave, (unsigned long)perTimeout);
  scanLog("-----------------------------------\n");

  for (int b = 0; b < NUM_BAUDS; b++) {
    scanState.baudIdx = b;
    scanState.currentBaud = bauds[b];

    scanLogf("\n[%d/%d] Baud %lu: switching UART...\n",
             b + 1, NUM_BAUDS, (unsigned long)bauds[b]);

    MBSerial.flush();
    MBSerial.end();
    delay(5);
    MBSerial.begin(bauds[b], SERIAL_8N1, RX_PIN, TX_PIN);
    delay(5);
    mbFlushRx();

    for (uint8_t sl = 1; sl <= maxSlave; sl++) {
      scanState.currentSlave = sl;
      scanState.attemptNum++;

      String respHex;
      uint8_t func = 0, exc = 0;

      bool ok = probeModbus(sl, testAddr, perTimeout, respHex, func, exc);

      if (ok) {
        scanLogf("  slave=%d -> RESPONSE: %s\n", sl, respHex.c_str());

        scanState.found      = true;
        scanState.foundBaud  = bauds[b];
        scanState.foundSlave = sl;
        scanState.foundFunc  = func;
        scanState.foundExc   = exc;

        strncpy(scanState.foundResp, respHex.c_str(), sizeof(scanState.foundResp) - 1);
        scanState.foundResp[sizeof(scanState.foundResp) - 1] = 0;

        if (exc == 0 && func == 0x03) {
          // Парсим значение первого регистра
          std::vector<uint8_t> bytes;
          const char* c = respHex.c_str();
          while (*c) {
            while (*c == ' ') c++;
            if (!*c) break;
            char hb[3] = { c[0], c[1], 0 };
            bytes.push_back((uint8_t)strtoul(hb, nullptr, 16));
            c += 2;
          }
          if (bytes.size() >= 5) {
            scanState.foundValue = ((uint16_t)bytes[3] << 8) | bytes[4];
          }
          scanLogf("  >>> FOUND! baud=%lu slave=%d value=%u (%s)\n",
                   (unsigned long)bauds[b], sl, scanState.foundValue,
                   hex4(scanState.foundValue).c_str());
        } else {
          scanLogf("  >>> FOUND! baud=%lu slave=%d exception=0x%02X\n",
                   (unsigned long)bauds[b], sl, exc);
        }
        break;
      } else {
        // Краткий отчёт: без мусора
        scanLogf("  slave=%d -> no reply\n", sl);
      }

      // Отдаём управление, чтобы веб-сервер успел ответить
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (scanState.found) break;
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  // Восстанавливаем исходную скорость
  scanLog("\nRestoring original baud...\n");
  MBSerial.flush();
  MBSerial.end();
  delay(5);
  MBSerial.begin(originalBaud, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(5);
  mbFlushRx();

  scanState.durationMs = millis() - t0;
  scanState.done = true;
  scanState.running = false;

  if (scanState.found) {
    scanLogf("\n=== DONE in %lu ms ===\n", (unsigned long)scanState.durationMs);
    scanLogf("Result: baud=%lu, slave=%d\n",
             (unsigned long)scanState.foundBaud, scanState.foundSlave);
  } else {
    scanLogf("\n=== DONE in %lu ms, NOT FOUND ===\n",
             (unsigned long)scanState.durationMs);
    scanLog("Check A/B wiring, GND, DE pin, or try a different test addr.\n");
  }

  vTaskDelete(nullptr);
}

// ============================================================
//                       ГЛАВНАЯ СТРАНИЦА
// ============================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Must Modbus Raw</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 12px; }
    input { width: 100px; padding: 5px; margin-right: 8px; }
    button, .btn { padding: 6px 10px; margin-right: 8px; }
    pre { background: #f8f8f8; border: 1px solid #ddd; padding: 10px;
          white-space: pre-wrap; word-break: break-all; min-height: 200px; }
    .row { margin-bottom: 10px; }
    .scan { background: #fff3cd; padding: 10px; border-radius: 6px; border: 1px solid #ffe08a; }
    .scan .bar { height: 12px; background: #e9e2c7; border-radius: 6px; overflow: hidden; margin-top: 6px; }
    .scan .fill { height: 100%; width: 0; background: #ffb300; transition: width 0.2s; }
    .scan .status { font-size: 13px; margin-top: 6px; font-weight: bold; }
    .scan .logbox { background: #1c1c1c; color: #d6d6d6; font-family: monospace; font-size: 11px;
                    max-height: 260px; overflow-y: auto; padding: 8px; border-radius: 4px;
                    white-space: pre-wrap; word-break: break-all; margin-top: 8px; }
    .found { color: #0a7a2b; }
    .notfound { color: #b71c1c; }
  </style>
</head>
<body>
  <h1>Must Modbus Raw Reader</h1>

  <div class="row">
    Slave <input id="slave" value="1">
    Start addr <input id="addr" value="44176">
    Count <input id="count" value="5">
    Timeout ms <input id="timeout" value="1500">
    <button onclick="readChunkUI()">Read chunk</button>
  </div>

  <div class="row">
    Dump from <input id="start" value="44100">
    to <input id="end" value="44199">
    chunk <input id="chunk" value="100">
    <button onclick="dumpRange()">Dump range</button>
    <a class="btn" href="/update" target="_blank">OTA</a>
  </div>

  <div class="row scan">
    <b>Автопоиск параметров связи</b><br><br>
    Max slave <input id="scanMaxSlave" value="5">
    Test addr <input id="scanAddr" value="0x0000">
    Per-timeout ms <input id="scanTimeout" value="250">
    <button id="scanBtn" onclick="startScan()">Сканировать</button>
    <button class="btn" onclick="stopPoll()">Стоп</button>
    <div class="status" id="scanStatus">Готов.</div>
    <div class="bar"><div class="fill" id="scanFill"></div></div>
    <div class="logbox" id="scanLog">— лог появится здесь —</div>
  </div>

  <pre id="out">Ready.</pre>

  <script>
    function getVal(id) { return document.getElementById(id).value; }
    function setOut(text) { document.getElementById("out").textContent = text; }
    function appendOut(text) { document.getElementById("out").textContent += text; }

    function parseNum(s) {
      s = String(s).trim();
      if (s.toLowerCase().startsWith("0x")) return parseInt(s, 16);
      return parseInt(s, 10);
    }

    function formatRegisters(d) {
      var s = "";
      if (!d.registers) return s;
      for (var i = 0; i < d.registers.length; i++) {
        var r = d.registers[i];
        s += r.addr + "  " + r.hex + "  " + r.value + "  " + r.hexValue + "\n";
      }
      return s;
    }

    async function fetchRaw(addr, count) {
      if (isNaN(addr)) addr = 0;
      if (isNaN(count) || count < 1) count = 1;
      if (count > 125) count = 125;
      var url = "/api/raw?slave=" + encodeURIComponent(getVal("slave")) +
                "&addr=" + encodeURIComponent(addr) +
                "&count=" + encodeURIComponent(count) +
                "&timeout=" + encodeURIComponent(getVal("timeout"));
      var r = await fetch(url);
      var text = await r.text();
      try { return JSON.parse(text); }
      catch (e) { return { ok: false, error: "JSON parse error", raw: text }; }
    }

    async function readChunkUI() {
      var addr = parseNum(getVal("addr"));
      var count = parseNum(getVal("count"));
      if (isNaN(addr)) { setOut("Invalid start address"); return; }
      if (isNaN(count) || count < 1) count = 1;
      if (count > 125) count = 125;
      setOut("Reading...");
      try {
        var d = await fetchRaw(addr, count);
        var out = "";
        out += "OK: " + d.ok + "\n";
        out += "Error: " + (d.error || "") + "\n";
        out += "Duration: " + d.durationMs + " ms\n";
        out += "Response bytes: " + d.responseBytes + "\n\n";
        out += "REQUEST HEX:\n" + d.requestHex + "\n\n";
        out += "RESPONSE HEX:\n" + d.responseHex + "\n\n";
        out += "REGISTERS:\naddr  hex  value  hexValue\n";
        out += formatRegisters(d);
        setOut(out);
      } catch (e) { setOut("Error: " + e); }
    }

    async function dumpRange() {
      var start = parseNum(getVal("start"));
      var end = parseNum(getVal("end"));
      var chunk = parseNum(getVal("chunk"));
      if (isNaN(start) || isNaN(end)) { setOut("Invalid start/end"); return; }
      if (isNaN(chunk) || chunk < 1) chunk = 1;
      if (chunk > 125) chunk = 125;
      if (end < start) { setOut("End is less than start"); return; }
      if ((end - start) > 20000) { if (!confirm("Range is very large. Continue?")) return; }
      setOut("Dumping...\n");
      try {
        var out = "";
        for (var a = start; a <= end; a += chunk) {
          var c = Math.min(chunk, end - a + 1);
          var d = await fetchRaw(a, c);
          out += "=== addr " + a + " count " + c + "\n";
          out += "ok: " + d.ok + ", error: " + (d.error || "") + ", bytes: " + d.responseBytes + "\n";
          out += "REQ: " + d.requestHex + "\n";
          out += "RES: " + d.responseHex + "\n";
          out += formatRegisters(d) + "\n";
          setOut(out);
          await new Promise(function(res) { setTimeout(res, 50); });
        }
        appendOut("Done.\n");
      } catch (e) { appendOut("Error: " + e + "\n"); }
    }

    // >>> SCAN: управление автопоиском с прогрессом
    let scanTimer = null;
    let lastLogLen = 0;

    function stopPoll() {
      if (scanTimer) { clearInterval(scanTimer); scanTimer = null; }
    }

    async function startScan() {
      var maxSlave = parseInt(getVal("scanMaxSlave"), 10);
      var addr     = parseNum(getVal("scanAddr"));
      var timeout  = parseInt(getVal("scanTimeout"), 10);
      if (isNaN(maxSlave) || maxSlave < 1) maxSlave = 5;
      if (maxSlave > 32) maxSlave = 32;
      if (isNaN(addr)) addr = 0;
      if (isNaN(timeout) || timeout < 50) timeout = 250;
      if (timeout > 2000) timeout = 2000;

      document.getElementById("scanStatus").innerText = "Запуск...";
      document.getElementById("scanFill").style.width = "0%";
      document.getElementById("scanLog").innerText = "";
      lastLogLen = 0;

      var url = "/api/scan?maxSlave=" + maxSlave +
                "&addr=" + addr +
                "&timeout=" + timeout;

      try {
        var r = await fetch(url);
        var d = await r.json();
        if (d.error) {
          document.getElementById("scanStatus").innerText = "Ошибка: " + d.error;
          return;
        }
      } catch (e) {
        document.getElementById("scanStatus").innerText = "Ошибка запуска: " + e;
        return;
      }

      // Каждые 300 мс — опрос прогресса
      stopPoll();
      scanTimer = setInterval(pollScan, 300);
      pollScan();
    }

    async function pollScan() {
      try {
        let r = await fetch("/api/scan_progress");
        let d = await r.json();

        let total = d.totalAttempts || 1;
        let cur   = d.attemptNum || 0;
        let pct   = Math.min(100, Math.round(cur * 100 / total));
        document.getElementById("scanFill").style.width = pct + "%";

        let st = "";
        if (d.running) {
          st = "Сканирую... " + pct + "% | baud=" + d.currentBaud +
               " slave=" + d.currentSlave +
               " (" + d.baudIdx + "/" + d.totalBauds + ")";
          document.getElementById("scanBtn").disabled = true;
        } else if (d.done) {
          if (d.found) {
            st = "НАЙДЕНО! baud=" + d.foundBaud + ", slave=" + d.foundSlave;
            document.getElementById("scanFill").style.width = "100%";
          } else {
            st = "Не найдено. Время: " + d.durationMs + " мс.";
          }
          document.getElementById("scanBtn").disabled = false;
          stopPoll();
        } else {
          st = "Ожидание...";
        }
        document.getElementById("scanStatus").innerText = st;

        // Обновляем лог
        let logBox = document.getElementById("scanLog");
        if (d.log && d.log.length !== lastLogLen) {
          logBox.innerText = d.log;
          logBox.scrollTop = logBox.scrollHeight;
          lastLogLen = d.log.length;
        }
      } catch (e) {
        console.error(e);
      }
    }
  </script>
</body>
</html>
)rawliteral";

// ============================================================
//                         OTA СТРАНИЦА
// ============================================================

const char OTA_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 OTA Update</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; }
    input[type="file"] { margin-bottom: 10px; }
    button { padding: 8px 14px; }
  </style>
</head>
<body>
  <h2>ESP32 OTA Update</h2>
  <form method="POST" action="/update" enctype="multipart/form-data">
    <input type="file" name="update" accept=".bin"><br>
    <button type="submit">Update</button>
  </form>
</body>
</html>
)rawliteral";

// ============================================================
//                         SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  if (DE_PIN >= 0) {
    pinMode(DE_PIN, OUTPUT);
    digitalWrite(DE_PIN, LOW);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  WiFi.setSleep(false);

  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("mDNS: http://");
    Serial.print(MDNS_NAME);
    Serial.println(".local");
  }

  Serial.println("Main page: http://" + WiFi.localIP().toString() + "/");
  Serial.println("OTA page:  http://" + WiFi.localIP().toString() + "/update");

  MBSerial.begin(MODBUS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  // Главная страница
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  // Сырой Modbus запрос
  server.on("/api/raw", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (otaInProgress) {
      request->send(200, "application/json", "{\"error\":\"busy during OTA\"}");
      return;
    }
    // >>> SCAN: не даём читать, пока идёт сканирование
    if (scanState.running) {
      request->send(200, "application/json", "{\"error\":\"scan in progress\"}");
      return;
    }

    uint32_t slave = 1, addr = 0, count = 1, timeout = 1500;
    if (request->hasParam("slave"))   slave   = parseNumber(request->getParam("slave")->value());
    if (request->hasParam("addr"))    addr    = parseNumber(request->getParam("addr")->value());
    if (request->hasParam("count"))   count   = parseNumber(request->getParam("count")->value());
    if (request->hasParam("timeout")) timeout = parseNumber(request->getParam("timeout")->value());

    if (slave < 1 || slave > 247) { request->send(200, "application/json", "{\"error\":\"slave must be 1..247\"}"); return; }
    if (addr > 65535) { request->send(200, "application/json", "{\"error\":\"addr must be 0..65535\"}"); return; }
    if (count < 1) count = 1;
    if (count > 125) count = 125;
    if (addr + count > 65536) count = 65536 - addr;
    if (timeout < 50) timeout = 50;
    if (timeout > 10000) timeout = 10000;

    String reqHex, respHex, err;
    std::vector<uint16_t> regs;
    uint16_t respLen = 0;
    uint32_t t0 = millis();
    bool ok = modbusReadHolding((uint8_t)slave, (uint16_t)addr, (uint16_t)count, timeout,
                                reqHex, respHex, err, regs, respLen);
    uint32_t duration = millis() - t0;

    String json;
    json.reserve(8192);
    json += "{";
    json += "\"ok\":"; json += ok ? "true" : "false"; json += ",";
    json += "\"error\":\"" + err + "\",";
    json += "\"durationMs\":" + String(duration) + ",";
    json += "\"requestHex\":\"" + reqHex + "\",";
    json += "\"responseHex\":\"" + respHex + "\",";
    json += "\"responseBytes\":" + String(respLen) + ",";
    json += "\"registers\":[";
    bool first = true;
    for (size_t i = 0; i < regs.size(); i++) {
      if (!first) json += ",";
      first = false;
      uint16_t regAddr = (uint16_t)(addr + i);
      uint16_t value = regs[i];
      json += "{";
      json += "\"addr\":" + String(regAddr) + ",";
      json += "\"hex\":\"" + hex4(regAddr) + "\",";
      json += "\"value\":" + String(value) + ",";
      json += "\"hexValue\":\"" + hex4(value) + "\"";
      json += "}";
    }
    json += "]}";
    request->send(200, "application/json", json);
  });

  // >>> SCAN: запуск задачи сканирования
  server.on("/api/scan", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (otaInProgress) {
      request->send(200, "application/json", "{\"error\":\"busy during OTA\"}");
      return;
    }
    if (scanState.running) {
      request->send(200, "application/json", "{\"error\":\"scan already running\"}");
      return;
    }

    uint8_t  maxSlave    = 5;
    uint16_t testAddr    = 0;
    uint32_t perTimeout  = 250;

    if (request->hasParam("maxSlave")) {
      uint32_t v = parseNumber(request->getParam("maxSlave")->value());
      if (v >= 1 && v <= 32) maxSlave = (uint8_t)v;
    }
    if (request->hasParam("addr")) {
      uint32_t v = parseNumber(request->getParam("addr")->value());
      if (v <= 0xFFFF) testAddr = (uint16_t)v;
    }
    if (request->hasParam("timeout")) {
      uint32_t v = parseNumber(request->getParam("timeout")->value());
      if (v >= 50 && v <= 2000) perTimeout = v;
    }

    // Сброс состояния
    scanState.running    = true;
    scanState.done       = false;
    scanState.found      = false;
    scanState.currentBaud = 0;
    scanState.currentSlave = 0;
    scanState.baudIdx = 0;
    scanState.totalBauds = 8;
    scanState.maxSlave = maxSlave;
    scanState.attemptNum = 0;
    scanState.totalAttempts = 8 * maxSlave;
    scanState.foundBaud = 0;
    scanState.foundSlave = 0;
    scanState.foundFunc = 0;
    scanState.foundExc = 0;
    scanState.foundValue = 0;
    scanState.durationMs = 0;
    scanState.startedAt = millis();
    scanState.foundResp[0] = 0;

    ScanParams* p = new ScanParams{ maxSlave, testAddr, perTimeout };

    // Задача на ядре 0, чтобы не блокировать веб-сервер (он на ядре 0/1 по умолчанию)
    xTaskCreatePinnedToCore(scanTask, "scanTask", 4096, p, 1, nullptr, 0);

    request->send(200, "application/json", "{\"started\":true}");
  });

  // >>> SCAN: прогресс и лог
  server.on("/api/scan_progress", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json;
    json.reserve(4096);

    json += "{";
    json += "\"running\":";  json += scanState.running ? "true" : "false"; json += ",";
    json += "\"done\":";     json += scanState.done ? "true" : "false"; json += ",";
    json += "\"found\":";    json += scanState.found ? "true" : "false"; json += ",";

    json += "\"currentBaud\":";  json += String(scanState.currentBaud); json += ",";
    json += "\"currentSlave\":"; json += String(scanState.currentSlave); json += ",";
    json += "\"baudIdx\":";      json += String(scanState.baudIdx + 1); json += ",";
    json += "\"totalBauds\":";   json += String(scanState.totalBauds); json += ",";
    json += "\"maxSlave\":";     json += String(scanState.maxSlave); json += ",";
    json += "\"attemptNum\":";   json += String(scanState.attemptNum); json += ",";
    json += "\"totalAttempts\":";json += String(scanState.totalAttempts); json += ",";

    json += "\"foundBaud\":";    json += String(scanState.foundBaud); json += ",";
    json += "\"foundSlave\":";   json += String(scanState.foundSlave); json += ",";
    json += "\"foundFunc\":";    json += String(scanState.foundFunc); json += ",";
    json += "\"foundExc\":";     json += String(scanState.foundExc); json += ",";
    json += "\"foundValue\":";   json += String(scanState.foundValue); json += ",";
    json += "\"durationMs\":";   json += String(scanState.durationMs); json += ",";

    // Лог
    char logCopy[sizeof(scanState.log)];
    portENTER_CRITICAL(&scanMux);
    uint16_t n = scanState.logLen;
    if (n >= sizeof(logCopy)) n = sizeof(logCopy) - 1;
    memcpy(logCopy, scanState.log, n);
    logCopy[n] = 0;
    portEXIT_CRITICAL(&scanMux);

    json += "\"log\":\"";
    // JSON-escape
    for (uint16_t i = 0; i < n; i++) {
      char c = logCopy[i];
      switch (c) {
        case '"':  json += "\\\""; break;
        case '\\': json += "\\\\"; break;
        case '\n': json += "\\n";  break;
        case '\r': json += "\\r";  break;
        case '\t': json += "\\t";  break;
        default:
          if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", (uint8_t)c); json += b; }
          else json += c;
      }
    }
    json += "\"";
    json += "}";

    request->send(200, "application/json", json);
  });

  // OTA страница
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", OTA_HTML);
  });

  // Прием OTA-файла
  server.on(
    "/update",
    HTTP_POST,
    [](AsyncWebServerRequest* request) {
      otaInProgress = false;
      String message = otaSuccess ? "OK. Restarting..." : "UPDATE FAILED";
      AsyncWebServerResponse* response =
        request->beginResponse(otaSuccess ? 200 : 500, "text/plain", message);
      response->addHeader("Connection", "close");
      request->send(response);
      if (otaSuccess) {
        otaRestartAt = millis();
        otaRestartPending = true;
      }
    },
    [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
      if (!index) {
        otaInProgress = true;
        otaSuccess = false;
        Serial.printf("OTA start: %s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      }
      if (!Update.hasError()) {
        if (Update.write(data, len) != len) Update.printError(Serial);
      }
      if (final) {
        otaInProgress = false;
        if (Update.end(true)) { otaSuccess = true; Serial.println("OTA update success"); }
        else { otaSuccess = false; Update.printError(Serial); }
      }
    }
  );

  server.begin();
}

// ============================================================
//                          LOOP
// ============================================================

void loop() {
  if (otaRestartPending && millis() - otaRestartAt > 1000) {
    ESP.restart();
  }
}
