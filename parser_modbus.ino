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

// Пины для ESP32 + MAX485
const int RX_PIN = 16;   // ESP32 RX2 -> RO модуля RS485
const int TX_PIN = 17;   // ESP32 TX2 -> DI модуля RS485
const int DE_PIN = 4;    // GPIO для RE/DE. Если авто-направление, поставь -1

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

  if (t.length() == 0) {
    return 0;
  }

  if (t.startsWith("0x") || t.startsWith("0X")) {
    return strtoul(t.c_str(), nullptr, 16);
  }

  return t.toInt();
}

uint16_t crc16(const uint8_t* buf, uint16_t len) {
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

String bytesToHex(const std::vector<uint8_t>& data) {
  String s;
  s.reserve(data.size() * 3);

  char buf[4];

  for (size_t i = 0; i < data.size(); i++) {
    if (i > 0) {
      s += " ";
    }

    snprintf(buf, sizeof(buf), "%02X", data[i]);
    s += buf;
  }

  return s;
}

void mbFlushRx() {
  while (MBSerial.available()) {
    MBSerial.read();
  }
}

// ============================================================
//                 ЧТЕНИЕ HOLDING REGISTERS 0x03
// ============================================================

bool modbusReadHolding(
  uint8_t slave,
  uint16_t addr,
  uint16_t qty,
  uint32_t timeoutMs,
  String& reqHex,
  String& respHex,
  String& err,
  std::vector<uint16_t>& regs,
  uint16_t& respLen
) {
  reqHex = "";
  respHex = "";
  err = "";
  regs.clear();
  respLen = 0;

  if (qty < 1 || qty > 125) {
    err = "count must be 1..125";
    return false;
  }

  uint8_t frame[8];

  frame[0] = slave;
  frame[1] = 0x03;
  frame[2] = (addr >> 8) & 0xFF;
  frame[3] = addr & 0xFF;
  frame[4] = (qty >> 8) & 0xFF;
  frame[5] = qty & 0xFF;

  uint16_t crc = crc16(frame, 6);

  frame[6] = crc & 0xFF;   // CRC low
  frame[7] = crc >> 8;     // CRC high

  std::vector<uint8_t> tx;
  tx.assign(frame, frame + 8);
  reqHex = bytesToHex(tx);

  mbFlushRx();

  if (DE_PIN >= 0) {
    digitalWrite(DE_PIN, HIGH);
  }

  MBSerial.write(frame, 8);
  MBSerial.flush();

  if (DE_PIN >= 0) {
    digitalWrite(DE_PIN, LOW);
  }

  std::vector<uint8_t> rx;
  rx.reserve(300);

  uint32_t start = millis();
  uint16_t expected = 5;

  while (millis() - start < timeoutMs) {
    while (MBSerial.available()) {
      rx.push_back(MBSerial.read());

      // Обычный ответ функции 03:
      // [slave][func][byteCount][data...][crc][crc]
      if (rx.size() >= 3 && rx[1] == 0x03) {
        expected = 3 + rx[2] + 2;
      }

      // Модбус-исключение:
      // [slave][0x83][exceptionCode][crc][crc]
      if (rx.size() >= 5 && rx[1] == 0x83) {
        expected = 5;
      }

      if (rx.size() >= expected) {
        break;
      }
    }

    if (rx.size() >= expected) {
      break;
    }

    delay(1);
  }

  respLen = rx.size();
  respHex = bytesToHex(rx);

  if (rx.empty()) {
    err = "no response";
    return false;
  }

  if (rx.size() < 5) {
    err = "response too short";
    return false;
  }

  // Проверка CRC
  if (rx.size() >= 4) {
    uint16_t calc = crc16(rx.data(), rx.size() - 2);
    uint16_t recv = rx[rx.size() - 2] | ((uint16_t)rx[rx.size() - 1] << 8);

    if (calc != recv) {
      err = "CRC error";
      return false;
    }
  }

  // Modbus exception
  if (rx[1] == 0x83) {
    err = "Modbus exception code " + String(rx[2]);
    return false;
  }

  if (rx[0] != slave || rx[1] != 0x03) {
    err = "bad slave/function";
    return false;
  }

  uint8_t byteCount = rx[2];

  if (rx.size() < (size_t)(3 + byteCount + 2)) {
    err = "data incomplete";
    return false;
  }

  for (uint16_t i = 0; i < byteCount / 2; i++) {
    uint16_t v = ((uint16_t)rx[3 + 2 * i] << 8) | rx[3 + 2 * i + 1];
    regs.push_back(v);
  }

  err = "";
  return true;
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
    body {
      font-family: Arial, sans-serif;
      margin: 12px;
    }
    input {
      width: 100px;
      padding: 5px;
      margin-right: 8px;
    }
    button, .btn {
      padding: 6px 10px;
      margin-right: 8px;
    }
    pre {
      background: #f8f8f8;
      border: 1px solid #ddd;
      padding: 10px;
      white-space: pre-wrap;
      word-break: break-all;
      min-height: 300px;
    }
    .row {
      margin-bottom: 10px;
    }
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

  <pre id="out">Ready.</pre>

  <script>
    function getVal(id) {
      return document.getElementById(id).value;
    }

    function setOut(text) {
      document.getElementById("out").textContent = text;
    }

    function appendOut(text) {
      document.getElementById("out").textContent += text;
    }

    function parseNum(s) {
      s = String(s).trim();

      if (s.toLowerCase().startsWith("0x")) {
        return parseInt(s, 16);
      }

      return parseInt(s, 10);
    }

    function formatRegisters(d) {
      var s = "";

      if (!d.registers) {
        return s;
      }

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

      try {
        return JSON.parse(text);
      } catch (e) {
        return {
          ok: false,
          error: "JSON parse error",
          raw: text
        };
      }
    }

    async function readChunkUI() {
      var addr = parseNum(getVal("addr"));
      var count = parseNum(getVal("count"));

      if (isNaN(addr)) {
        setOut("Invalid start address");
        return;
      }

      if (isNaN(count) || count < 1) {
        count = 1;
      }

      if (count > 125) {
        count = 125;
      }

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

        out += "REGISTERS:\n";
        out += "addr  hex  value  hexValue\n";
        out += formatRegisters(d);

        setOut(out);
      } catch (e) {
        setOut("Error: " + e);
      }
    }

    async function dumpRange() {
      var start = parseNum(getVal("start"));
      var end = parseNum(getVal("end"));
      var chunk = parseNum(getVal("chunk"));

      if (isNaN(start) || isNaN(end)) {
        setOut("Invalid start/end");
        return;
      }

      if (isNaN(chunk) || chunk < 1) {
        chunk = 1;
      }

      if (chunk > 125) {
        chunk = 125;
      }

      if (end < start) {
        setOut("End is less than start");
        return;
      }

      if ((end - start) > 20000) {
        if (!confirm("Range is very large. Continue?")) {
          return;
        }
      }

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

          await new Promise(function(res) {
            setTimeout(res, 50);
          });
        }

        appendOut("Done.\n");
      } catch (e) {
        appendOut("Error: " + e + "\n");
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
    body {
      font-family: Arial, sans-serif;
      margin: 20px;
    }
    input[type="file"] {
      margin-bottom: 10px;
    }
    button {
      padding: 8px 14px;
    }
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
  // Пример:
  // /api/raw?slave=1&addr=44180&count=1&timeout=1500
  // /api/raw?slave=1&addr=0xAC94&count=1&timeout=1500
  server.on("/api/raw", HTTP_GET, [](AsyncWebServerRequest* request) {

    if (otaInProgress) {
      request->send(200, "application/json", "{\"error\":\"busy during OTA\"}");
      return;
    }

    uint32_t slave = 1;
    uint32_t addr = 0;
    uint32_t count = 1;
    uint32_t timeout = 1500;

    if (request->hasParam("slave")) {
      slave = parseNumber(request->getParam("slave")->value());
    }

    if (request->hasParam("addr")) {
      addr = parseNumber(request->getParam("addr")->value());
    }

    if (request->hasParam("count")) {
      count = parseNumber(request->getParam("count")->value());
    }

    if (request->hasParam("timeout")) {
      timeout = parseNumber(request->getParam("timeout")->value());
    }

    if (slave < 1 || slave > 247) {
      request->send(200, "application/json", "{\"error\":\"slave must be 1..247\"}");
      return;
    }

    if (addr > 65535) {
      request->send(200, "application/json", "{\"error\":\"addr must be 0..65535\"}");
      return;
    }

    if (count < 1) {
      count = 1;
    }

    if (count > 125) {
      count = 125;
    }

    if (addr + count > 65536) {
      count = 65536 - addr;
    }

    if (timeout < 50) {
      timeout = 50;
    }

    if (timeout > 10000) {
      timeout = 10000;
    }

    String reqHex;
    String respHex;
    String err;
    std::vector<uint16_t> regs;
    uint16_t respLen = 0;

    uint32_t t0 = millis();

    bool ok = modbusReadHolding(
      (uint8_t)slave,
      (uint16_t)addr,
      (uint16_t)count,
      timeout,
      reqHex,
      respHex,
      err,
      regs,
      respLen
    );

    uint32_t duration = millis() - t0;

    String json;
    json.reserve(8192);

    json += "{";
    json += "\"ok\":";
    json += ok ? "true" : "false";
    json += ",";

    json += "\"error\":\"" + err + "\",";
    json += "\"durationMs\":" + String(duration) + ",";
    json += "\"requestHex\":\"" + reqHex + "\",";
    json += "\"responseHex\":\"" + respHex + "\",";
    json += "\"responseBytes\":" + String(respLen) + ",";

    json += "\"registers\":[";

    bool first = true;

    for (size_t i = 0; i < regs.size(); i++) {
      if (!first) {
        json += ",";
      }

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

      String message;

      if (otaSuccess) {
        message = "OK. Restarting...";
      } else {
        message = "UPDATE FAILED";
      }

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

        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
        }
      }

      if (!Update.hasError()) {
        if (Update.write(data, len) != len) {
          Update.printError(Serial);
        }
      }

      if (final) {
        otaInProgress = false;

        if (Update.end(true)) {
          otaSuccess = true;
          Serial.println("OTA update success");
        } else {
          otaSuccess = false;
          Update.printError(Serial);
        }
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
