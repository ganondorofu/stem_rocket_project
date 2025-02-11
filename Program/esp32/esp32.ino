// ------------------------------------------------
// ライブラリのインクルード
// ------------------------------------------------
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "mbedtls/sha256.h"
#include <HTTPClient.h>

// ------------------------------------------------
// Wi-Fi設定
// ------------------------------------------------
const char* ssid = "ctc-2g-hin0zr";
const char* password = "1904affe1d065";

// ------------------------------------------------
// DuckDNS設定 (任意)
// ------------------------------------------------
const char* duckdns_domain = "yn3143";
const char* duckdns_token  = "42ead428-1a5e-408a-a5fa-3e491578416f";

// ------------------------------------------------
// シリアル通信ピン (ESP32)
// ------------------------------------------------
#define TX_PIN 17
#define RX_PIN 16

// ------------------------------------------------
// 表面LED設定（起動失敗時エラー表示用）
// ------------------------------------------------
// 多くのESP32ボードでは、内蔵LEDがGPIO2に接続されています。
// ※ボードに合わせて変更してください。
#define SURFACE_LED_PIN 2

// ------------------------------------------------
// Webサーバー (ポート28567 ※普段使われにくいポート)
// ------------------------------------------------
WebServer server(28567);

// ------------------------------------------------
// グローバル変数
// ------------------------------------------------
String lockState = "UNLOCKED";  // 初期状態: UNLOCKED
unsigned long lastDuckdnsUpdate = 0;
const unsigned long DUCKDNS_INTERVAL = 600000UL;  // 10分ごと

// エラー発生フラグ（Wi‑Fi接続失敗など、起動失敗時にtrueとなる）
bool errorOccurred = false;

// ------------------------------------------------
// ログイン用ID/Pass
// ------------------------------------------------
const char* LOGIN_ID   = "yone";
const char* LOGIN_PASS = "uyan5312";

// ------------------------------------------------
// 簡易ハッシュ関数 (DJB2方式)
// 文字列から簡易ハッシュ値を計算する（盗聴リスクが低い用途向け）
// ------------------------------------------------
String simpleHash(const char* input) {
  unsigned long hash = 5381;
  for (int i = 0; i < strlen(input); i++) {
    hash = ((hash << 5) + hash) + input[i];  // hash * 33 + c
  }
  char buf[17];
  sprintf(buf, "%08lx", hash);
  return String(buf);
}
const String hashedLoginPass = simpleHash(LOGIN_PASS);

// ------------------------------------------------
// SHA-256 ハッシュ関数 (mbedtlsを利用)
// ------------------------------------------------
String getSha256(const char* input) {
  uint8_t hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char*)input, strlen(input));
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);

  String hashStr;
  for (int i = 0; i < 32; i++) {
    char buf[3];
    sprintf(buf, "%02x", hash[i]);
    hashStr += buf;
  }
  return hashStr;
}
const String hashedKey = getSha256("XkyaetE6V-FC");

// ------------------------------------------------
// エラー発生時に表面LEDを点滅させる関数
// ※エラー状態の場合、この関数内で無限ループしLEDを500ms間隔でON/OFFします
// ------------------------------------------------
void errorBlink() {
  while (true) {
    digitalWrite(SURFACE_LED_PIN, HIGH);
    delay(500);
    digitalWrite(SURFACE_LED_PIN, LOW);
    delay(500);
  }
}

// ------------------------------------------------
// 統合ハンドラ: ログインフォーム表示と施錠/解錠画面表示
// GETリクエスト時は常にログインフォームを表示し、
// POSTリクエストでIDとパスワードのハッシュが一致した場合のみ施錠/解錠画面を返す。
// ------------------------------------------------
void handleLoginPage() {
  if (server.method() == HTTP_GET) {
    String formPage = "";
    formPage += "<!DOCTYPE html>\n";
    formPage += "<html>\n";
    formPage += "<head>\n";
    formPage += "  <meta charset=\"utf-8\">\n";
    formPage += "  <title>Login (Hashed)</title>\n";
    formPage += "  <style>\n";
    formPage += "    body { font-family: Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 0; }\n";
    formPage += "    .container { max-width: 400px; margin: 80px auto; padding: 20px; background-color: #fff; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); text-align: center; }\n";
    formPage += "    h1 { color: #333; }\n";
    formPage += "    input { width: 90%; padding: 10px; margin: 10px 0; border: 1px solid #ccc; border-radius: 4px; }\n";
    formPage += "    button { padding: 10px 20px; background-color: #007BFF; color: #fff; border: none; border-radius: 4px; cursor: pointer; }\n";
    formPage += "    button:hover { background-color: #0056b3; }\n";
    formPage += "  </style>\n";
    formPage += "</head>\n";
    formPage += "<body>\n";
    formPage += "  <div class=\"container\">\n";
    formPage += "    <h1>Login</h1>\n";
    // 以下、パスワード保存／自動入力に対応するため、name属性およびautocomplete属性を追加
    formPage += "    <form method=\"POST\" action=\"/login\" onsubmit=\"hashPass();\">\n";
    formPage += "      <input name=\"id\" id=\"uid\" placeholder=\"User ID\" autocomplete=\"username\" required>\n";
    formPage += "      <input name=\"password\" id=\"pass\" type=\"password\" placeholder=\"Password\" autocomplete=\"current-password\" required>\n";
    formPage += "      <input type=\"hidden\" name=\"passHash\" id=\"passHashField\">\n";
    formPage += "      <button type=\"submit\">Login</button>\n";
    formPage += "    </form>\n";
    formPage += "  </div>\n";
    formPage += "  <script>\n";
    formPage += "    // クライアント側で簡易ハッシュ (DJB2方式) を計算する関数\n";
    formPage += "    function simpleHash(str) {\n";
    formPage += "      let hash = 5381;\n";
    formPage += "      for (let i = 0; i < str.length; i++) {\n";
    formPage += "        hash = ((hash << 5) + hash) + str.charCodeAt(i);\n";
    formPage += "      }\n";
    formPage += "      return hash.toString(16);\n";
    formPage += "    }\n";
    formPage += "    function hashPass(){\n";
    formPage += "      let pass = document.getElementById('pass').value;\n";
    formPage += "      let passH = simpleHash(pass);\n";
    formPage += "      document.getElementById('passHashField').value = passH;\n";
    // パスワード入力欄をクリアしないようにする（これによりブラウザがパスワード保存／自動入力を行えます）\n";
    formPage += "    }\n";
    formPage += "  </script>\n";
    formPage += "</body>\n";
    formPage += "</html>\n";
    server.send(200, "text/html", formPage);
  } else if (server.method() == HTTP_POST) {
    String uid = server.arg("id");
    String passHash = server.arg("passHash");
    if (uid == LOGIN_ID && passHash == hashedLoginPass) {
      String page = "";
      page += "<!DOCTYPE html>\n";
      page += "<html>\n";
      page += "<head>\n";
      page += "  <meta charset=\"utf-8\">\n";
      page += "  <title>Lock State</title>\n";
      page += "  <style>\n";
      page += "    body { font-family: Arial, sans-serif; background-color: #eef2f3; margin: 0; padding: 0; }\n";
      page += "    .container { max-width: 600px; margin: 80px auto; padding: 20px; background-color: #fff; border-radius: 8px; box-shadow: 0 0 15px rgba(0,0,0,0.1); text-align: center; }\n";
      page += "    h1 { color: #333; }\n";
      page += "    button { padding: 10px 20px; margin: 10px; background-color: #28a745; color: #fff; border: none; border-radius: 4px; cursor: pointer; }\n";
      page += "    button:hover { background-color: #218838; }\n";
      page += "    .lock-icon { font-size: 80px; transition: transform 0.5s ease; margin-bottom: 20px; }\n";
      page += "    .locked { color: #d9534f; transform: rotate(0deg); }\n";
      page += "    .unlocked { color: #5cb85c; transform: rotate(360deg); }\n";
      page += "  </style>\n";
      page += "</head>\n";
      page += "<body>\n";
      page += "  <div class=\"container\">\n";
      page += "    <div id=\"lockIcon\" class=\"lock-icon unlocked\">🔓</div>\n";
      page += "    <h1>Lock State: <span id=\"lockState\">UNLOCKED</span></h1>\n";
      page += "    <p>Logged in. Use the buttons below to change lock state.</p>\n";
      page += "    <button onclick=\"lockDoor(true)\">LOCK</button>\n";
      page += "    <button onclick=\"lockDoor(false)\">UNLOCK</button>\n";
      page += "  </div>\n";
      page += "  <script>\n";
      page += "    const hashedKey = '" + hashedKey + "';\n";
      page += "    async function fetchLockState(){\n";
      page += "      try {\n";
      page += "        const res = await fetch('/api?cmd=getLockState');\n";
      page += "        if(!res.ok){\n";
      page += "          document.getElementById('lockState').textContent = 'Error';\n";
      page += "          return;\n";
      page += "        }\n";
      page += "        const data = await res.json();\n";
      page += "        document.getElementById('lockState').textContent = data.lockState;\n";
      page += "        const icon = document.getElementById('lockIcon');\n";
      page += "        if(data.lockState === 'LOCKED'){\n";
      page += "          icon.textContent = '🔒';\n";
      page += "          icon.className = 'lock-icon locked';\n";
      page += "        } else {\n";
      page += "          icon.textContent = '🔓';\n";
      page += "          icon.className = 'lock-icon unlocked';\n";
      page += "        }\n";
      page += "      } catch(e){\n";
      page += "        console.log(e);\n";
      page += "      }\n";
      page += "    }\n";
      page += "    async function lockDoor(doLock){\n";
      page += "      try {\n";
      page += "        const body = { key: hashedKey, lock: doLock };\n";
      page += "        const res = await fetch('/api?cmd=setLockState', {\n";
      page += "          method: 'POST',\n";
      page += "          headers: { 'Content-Type': 'application/json' },\n";
      page += "          body: JSON.stringify(body)\n";
      page += "        });\n";
      page += "        const data = await res.json();\n";
      page += "        if(!res.ok){\n";
      page += "          alert('Error: ' + data.message);\n";
      page += "        } else {\n";
      page += "          console.log('Lock updated:', data.lockState);\n";
      page += "        }\n";
      page += "      } catch(e){\n";
      page += "        console.error(e);\n";
      page += "      }\n";
      page += "    }\n";
      page += "    setInterval(fetchLockState, 1000);\n";
      page += "  </script>\n";
      page += "</body>\n";
      page += "</html>\n";
      server.send(200, "text/html", page);
    } else {
      server.send(401, "text/plain", "Login failed");
    }
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

// ------------------------------------------------
// APIハンドラ: ロック状態の取得と変更を行う
// GET: URLパラメータ cmd=getLockState で現在の lockState を返す
// POST: URLパラメータまたはJSON内に cmd:"setLockState" と key, lock が含まれている場合、ロック状態を変更する
// ------------------------------------------------
void handleApi() {
  if (server.method() == HTTP_GET) {
    String cmd = server.arg("cmd");
    if (cmd == "getLockState") {
      StaticJsonDocument<200> doc;
      doc["lockState"] = lockState;
      String resp;
      serializeJson(doc, resp);
      server.send(200, "application/json", resp);
      return;
    } else {
      server.send(405, "application/json", "{\"message\":\"Method Not Allowed or invalid cmd\"}");
      return;
    }
  } else if (server.method() == HTTP_POST) {
    String cmd = server.arg("cmd");
    String jsonBody = server.arg("plain");
    StaticJsonDocument<200> doc;
    DeserializationError err = deserializeJson(doc, jsonBody);
    if (err) {
      server.send(400, "application/json", "{\"message\":\"Invalid JSON\"}");
      return;
    }
    if (cmd == "" && doc.containsKey("cmd")) {
      cmd = doc["cmd"].as<String>();
    }
    if (cmd == "" && doc.containsKey("key") && doc.containsKey("lock")) {
      cmd = "setLockState";
    }
    if (cmd == "setLockState") {
      if (!doc.containsKey("key") || !doc.containsKey("lock")) {
        server.send(400, "application/json", "{\"message\":\"Missing parameters\"}");
        return;
      }
      String receivedKey = doc["key"].as<String>();
      bool lockVal = doc["lock"].as<bool>();
      if (receivedKey != hashedKey) {
        server.send(400, "application/json", "{\"message\":\"key is incorrect\"}");
        return;
      }
      lockState = lockVal ? "LOCKED" : "UNLOCKED";
      Serial.println("Lock state changed: " + lockState);
      Serial2.println(lockState);
      StaticJsonDocument<200> respDoc;
      respDoc["message"] = "Lock state updated";
      respDoc["lockState"] = lockState;
      String resp;
      serializeJson(respDoc, resp);
      server.send(200, "application/json", resp);
      return;
    } else {
      server.send(405, "application/json", "{\"message\":\"Method Not Allowed or invalid cmd\"}");
      return;
    }
  } else {
    server.send(405, "application/json", "{\"message\":\"Method Not Allowed or invalid cmd\"}");
    return;
  }
}

// ------------------------------------------------
// DuckDNS更新 (任意)
// ------------------------------------------------
void updateDuckDNS() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "http://www.duckdns.org/update?domains=" + String(duckdns_domain)
                 + "&token=" + String(duckdns_token) + "&ip=";
    Serial.println("DuckDNS更新: " + url);
    http.begin(url);
    int code = http.GET();
    if (code > 0) {
      String payload = http.getString();
      Serial.println("DuckDNSレスポンス: " + payload);
    } else {
      Serial.println("DuckDNS更新失敗");
    }
    http.end();
  }
}

// ------------------------------------------------
// Wi‑Fi接続（タイムアウト付き）
// ------------------------------------------------
void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Wi‑Fi接続中");
  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 30000; // 30秒のタイムアウト
  while (WiFi.status() != WL_CONNECTED && (millis() - startAttemptTime < wifiTimeout)) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi‑Fi接続完了");
    Serial.print("IPアドレス: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWi‑Fi接続失敗");
    errorOccurred = true;
  }
}

// ------------------------------------------------
// ArduinoからLOCKED/UNLOCKED受信
// ------------------------------------------------
void receiveFromArduino() {
  if (Serial2.available()) {
    String received = Serial2.readStringUntil('\n');
    received.trim();
    // 受信文字列に「自動再起動に失敗して」が含まれている場合、エラーと判断しエラーフラグをセット
    if (received.indexOf("自動再起動に失敗して") != -1) {
      Serial.println("Error detected: " + received);
      errorOccurred = true;
      return;
    }
    if (received == "LOCKED" || received == "UNLOCKED") {
      lockState = received;
      Serial.println("Received from Arduino: " + lockState);
    }
  }
}

// ------------------------------------------------
// setup()
// ------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // 表面LEDの初期化（エラー表示用）
  pinMode(SURFACE_LED_PIN, OUTPUT);
  digitalWrite(SURFACE_LED_PIN, LOW);

  setupWiFi();
  randomSeed(esp_random());

  server.on("/login", handleLoginPage);
  server.on("/api", handleApi);

  server.begin();
  Serial.println("HTTPサーバー起動");

  updateDuckDNS();
  lastDuckdnsUpdate = millis();
}

// ------------------------------------------------
// loop()
// ------------------------------------------------
void loop() {
  // エラー発生時は表面LEDを点滅させる（errorBlink()は無限ループします）
  if (errorOccurred) {
    errorBlink();
  }
  
  server.handleClient();
  receiveFromArduino();

  unsigned long now = millis();
  if ((now - lastDuckdnsUpdate) >= DUCKDNS_INTERVAL) {
    updateDuckDNS();
    lastDuckdnsUpdate = now;
  }
}
