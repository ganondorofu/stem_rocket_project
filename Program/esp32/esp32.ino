// ------------------------------------------------
// ライブラリのインクルード
// ------------------------------------------------
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "mbedtls/sha256.h"
#include <HTTPClient.h>

// ------------------------------------------------
// Wi‑Fi設定
// ------------------------------------------------
const char* ssid = "ctc-2g-hin0zr";
const char* password = "1904affe1d065";

// ------------------------------------------------
// DuckDNS設定 (任意)
// ------------------------------------------------
const char* duckdns_domain = "yn3143";
const char* duckdns_token  = "42ead428-1a5e-408a-a5fa-3e491578416f";

// ------------------------------------------------
// OpenWeather API設定
// ------------------------------------------------
const char* openweather_apiKey = "7fb64a2a93949c3706c41ccb4a278aa8";
const char* openweather_city   = "Nagoya,JP";  // 名古屋市（国コードJP）

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

// OpenWeather更新用タイマー（例として10分ごとに更新）
unsigned long lastWeatherUpdate = 0;
const unsigned long WEATHER_UPDATE_INTERVAL = 600000UL; // 10分ごと

// エラー発生フラグ（Wi‑Fi接続失敗などの場合）
bool errorOccurred = false;

// ------------------------------------------------
// ログイン用ID/Pass
// ------------------------------------------------
const char* LOGIN_ID   = "yone";
const char* LOGIN_PASS = "uyan5312";

// ------------------------------------------------
// 簡易ハッシュ関数 (DJB2方式)
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
// OpenWeather APIから名古屋市の天気情報を取得する関数
// 取得結果はSerialモニタに出力するとともに、LCD表示用内容を生成する
// ------------------------------------------------
void updateWeather() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "http://api.openweathermap.org/data/2.5/weather?q=";
    url += openweather_city;
    url += "&appid=";
    url += openweather_apiKey;
    url += "&units=metric";  // 摂氏表示
    Serial.println("OpenWeather API URL: " + url);
    
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode > 0) {
      String payload = http.getString();
      Serial.println("OpenWeatherレスポンスコード: " + String(httpCode));
      Serial.println("OpenWeatherレスポンス:");
      Serial.println(payload);
      // 取得した天気情報からLCD表示用内容を生成
      displayWeatherOnLCD(payload);
    } else {
      Serial.println("OpenWeather API取得失敗");
    }
    http.end();
  } else {
    Serial.println("Wi‑Fi未接続のため、OpenWeather API取得不可");
  }
}

// ------------------------------------------------
// LCD表示用内容生成関数
// 取得した天気情報(JSON)をパースし、1602 LCD用の2行（各16文字以内）の内容を生成する
// 玄関前の表示例として、1行目に気温と大まかな天気（短縮表記）、2行目に傘のアドバイスを表示
// ------------------------------------------------
void displayWeatherOnLCD(const String &payload) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.println("天気情報JSONのパースエラー");
    return;
  }
  
  // JSONから必要な情報を取得
  float temperature = doc["main"]["temp"];
  const char* weatherDesc = doc["weather"][0]["description"];
  
  // 天気説明を小文字に変換して判定
  String desc = String(weatherDesc);
  desc.toLowerCase();
  
  // 短縮天気表記（1行目用）と傘アドバイス（2行目用）を決定
  String shortWeather;
  String umbrellaAdvice;
  
  if (desc.indexOf("rain") != -1 || desc.indexOf("shower") != -1 || desc.indexOf("drizzle") != -1) {
    shortWeather = "雨";
    umbrellaAdvice = "傘持参";
  }
  else if (desc.indexOf("clear") != -1 || desc.indexOf("sunny") != -1) {
    shortWeather = "晴";
    umbrellaAdvice = "傘不要";
  }
  else if (desc.indexOf("cloud") != -1) {
    shortWeather = "曇";
    umbrellaAdvice = "傘検討";
  }
  else {
    shortWeather = "?";
    umbrellaAdvice = "注意";
  }
  
  // 1行目：気温と短縮天気表記（例："23.5C 晴"）
  String line1 = String(temperature, 1) + "C " + shortWeather;
  if (line1.length() > 16) {
    line1 = line1.substring(0, 16);
  }
  
  // 2行目：傘のアドバイス（例："傘持参"）
  String line2 = umbrellaAdvice;
  if (line2.length() > 16) {
    line2 = line2.substring(0, 16);
  }
  
  // LCDに出力する場合は、ここで実際にLCDへの書き込みを行う
  // 今回は実機がないため、Serial出力で内容を確認
  Serial.println("----- LCD 表示内容 -----");
  Serial.println(line1);
  Serial.println(line2);
  Serial.println("-------------------------");
}

// ------------------------------------------------
// 統合ハンドラ: ログインフォーム表示と施錠/解錠画面表示
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
    formPage += "    <form method=\"POST\" action=\"/login\" onsubmit=\"hashPass();\">\n";
    formPage += "      <input name=\"id\" id=\"uid\" placeholder=\"User ID\" autocomplete=\"username\" required>\n";
    formPage += "      <input name=\"password\" id=\"pass\" type=\"password\" placeholder=\"Password\" autocomplete=\"current-password\" required>\n";
    formPage += "      <input type=\"hidden\" name=\"passHash\" id=\"passHashField\">\n";
    formPage += "      <button type=\"submit\">Login</button>\n";
    formPage += "    </form>\n";
    formPage += "  </div>\n";
    formPage += "  <script>\n";
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
  // 初回のDuckDNS更新時刻を記録
  lastDuckdnsUpdate = millis();
  
  // ※玄関前の表示用に、定期的にOpenWeather APIから天気情報を取得し、LCD表示用内容を生成する
  updateWeather();
  lastWeatherUpdate = millis();
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
  // OpenWeather APIの情報取得（指定間隔ごと）
  if ((now - lastWeatherUpdate) >= WEATHER_UPDATE_INTERVAL) {
    updateWeather();
    lastWeatherUpdate = now;
  }
}
