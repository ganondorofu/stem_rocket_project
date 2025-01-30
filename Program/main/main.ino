/***************************************************************
 *  Spresense アドオンボード用 GNSS + BME280 + MPU6050
 *  取得データを SD カードへ直接書き込み（バッファ使用）
 *  GNSSは非同期処理 (waitUpdate(0)) で、ほかの処理をブロックしないようにする
 *
 *  変更点:
 *    1) 既存ファイルがあれば連番付きファイル名を生成
 *    2) センサーデータを10回分バッファにため、まとめて書き込む
 *    3) 書き込みエラーがあった場合はバッファを消さずに保持し、次回以降リトライ
 *    4) 書き込み完了後にイベントログへ「バッファ10回分を書き込み完了」と記録
 *    5) bool continueOnError を追加。trueならエラーでも停止せず続行、falseなら停止
 *
 *    さらに今回の変更:
 *    - パラシュート展開のロジック削除
 *    - MPU6050_INTERVAL を 10ms (100Hz) に変更, BME280は50ms, GNSSは10Hz
 *    - SDカード書き込みに失敗したら、以後は内蔵Flashに保存する
 *    - イベントログ (event()) も同様に、SDエラー後はFlashに保存
 *    - 起動時に Flash をフォーマット (リセット)
 ***************************************************************/

#include <Arduino.h>
#include <SDHCI.h>               // SDカード制御クラス (SpresenseのSDHCIライブラリ)
#include <File.h>                // ファイルクラス (SDカード上のファイル操作)
#include <Wire.h>                // I2C通信 (BME280, MPU6050で使用)
#include <Adafruit_Sensor.h>     // Adafruitセンサー基底クラス
#include <Adafruit_BME280.h>     // BME280センサ制御クラス
#include <MPU6050.h>             // MPU6050制御クラス
#include <GNSS.h>                // Spresense Add-on用 GNSSライブラリ (SpGnssAddon)

//======================================================================
// ★ エラー発生時に停止するかどうかのフラグ
//======================================================================
bool continueOnError = true; // true: エラーでも止まらず続行 / false: エラー時に停止

//======================================================================
// ファイル名関連定数
//======================================================================
#define CSV_BASE     "sensor_data"   
#define CSV_EXT      ".csv"
#define LOG_BASE     "event_log"     
#define LOG_EXT      ".txt"

//======================================================================
// LEDピン定義
//======================================================================
#define PIN_LED0 2 // 動作インジケータLED (アクティブLED)
#define PIN_LED1 3 // GNSSがFixしたかどうかを示すLED
#define PIN_LED3 4 // エラー表示用LED

//======================================================================
// センサー更新インターバル（ミリ秒）
//======================================================================
#define BME280_INTERVAL       50     // 50msごとにBME280を取得
#define MPU6050_INTERVAL      10     // 10msごとにMPU6050を取得(100Hz)
#define SERIAL_PRINT_INTERVAL 1000   // 1秒間隔でシリアル出力

// GNSS再起動サイクル (任意) - 今回は使用しない
#define RESTART_CYCLE (60 * 5)       // 例: 5分(300秒)

//======================================================================
// まとめ書き込み時に使用するバッファ設定
//======================================================================
#define BATCH_SIZE 10   // 10回分のセンサーデータをまとめて書き込み

//======================================================================
// グローバル変数
//======================================================================
unsigned long previousBme280Millis = 0;
unsigned long previousMpu6050Millis = 0;
unsigned long previousSerialMillis = 0;
unsigned long startTime = 0;

static int LoopCount = 0;

SDClass SD;
String csvFilename;   // 実際に使用するCSVファイル名
String logFilename;   // 実際に使用するLOGファイル名

// GNSS, BME280, MPU6050
SpGnssAddon Gnss;
Adafruit_BME280 bme;
MPU6050 mpu;

// GNSSデータ構造体
SpNavData gnssData;  

//======================================================================
// 10回分の行データを保持するバッファ
//======================================================================
static String csvBuffer[BATCH_SIZE]; 
static int batchIndex = 0; // バッファ内にいくつ溜まっているか

// SDカード書き込みに一度でも失敗したら true
bool sdErrorHappened = false;

//======================================================================
// Flash用: 今回は疑似コードで実装
//======================================================================
bool flashInitialized = false;

//======================================================================
// 関数プロトタイプ
//======================================================================
int getNextFileIndex(); // CSV/LOGファイルの連番を決定
void initGNSS();
// void restartGNSS(); // 今回は使用しないのでコメントアウト
void initBME280();
void initMPU6050();
void initSDandCSV();

// "SD" 書き込み → エラー発生時は sdErrorHappened = true
void readAndLogSensors();
void flushCsvBuffer();

// イベントログ
void event(String event_msg);

// エラーハンドリング
void handleError(int errCode);
void errorLoop(int num);

// LED制御
void Led_isActive();
void Led_isPosfix(bool state);
void Led_isError(bool state);

void printSensorDataToSerial(
  float time_s,
  float temperature, float humidity, float pressure,
  float latitude, float longitude, float altitude,
  bool fix, int satellites,
  float accelX, float accelY, float accelZ,
  float gyroX, float gyroY, float gyroZ
);

//======================================================================
// Flash関連 (疑似コード)
//======================================================================
void initFlash();                   // 起動時にフォーマット(初期化)
bool writeLineToFlash(String line); // 一行追加
bool writeEventToFlash(String line);// イベントログ追加

//======================================================================
// setup()
//======================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  Wire.begin();

  // 起動時に内蔵Flashをフォーマット(初期化)
  initFlash();

  startTime = millis();

  // 初期化処理
  initGNSS();
  initBME280();
  initMPU6050();
  initSDandCSV();

  event("All initialization completed.");
}

//======================================================================
// loop()
//======================================================================
void loop() {
  unsigned long currentMillis = millis();

  // LED点滅(ループ動作インジケータ)
  Led_isActive();

  // GNSSデータを非ブロッキングで更新チェック
  if (Gnss.waitUpdate(0)) {
    Gnss.getNavData(&gnssData);
    bool fixState = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
    Led_isPosfix(fixState);
  }

  // センサー取得タイミングチェック
  if (currentMillis - previousBme280Millis >= BME280_INTERVAL) {
    previousBme280Millis = currentMillis;
  }
  if (currentMillis - previousMpu6050Millis >= MPU6050_INTERVAL) {
    previousMpu6050Millis = currentMillis;
  }

  // センサー読み取り & バッファ追記
  readAndLogSensors();

  // シリアル出力タイミング (例: 1秒おき)
  if (currentMillis - previousSerialMillis >= SERIAL_PRINT_INTERVAL) {
    previousSerialMillis = currentMillis;
    // 今回は readAndLogSensors() 内ですでにシリアル出力済み
  }

  // LoopCount++ など任意の処理があれば追加
}

//======================================================================
// CSV/LOGファイル共通の連番を探し、両方存在しないindexを返す
//======================================================================
int getNextFileIndex() {
  int idx = 0;
  while (true) {
    String csvCandidate;
    String logCandidate;
    if (idx == 0) {
      csvCandidate = String(CSV_BASE) + String(CSV_EXT);
      logCandidate = String(LOG_BASE) + String(LOG_EXT);
    } else {
      csvCandidate = String(CSV_BASE) + String(idx) + String(CSV_EXT);
      logCandidate = String(LOG_BASE) + String(idx) + String(LOG_EXT);
    }

    bool csvExist = SD.exists(csvCandidate);
    bool logExist = SD.exists(logCandidate);
    if (!csvExist && !logExist) {
      // どちらも存在しなければこれを使う
      return idx;
    }
    idx++;
  }
}

//======================================================================
// GNSS初期化
//======================================================================
void initGNSS() {
  int result = Gnss.begin();
  if (result != 0) {
    Serial.println("Error: Gnss begin error!!");
    handleError(5);
  }

  result = Gnss.start();
  if (result != 0) {
    Serial.println("Error: Gnss start error!!");
    handleError(5);
  }

  // GNSSを10Hzに設定
  Gnss.setInterval(SpInterval_10Hz);

  Serial.println("GNSS setup OK");
}

/*
//======================================================================
// GNSS再起動 (今回は不要なのでコメントアウト)
//======================================================================
void restartGNSS() {
  Serial.println("Restarting GNSS...");
  if (Gnss.stop() != 0) {
    Serial.println("Gnss stop error!!");
    Led_isError(true);
    return;
  }
  if (Gnss.end() != 0) {
    Serial.println("Gnss end error!!");
    Led_isError(true);
    return;
  }
  if (Gnss.begin() != 0) {
    Serial.println("Gnss begin error!!");
    Led_isError(true);
    return;
  }
  if (Gnss.start() != 0) {
    Serial.println("Gnss start error!!");
    Led_isError(true);
    return;
  }
  // 10Hz
  Gnss.setInterval(SpInterval_10Hz);

  Serial.println("Gnss restart OK.");
}
*/

//======================================================================
// BME280初期化
//======================================================================
void initBME280() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
    handleError(3);
  }
  Serial.println("BME280 setup OK");
}

//======================================================================
// MPU6050初期化 (MPU6050_INTERVAL=10msで実行想定)
//======================================================================
void initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    handleError(4);
  }
  Serial.println("MPU6050 setup OK");
}

//======================================================================
// SDカード初期化 & CSV/LOGファイル作成
//======================================================================
void initSDandCSV() {
  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed.");
    handleError(1);
  }
  Serial.println("SD card setup OK");

  int index = getNextFileIndex();
  if (index == 0) {
    csvFilename = String(CSV_BASE) + String(CSV_EXT);
    logFilename = String(LOG_BASE) + String(LOG_EXT);
  } else {
    csvFilename = String(CSV_BASE) + String(index) + String(CSV_EXT);
    logFilename = String(LOG_BASE) + String(index) + String(LOG_EXT);
  }

  // CSVファイルを作成しヘッダを追加
  File csvFile = SD.open(csvFilename, FILE_WRITE);
  if (csvFile) {
    csvFile.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s");
    csvFile.close();
    Serial.print("Logging to: ");
    Serial.println(csvFilename);
  } else {
    Serial.println("Error: Could not create CSV file");
    handleError(2);
  }

  // ログファイル作成
  File logFile = SD.open(logFilename, FILE_WRITE);
  if (logFile) {
    logFile.println("Time_s:Event");
    logFile.close();
    Serial.print("Logging events to: ");
    Serial.println(logFilename);
  } else {
    Serial.println("Error: Could not create LOG file");
    handleError(2);
  }
}

//======================================================================
// センサー読み取り & バッファ格納
//======================================================================
void readAndLogSensors() {
  float nowSec = (millis() - startTime) / 1000.0;

  // BME280
  float temperature = bme.readTemperature();
  float humidity    = bme.readHumidity();
  float pressure    = bme.readPressure() / 100.0;

  // MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX  = gx / 131.0;
  float gyroY  = gy / 131.0;
  float gyroZ  = gz / 131.0;

  // GNSS
  bool fix = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
  float latitude  = gnssData.latitude;
  float longitude = gnssData.longitude;
  float altitude  = gnssData.altitude;
  int satellites  = gnssData.numSatellites;

  // CSV行の文字列を作成
  String line;
  line += String(nowSec, 3);       line += ",";
  line += String(temperature, 2); line += ",";
  line += String(humidity, 2);    line += ",";
  line += String(pressure, 2);    line += ",";
  line += String(latitude, 6);    line += ",";
  line += String(longitude, 6);   line += ",";
  line += String(altitude, 2);    line += ",";

  // fix (true/false) を "1"/"0"
  line += (fix ? "1" : "0");
  line += ",";

  line += String(satellites);
  line += ",";
  line += String(accelX, 4);   line += ",";
  line += String(accelY, 4);   line += ",";
  line += String(accelZ, 4);   line += ",";
  line += String(gyroX, 4);    line += ",";
  line += String(gyroY, 4);    line += ",";
  line += String(gyroZ, 4);

  // バッファに追加
  if (batchIndex < BATCH_SIZE) {
    csvBuffer[batchIndex] = line;
    batchIndex++;
  } else {
    // バッファが満タン
    Serial.println("Warning: CSV buffer is full, discarding new data");
  }

  // バッファが満タンなら書き込み
  if (batchIndex >= BATCH_SIZE) {
    flushCsvBuffer();
  }

  // シリアル出力 (確認用)
  printSensorDataToSerial(
    nowSec,
    temperature, humidity, pressure,
    latitude, longitude, altitude,
    fix, satellites,
    accelX, accelY, accelZ,
    gyroX, gyroY, gyroZ
  );
}

//======================================================================
// バッファをファイルに一括書き込み
//======================================================================
void flushCsvBuffer() {
  if (sdErrorHappened) {
    // 既に SDエラーになった → Flashに書き込み
    // Flashに書き込んでバッファクリア
    for (int i = 0; i < batchIndex; i++) {
      // Flashへ書き込み（疑似関数）
      if (!writeLineToFlash(csvBuffer[i])) {
        Serial.println("Flash write error!");
        handleError(9);
        return;
      }
    }
    batchIndex = 0;
    // イベントログにもFlashへ書く
    String msg = "Flushed " + String(BATCH_SIZE) + " lines to FLASH";
    Serial.println(msg);
    writeEventToFlash(msg); // Flash上にイベント保存
    return;
  }

  // SD書き込みを試みる
  File csvFile = SD.open(csvFilename, FILE_WRITE);
  if (!csvFile) {
    Serial.println("Error: Could not open CSV file for writing");
    handleError(2);
    // 以後Flashに切り替える
    sdErrorHappened = true;
    return;
  }

  for (int i = 0; i < batchIndex; i++) {
    csvFile.println(csvBuffer[i]);
  }
  csvFile.close();

  // バッファをクリア
  batchIndex = 0;

  // 書き込み完了をイベントログに残す
  String msg = "Flushed " + String(BATCH_SIZE) + " lines to CSV";
  event(msg);
}

//======================================================================
// イベントログ書き込み
//======================================================================
void event(String event_msg) {
  float t = (millis() - startTime) / 1000.0;
  String s = String(t, 3) + ": " + event_msg;
  Serial.println(s);

  // SDエラー後はFlashに記録
  if (sdErrorHappened) {
    writeEventToFlash(s);
    return;
  }

  // SDへ書き込み
  File lf = SD.open(logFilename, FILE_WRITE);
  if (lf) {
    lf.println(s);
    lf.close();
  } else {
    Serial.println("Error: Could not open log file");
    handleError(2);
    // SDエラー発生
    sdErrorHappened = true;
  }
}

//======================================================================
// エラーハンドリング: continueOnErrorフラグにより停止or継続
//======================================================================
void handleError(int errCode) {
  if (continueOnError) {
    // エラーLEDを点灯しつつ続行
    Led_isError(true);
    Serial.print("Warning: error code=");
    Serial.println(errCode);
  } else {
    // 停止
    errorLoop(errCode);
  }
}

//======================================================================
// エラー時にLED点滅して停止
//======================================================================
void errorLoop(int num) {
  while (true) {
    for (int i = 0; i < num; i++) {
      digitalWrite(PIN_LED3, HIGH);
      delay(300);
      digitalWrite(PIN_LED3, LOW);
      delay(300);
    }
    delay(1000);
  }
}

//======================================================================
// LED制御
//======================================================================
void Led_isActive() {
  static unsigned long lastToggle = 0;
  static bool state = false;
  unsigned long current = millis();
  if (current - lastToggle >= 500) {
    state = !state;
    digitalWrite(PIN_LED0, state ? HIGH : LOW);
    lastToggle = current;
  }
}

void Led_isPosfix(bool state) {
  digitalWrite(PIN_LED1, state ? HIGH : LOW);
}

void Led_isError(bool state) {
  digitalWrite(PIN_LED3, state ? HIGH : LOW);
}

//======================================================================
// デバッグ用シリアル出力
//======================================================================
void printSensorDataToSerial(
  float time_s,
  float temperature, float humidity, float pressure,
  float latitude, float longitude, float altitude,
  bool fix, int satellites,
  float accelX, float accelY, float accelZ,
  float gyroX, float gyroY, float gyroZ
) {
  Serial.print("Time: ");
  Serial.print(time_s, 3);
  Serial.print(" s, Temp: ");
  Serial.print(temperature, 2);
  Serial.print(" C, Humi: ");
  Serial.print(humidity, 2);
  Serial.print(" %, Press: ");
  Serial.print(pressure, 2);
  Serial.print(" hPa, Lat: ");
  Serial.print(latitude, 6);
  Serial.print(", Lng: ");
  Serial.print(longitude, 6);
  Serial.print(", Alt: ");
  Serial.print(altitude, 2);
  Serial.print(" m, Fix: ");
  Serial.print(fix);
  Serial.print(", Sats: ");
  Serial.print(satellites);

  Serial.print(", Accel(g): ");
  Serial.print(accelX, 4); Serial.print("/");
  Serial.print(accelY, 4); Serial.print("/");
  Serial.print(accelZ, 4);

  Serial.print(", Gyro(deg/s): ");
  Serial.print(gyroX, 4); Serial.print("/");
  Serial.print(gyroY, 4); Serial.print("/");
  Serial.print(gyroZ, 4);

  Serial.println();
}

//======================================================================
// Flash関連（疑似実装）
//======================================================================

// 起動時にFlashをフォーマット(全消去)する
void initFlash() {
  // ここでは疑似的に「flashInitialized = true」にするのみ
  // 実際にはLittleFSなどをフォーマットする処理が必要
  Serial.println("Formatting internal Flash... (Pseudo)");
  flashInitialized = true;
  // 例: 
  //   LittleFS.format(); 
  //   LittleFS.begin();
}

bool writeLineToFlash(String line) {
  if (!flashInitialized) return false;

  // ここでは単にシリアルに出力するのみ
  // 実際にはLittleFSなどに追記する処理を書く
  Serial.print("[FLASH] ");
  Serial.println(line);

  return true;
}

bool writeEventToFlash(String line) {
  if (!flashInitialized) return false;

  // イベントログをFlashに記録（疑似）
  Serial.print("[FLASH EVENT] ");
  Serial.println(line);

  return true;
}
