/***************************************************************
 *  Spresense アドオンボード用 GNSS + BME280 + MPU6050
 *  取得データを SD カードへ直接書き込み（バッファ未使用）
 *
 *  - GNSS: Spresense アドオンボードの GNSS モジュールを使用
 *    -> "GNSS.addon.h" + "GNSS" クラス
 *    -> UART1 (TX: D1, RX: D0)で通信
 *  - センサ: BME280 (I2C), MPU6050 (I2C)
 *  - データは取得のたびに SD カードに即書き込み
 ***************************************************************/

#include <Arduino.h>
#include <SDHCI.h>               // SDカード制御クラス (SpresenseのSDHCIライブラリ)
#include <File.h>                // ファイルクラス (SDカード上のファイル操作)
#include <Wire.h>                // I2C通信 (BME280, MPU6050で使用)
#include <Adafruit_Sensor.h>     // Adafruitセンサー基底クラス
#include <Adafruit_BME280.h>     // BME280センサ制御クラス
#include <MPU6050.h>             // MPU6050制御クラス
#include <GNSS.addon.h>          // Spresenseアドオンボード用のGNSSライブラリ

//==============================================================
// 定義・定数
//==============================================================

// ファイル名の定義（拡張子含む）
#define CSV_BASE_FILENAME "sensor_data"
#define CSV_EXTENSION ".csv"
#define LOG_BASE_FILENAME "event_log"
#define LOG_EXTENSION ".txt"

// LEDピンの定義
#define PIN_LED0 2 // 動作インジケータLED (アクティブLED)
#define PIN_LED1 3 // GNSSがFixしたかどうかを示すLED
#define PIN_LED3 4 // エラー表示用LED

// センサー更新インターバル（ミリ秒）
#define BME280_INTERVAL 50         // 50msごとにBME280を取得
#define MPU6050_INTERVAL 50        // 50msごとにMPU6050を取得
#define SERIAL_PRINT_INTERVAL 1000 // 1秒間隔でシリアル出力

// GNSS通信ポート設定（SpresenseアドオンボードはUART1を利用）
#define SerialGNSS Serial1  // TX: D1, RX: D0

//==============================================================
// グローバル変数
//==============================================================

// ----- タイミング制御用 -----
unsigned long previousBme280Millis = 0;
unsigned long previousMpu6050Millis = 0;
unsigned long previousSerialMillis = 0;

// ----- SDカード関連 -----
SDClass SD;          // SDカード制御用クラスのインスタンス
File myFile;         // 書き込みや読み込みに使うFileオブジェクト

// ----- GNSS関連 (アドオンボード) -----
/*
  GNSSクラス:
   - SpresenseアドオンボードのGNSS機能を制御するためのクラス
   - UART通信でNMEAなどを受信し、位置情報や衛星情報を取得する
 */
GNSS Gnss;

// ----- BME280センサ関連 -----
/*
  Adafruit_BME280クラス:
   - BME280(温度・湿度・気圧)を制御するAdafruit製ライブラリのクラス
   - begin() で初期化、readTemperature() 等でセンサ値を読み取る
 */
Adafruit_BME280 bme;

// ----- MPU6050センサ関連 -----
/*
  MPU6050クラス:
   - MPU6050(6軸加速度+ジャイロ)を制御するライブラリクラス
   - initialize()で初期化、getMotion6()等で加速度・ジャイロ値を取得
 */
MPU6050 mpu;

// ----- GNSSから取得した最新データを入れる構造体 -----
/*
  GNSSLocation:
   - Gnss.read()後にGnss.getLocation()で得られる位置情報を保持
   - latitude, longitude, altitude, fix, satellitesなどを持つ
 */
GNSSLocation gnssData;

// ----- その他 -----
unsigned long startTime = 0;      // 起動時刻(ミリ秒)を格納
char currentCSVFilename[32] = {0}; // CSVファイル名
char currentLOGFilename[32] = {0}; // ログファイル名

//==============================================================
// 関数プロトタイプ宣言
//==============================================================

// 初期化関連
void initGNSS();
void initBME280();
void initMPU6050();
void initCSV();

// センサー読み取り & SDカード書き込み
void readAndLogSensors();

// ログ・エラー処理
void event(String event_msg);
void errorLoop(int num);

// LED制御
void Led_isActive();
void Led_isPosfix(bool state);
void Led_isError(bool state);

// デバッグ出力用
void printSensorDataToSerial(
  float time_s,
  float temperature, float humidity, float pressure,
  float latitude, float longitude, float altitude,
  int fix, int satellites,
  float accelX, float accelY, float accelZ,
  float gyroX, float gyroY, float gyroZ
);

//==============================================================
// setup()
//==============================================================
void setup() {
  // シリアルモニタ初期化
  Serial.begin(115200);
  while (!Serial) {
    // シリアルの初期化待ち（USB接続環境で有効）
  }

  // LEDピンの初期化
  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);

  // I2C開始
  Wire.begin();

  // 起動時刻を記録
  startTime = millis();

  // GNSS / BME280 / MPU6050初期化
  initGNSS();
  initBME280();
  initMPU6050();

  // SDカード初期化
  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed.");
    errorLoop(1);
  }

  // CSVファイルの作成 & ヘッダー書き込み
  initCSV();

  // ログに初期化完了の記録
  event("All initialization completed");
}

//==============================================================
// loop()
//==============================================================
void loop() {
  unsigned long currentMillis = millis();

  // LED点滅: ループが動いていることを示す
  Led_isActive();

  // 1) GNSS更新
  //    アドオンボード用 GNSSライブラリは Gnss.read() でデータを読み込み
  //    戻り値 trueなら新しいデータを受信 → gnssDataに反映
  if (Gnss.read()) {
    gnssData = Gnss.getLocation();
    // FixしていればLED点灯
    Led_isPosfix(gnssData.fix);
  }

  // 2) BME280更新タイミング
  if (currentMillis - previousBme280Millis >= BME280_INTERVAL) {
    previousBme280Millis = currentMillis;
    // 実際の読み取りは後段の readAndLogSensors() で行う
  }

  // 3) MPU6050更新タイミング
  if (currentMillis - previousMpu6050Millis >= MPU6050_INTERVAL) {
    previousMpu6050Millis = currentMillis;
  }

  // 4) センサーを読み取り、SDに記録（バッファを使わず直接書き込み）
  readAndLogSensors();

  // 5) 一定間隔でシリアル出力 (1秒おき)
  if (currentMillis - previousSerialMillis >= SERIAL_PRINT_INTERVAL) {
    previousSerialMillis = currentMillis;
    // 必要に応じてシリアル出力を追加
    // この例では readAndLogSensors() の中でセンサーデータを
    // すでにシリアル出力しているので、ここでは特に何もしない
  }
}

//==============================================================
// GNSS初期化 (アドオンボード用)
//==============================================================
void initGNSS() {
  // UART1設定
  SerialGNSS.begin(115200);
  delay(100);

  // GNSSライブラリ初期化
  // -> GNSSモジュールとの通信開始
  if (!Gnss.begin(SerialGNSS)) {
    Serial.println("Error: GNSS initialization failed.");
    errorLoop(5);
  }

  Serial.println("GNSS setup OK");
}

//==============================================================
// BME280初期化
//==============================================================
void initBME280() {
  // BME280をI2Cアドレス0x76で開始
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
    errorLoop(3);
  }
}

//==============================================================
// MPU6050初期化
//==============================================================
void initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    errorLoop(4);
  }
}

//==============================================================
// CSVファイル作成 & ヘッダー書き込み
//==============================================================
void initCSV() {
  // シンプルに固定ファイル名として生成
  snprintf(currentCSVFilename, sizeof(currentCSVFilename), "%s%s", CSV_BASE_FILENAME, CSV_EXTENSION);
  snprintf(currentLOGFilename, sizeof(currentLOGFilename), "%s%s", LOG_BASE_FILENAME, LOG_EXTENSION);

  // CSVファイルを作成し、ヘッダを書き込む
  File csvFile = SD.open(currentCSVFilename, FILE_WRITE);
  if (csvFile) {
    // CSVのヘッダ行
    csvFile.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s");
    csvFile.close();
    Serial.print("Logging to: ");
    Serial.println(currentCSVFilename);
  } else {
    Serial.println("Error: Could not create CSV file");
    errorLoop(2);
  }

  // ログファイルも作成
  File logFile = SD.open(currentLOGFilename, FILE_WRITE);
  if (logFile) {
    // ログファイルにもヘッダを入れる（必要に応じて）
    logFile.println("Time_s: Event");
    logFile.close();
    Serial.print("Logging events to: ");
    Serial.println(currentLOGFilename);
  } else {
    Serial.println("Error: Could not create log file");
    errorLoop(2);
  }
}

//==============================================================
// センサー読み取り → SDカードへ直接書き込み
//==============================================================
void readAndLogSensors() {
  // 経過時間（秒）
  float nowSec = (millis() - startTime) / 1000.0;

  // 1) BME280データ取得
  float temperature = bme.readTemperature();      // 温度 (℃)
  float humidity    = bme.readHumidity();         // 湿度 (%)
  float pressure    = bme.readPressure() / 100.0; // 気圧 (hPa)

  // 2) MPU6050データ取得
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX  = gx / 131.0;
  float gyroY  = gy / 131.0;
  float gyroZ  = gz / 131.0;

  // 3) GNSSデータ取得 (Gnss.read()はloop内で実行済み)
  float latitude   = gnssData.latitude;
  float longitude  = gnssData.longitude;
  float altitude   = gnssData.altitude;
  int fix          = gnssData.fix;        // true/false
  int satellites   = gnssData.satellites; // 衛星数

  // 4) CSVファイルを開いて書き込み
  File csvFile = SD.open(currentCSVFilename, FILE_WRITE);
  if (csvFile) {
    csvFile.print(nowSec, 3);      csvFile.print(",");
    csvFile.print(temperature, 2); csvFile.print(",");
    csvFile.print(humidity, 2);    csvFile.print(",");
    csvFile.print(pressure, 2);    csvFile.print(",");
    csvFile.print(latitude, 6);    csvFile.print(",");
    csvFile.print(longitude, 6);   csvFile.print(",");
    csvFile.print(altitude, 2);    csvFile.print(",");
    csvFile.print(fix);            csvFile.print(",");
    csvFile.print(satellites);     csvFile.print(",");
    csvFile.print(accelX, 4);      csvFile.print(",");
    csvFile.print(accelY, 4);      csvFile.print(",");
    csvFile.print(accelZ, 4);      csvFile.print(",");
    csvFile.print(gyroX, 4);       csvFile.print(",");
    csvFile.print(gyroY, 4);       csvFile.print(",");
    csvFile.println(gyroZ, 4);     // printlnで行末

    csvFile.close();
  } else {
    Serial.println("Error: Could not open CSV file for writing");
    // 必要に応じてエラー処理
  }

  // 5) シリアルモニタ表示（確認用）
  printSensorDataToSerial(
    nowSec,
    temperature, humidity, pressure,
    latitude, longitude, altitude,
    fix, satellites,
    accelX, accelY, accelZ,
    gyroX, gyroY, gyroZ
  );
}

//==============================================================
// イベントログ出力
//==============================================================
void event(String event_msg) {
  float t = (millis() - startTime) / 1000.0;
  String s = String(t, 3) + ": " + event_msg;
  Serial.println(s);

  // ログファイルに追記
  File lf = SD.open(currentLOGFilename, FILE_WRITE);
  if (lf) {
    lf.println(s);
    lf.close();
  } else {
    Serial.println("Error: Could not open log file");
  }
}

//==============================================================
// エラー時にLEDを点滅して停止し続ける
//==============================================================
void errorLoop(int num) {
  // 例: num=3 なら、(3回点滅→1秒待ち)を繰り返す
  while (1) {
    for (int i = 0; i < num; i++) {
      digitalWrite(PIN_LED3, HIGH);
      delay(300);
      digitalWrite(PIN_LED3, LOW);
      delay(300);
    }
    delay(1000);
  }
}

//==============================================================
// LED制御
//==============================================================

// ループ動作中: 500msごとにトグル
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

// GNSS Fix状態をLEDで表示: fix==trueならLED点灯
void Led_isPosfix(bool state) {
  digitalWrite(PIN_LED1, state ? HIGH : LOW);
}

// エラー時LED: HIGHで点灯
void Led_isError(bool state) {
  digitalWrite(PIN_LED3, state ? HIGH : LOW);
}

//==============================================================
// シリアルモニタ用にセンサーデータを出力 (デバッグ表示)
//==============================================================
void printSensorDataToSerial(
  float time_s,
  float temperature, float humidity, float pressure,
  float latitude, float longitude, float altitude,
  int fix, int satellites,
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
