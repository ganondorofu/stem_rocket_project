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
 *    6) パラシュート展開のロジック削除
 *    7) MPU6050_INTERVAL = 10ms (100Hz), BME280_INTERVAL=50ms, GNSS=10Hz
 *    8) SDエラー後は内蔵Flashへ書き込み
 *    9) eventログもFlashへ書き込み
 *    10) **起動時にFlashをフォーマットするかどうかを変数で制御 (formatFlashOnBoot)**
 ***************************************************************/

#include <Arduino.h>
#include <SDHCI.h>           // SDカード制御 (Spresense)
#include <File.h>            // SDカードファイル操作
#include <Wire.h>            // I2C (BME280, MPU6050)
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> // BME280
#include <MPU6050.h>         // MPU6050
#include <GNSS.h>            // GNSS Addon
#include <Flash.h>           // SpresenseのFlash制御クラス

/***************************************************************
 * ユーザー設定: 起動時にFlashをフォーマットするかどうか
 * true  -> 起動時にFlash.format()を実行（前回データ消去）
 * false -> Flash.format()をスキップ（前回データ保持）
 ***************************************************************/
bool formatFlashOnBoot = true;

/***************************************************************
 * エラー発生時に停止するかどうか
 ***************************************************************/
bool continueOnError = false;

/***************************************************************
 * ファイル名関連定数
 ***************************************************************/
#define CSV_BASE   "sensor_data"
#define CSV_EXT    ".csv"
#define LOG_BASE   "event_log"
#define LOG_EXT    ".txt"

/***************************************************************
 * LEDピン定義
 ***************************************************************/
#define PIN_LED0 2 // 動作インジケータLED
#define PIN_LED1 3 // GNSS Fix LED
#define PIN_LED3 4 // エラー表示用LED

/***************************************************************
 * センサー更新インターバル
 ***************************************************************/
#define BME280_INTERVAL       50    // 50ms (20Hz)
#define MPU6050_INTERVAL      10    // 10ms (100Hz)
#define SERIAL_PRINT_INTERVAL 1000  // 1秒おきにシリアル出力

/***************************************************************
 * GNSSを10Hzに設定
 ***************************************************************/
#define GNSS_RATE SpInterval_10Hz

/***************************************************************
 * バッファ設定
 ***************************************************************/
#define BATCH_SIZE 10

/***************************************************************
 * グローバル変数
 ***************************************************************/
unsigned long previousBme280Millis = 0;
unsigned long previousMpu6050Millis = 0;
unsigned long previousSerialMillis = 0;
unsigned long startTime = 0;

SDClass SD;            // SDカード
SpGnssAddon Gnss;      // GNSSインスタンス
Adafruit_BME280 bme;   // BME280
MPU6050 mpu;           // MPU6050
SpNavData gnssData;    // GNSSデータ

// SDカード用CSV/LOGファイル名
String csvFilename;
String logFilename;

// 10回分の行データバッファ
static String csvBuffer[BATCH_SIZE];
static int batchIndex = 0;

// SDエラーが一度でも起きたか
bool sdErrorHappened = false;

// Flash初期化済みかどうか
bool flashInitialized = false;

/***************************************************************
 * 関数プロトタイプ
 ***************************************************************/
int  getNextFileIndex();
void initFlash();
void initGNSS();
void initBME280();
void initMPU6050();
void initSDandCSV();

void readAndLogSensors();
void flushCsvBuffer();
void event(String msg);
void handleError(int errCode);
void errorLoop(int num);

// LED
void Led_isActive();
void Led_isPosfix(bool state);
void Led_isError(bool state);

// デバッグ用
void printSensorDataToSerial(
  float time_s,
  float temperature, float humidity, float pressure,
  float latitude, float longitude, float altitude,
  bool fix, int satellites,
  float accelX, float accelY, float accelZ,
  float gyroX, float gyroY, float gyroZ
);

/***************************************************************
 * setup()
 ***************************************************************/
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  Wire.begin();

  // 起動時にFlash初期化
  initFlash();

  startTime = millis();

  // 各種初期化
  initGNSS();
  initBME280();
  initMPU6050();
  initSDandCSV();

  event("All initialization completed.");
}

/***************************************************************
 * loop()
 ***************************************************************/
void loop() {
  unsigned long currentMillis = millis();

  // LED点滅
  Led_isActive();

  // GNSS更新(非ブロッキング)
  if (Gnss.waitUpdate(0)) {
    Gnss.getNavData(&gnssData);
    bool fixState = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
    Led_isPosfix(fixState);
  }

  // センサー取得タイミング
  if (currentMillis - previousBme280Millis >= BME280_INTERVAL) {
    previousBme280Millis = currentMillis;
  }
  if (currentMillis - previousMpu6050Millis >= MPU6050_INTERVAL) {
    previousMpu6050Millis = currentMillis;
  }

  // センサー読み取り & バッファ追記
  readAndLogSensors();

  // シリアル出力
  if (currentMillis - previousSerialMillis >= SERIAL_PRINT_INTERVAL) {
    previousSerialMillis = currentMillis;
    // readAndLogSensors() 内で既に出力済み
  }
}

/***************************************************************
 * Flash初期化
 *   formatFlashOnBoot = true ならフォーマット
 *   その後 Flash.begin() でマウント
 ***************************************************************/
void initFlash() {
  if (formatFlashOnBoot) {
    Serial.println("Formatting internal Flash... (on boot)");
    if (!Flash.format()) {
      Serial.println("Warning: Flash.format() failed!");
    } else {
      Serial.println("Flash format done.");
    }
  } else {
    Serial.println("Skipping Flash format on boot.");
  }

  if (!Flash.begin()) {
    Serial.println("Warning: Flash.begin() failed!");
  } else {
    Serial.println("Flash.begin() success.");
  }
  flashInitialized = true;
}

/***************************************************************
 * GNSS初期化 (10Hz)
 ***************************************************************/
void initGNSS() {
  int ret = Gnss.begin();
  if (ret != 0) {
    Serial.println("Error: Gnss.begin() fail");
    handleError(5);
  }
  Gnss.setInterval(GNSS_RATE);
  ret = Gnss.start();
  if (ret != 0) {
    Serial.println("Error: Gnss.start() fail");
    handleError(5);
  }
  Serial.println("GNSS setup OK (10Hz)");
}

/***************************************************************
 * BME280初期化 (50ms => 20Hz)
 ***************************************************************/
void initBME280() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
    handleError(3);
  }
  Serial.println("BME280 setup OK (50ms)");
}

/***************************************************************
 * MPU6050初期化 (10ms => 100Hz)
 ***************************************************************/
void initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    handleError(4);
  }
  Serial.println("MPU6050 setup OK (10ms)");
}

/***************************************************************
 * SDカード初期化 & CSV/LOGファイル作成
 ***************************************************************/
void initSDandCSV() {
  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed.");
    handleError(1);
    sdErrorHappened = true; // 以後Flashへ
    return;
  }
  Serial.println("SD card setup OK");

  int idx = getNextFileIndex();
  String csvCandidate, logCandidate;
  if (idx == 0) {
    csvCandidate = String(CSV_BASE) + String(CSV_EXT);
    logCandidate = String(LOG_BASE) + String(LOG_EXT);
  } else {
    csvCandidate = String(CSV_BASE) + String(idx) + String(CSV_EXT);
    logCandidate = String(LOG_BASE) + String(idx) + String(LOG_EXT);
  }

  // CSVファイル
  File csvFile = SD.open(csvCandidate, FILE_WRITE);
  if (csvFile) {
    csvFile.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s");
    csvFile.close();
    csvFilename = csvCandidate;
    Serial.print("Logging to: ");
    Serial.println(csvFilename);
  } else {
    Serial.println("Error: Could not create CSV file");
    handleError(2);
    sdErrorHappened = true;
  }

  // LOGファイル
  File lf = SD.open(logCandidate, FILE_WRITE);
  if (lf) {
    lf.println("Time_s:Event");
    lf.close();
    logFilename = logCandidate;
    Serial.print("Logging events to: ");
    Serial.println(logFilename);
  } else {
    Serial.println("Error: Could not create LOG file");
    handleError(2);
    sdErrorHappened = true;
  }
}

/***************************************************************
 * CSV/LOGファイル共通の連番を探し、両方存在しないindexを返す
 ***************************************************************/
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
      return idx;
    }
    idx++;
  }
}

/***************************************************************
 * センサー読み取り & バッファ格納
 ***************************************************************/
void readAndLogSensors() {
  float nowSec = (millis() - startTime) / 1000.0f;

  // BME280
  float temperature = bme.readTemperature();
  float humidity    = bme.readHumidity();
  float pressure    = bme.readPressure() / 100.0f;

  // MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;
  float gyroX  = gx / 131.0f;
  float gyroY  = gy / 131.0f;
  float gyroZ  = gz / 131.0f;

  // GNSS
  bool fix = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
  float latitude  = gnssData.latitude;
  float longitude = gnssData.longitude;
  float altitude  = gnssData.altitude;
  int satellites  = gnssData.numSatellites;

  // CSV行を作る
  String line;
  line += String(nowSec, 3);       line += ",";
  line += String(temperature, 2); line += ",";
  line += String(humidity, 2);    line += ",";
  line += String(pressure, 2);    line += ",";
  line += String(latitude, 6);    line += ",";
  line += String(longitude, 6);   line += ",";
  line += String(altitude, 2);    line += ",";

  line += (fix ? "1" : "0");
  line += ",";

  line += String(satellites);
  line += ",";
  line += String(accelX, 4);      line += ",";
  line += String(accelY, 4);      line += ",";
  line += String(accelZ, 4);      line += ",";
  line += String(gyroX, 4);       line += ",";
  line += String(gyroY, 4);       line += ",";
  line += String(gyroZ, 4);

  // バッファに追加
  if (batchIndex < BATCH_SIZE) {
    csvBuffer[batchIndex] = line;
    batchIndex++;
  } else {
    Serial.println("Warning: CSV buffer is full, discarding new data");
  }

  // バッファが満タンなら書き込み
  if (batchIndex >= BATCH_SIZE) {
    flushCsvBuffer();
  }

  // シリアル出力(確認用)
  printSensorDataToSerial(
    nowSec,
    temperature, humidity, pressure,
    latitude, longitude, altitude,
    fix, satellites,
    accelX, accelY, accelZ,
    gyroX, gyroY, gyroZ
  );
}

/***************************************************************
 * バッファをファイルに一括書き込み
 ***************************************************************/
void flushCsvBuffer() {
  if (batchIndex == 0) return;

  if (sdErrorHappened) {
    // 既にSDでエラーが発生 → Flash書き込み
    File flashFile = Flash.open("sensor_data.csv", FILE_WRITE);
    if (!flashFile) {
      Serial.println("Error: Could not open Flash for sensor_data.csv");
      handleError(9);
      return;
    }
    for (int i = 0; i < batchIndex; i++) {
      flashFile.println(csvBuffer[i]);
    }
    flashFile.close();
    batchIndex = 0;

    Serial.println("Flushed 10 lines to Flash");
    event("Flushed 10 lines to CSV (Flash)");
    return;
  }

  // SDへ書き込み
  File csvFile = SD.open(csvFilename, FILE_WRITE);
  if (!csvFile) {
    Serial.println("Error: Could not open CSV file for writing");
    handleError(2);
    sdErrorHappened = true;
    return;
  }

  for (int i = 0; i < batchIndex; i++) {
    csvFile.println(csvBuffer[i]);
  }
  csvFile.close();
  batchIndex = 0;

  event("Flushed 10 lines to CSV (SD)");
}

/***************************************************************
 * イベントログ書き込み
 ***************************************************************/
void event(String msg) {
  float t = (millis() - startTime) / 1000.0f;
  String s = String(t, 3) + ": " + msg;
  Serial.println(s);

  if (sdErrorHappened) {
    // Flashへ
    File f = Flash.open("event.txt", FILE_WRITE);
    if (!f) {
      Serial.println("Error: Could not open event.txt in Flash");
      handleError(9);
      return;
    }
    f.println(s);
    f.close();
    return;
  }

  // SDへ
  File lf = SD.open(logFilename, FILE_WRITE);
  if (lf) {
    lf.println(s);
    lf.close();
  } else {
    Serial.println("Error: Could not open log file");
    handleError(2);
    sdErrorHappened = true;
  }
}

/***************************************************************
 * エラーハンドリング: continueOnErrorフラグにより停止or継続
 ***************************************************************/
void handleError(int errCode) {
  if (continueOnError) {
    Led_isError(true);
    Serial.print("Warning: error code=");
    Serial.println(errCode);
  } else {
    errorLoop(errCode);
  }
}

/***************************************************************
 * エラー時にLED点滅して停止
 ***************************************************************/
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

/***************************************************************
 * LED制御
 ***************************************************************/
void Led_isActive() {
  static unsigned long lastToggle = 0;
  static bool state = false;
  unsigned long current = millis();
  if (current - lastToggle >= 500) {
    state = !state;
    digitalWrite(PIN_LED0, (state ? HIGH : LOW));
    lastToggle = current;
  }
}

void Led_isPosfix(bool state) {
  digitalWrite(PIN_LED1, state ? HIGH : LOW);
}

void Led_isError(bool state) {
  digitalWrite(PIN_LED3, state ? HIGH : LOW);
}

/***************************************************************
 * デバッグ用シリアル出力
 ***************************************************************/
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
