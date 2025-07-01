/***************************************************************
 * Spresense アドオンボード用 GNSS + BME280 + MPU6050
 * 取得データを SD カードへ直接書き込み（バッファ使用）
 * GNSSは非同期処理 (waitUpdate(0)) で、ほかの処理をブロックしないようにする
 *
 * 変更点:
 *   1) 既存ファイルがあれば連番付きファイル名を生成（CSV, LOG, 前段記録用 befor_flight.csv）
 *   2) センサーデータを複数行分バッファにため、まとめて書き込む（フライト開始後）
 *   3) 書き込みエラーがあった場合はバッファを保持し、次回以降リトライ
 *   4) 書き込み完了後にイベントログへ書き込み
 *   5) bool continueOnError を追加。trueならエラー発生時も処理続行、falseなら停止
 *   6) パラシュート展開のロジック削除 → 今回は自由落下検知により展開する
 *   7) MPU6050_INTERVAL = 10ms (100Hz), BME280_INTERVAL = 50ms, GNSS = 10Hz
 *   8) SDエラー後は内蔵Flashへ書き込み
 *   9) eventログもFlashへ書き込み
 *   10) 起動時にFlashをフォーマットするかを変数で制御 (formatFlashOnBoot)
 *
 * ★ 追加変更点: 前段記録（Pre-flight logging）＋加速度測定
 *   - フライト前もフライト中と同じレート（例：50ms毎）でセンサーデータを取得し、
 *     メモリ上の循環バッファに保存します。バッファは常に直近10秒分のみ保持します。
 *   - フライト検知時、バッファ内の直近10秒分のデータをCSVに出力し、その後は通常のフライト中記録に切り替えます。
 *
 * ★ 高速化のため、センサーデータ取得処理を1回でまとめるための構造体を導入しています。
 * ★ BATCH_SIZE を50に拡大して、SDカードへのアクセス回数を低減しています。
 *
 * ★ フライトピン実装（ピン8,9 使用）は従来通りです。
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
#include <Servo.h>           // サーボ制御

// ★ カメラ・動画関連ライブラリ ★
#include <Camera.h>          // カメラ制御用ライブラリ
#include "AviLibrary.h"      // AVI動画生成用ライブラリ

// ※ カメラ側で使用していたSDカードオブジェクトは、センサ側の SD オブジェクトを利用するため
#define theSD SD

// math.h (sqrt用)
#include <math.h>

/***************************************************************
 * ユーザー設定
 ***************************************************************/
bool formatFlashOnBoot = false;
bool continueOnError = true; // エラー発生時、trueなら処理続行、falseなら停止

/***************************************************************
 * ファイル名関連定数
 ***************************************************************/
#define CSV_BASE       "sensor_data"
#define CSV_EXT        ".csv"
#define LOG_BASE       "event_log"
#define LOG_EXT        ".txt"
#define PRE_FLIGHT_BASE "befor_flight"
#define PRE_FLIGHT_EXT  ".csv"

/***************************************************************
 * LEDピン定義
 ***************************************************************/
#define PIN_LED0 2
#define PIN_LED1 3
#define PIN_LED3 4

/***************************************************************
 * センサー更新インターバル
 ***************************************************************/
#define BME280_INTERVAL       50    // 50ms (20Hz)
#define MPU6050_INTERVAL      10    // 10ms (100Hz)
#define SERIAL_PRINT_INTERVAL 1000  // 1秒間隔

/***************************************************************
 * GNSS設定
 ***************************************************************/
#define GNSS_RATE SpInterval_10Hz

/***************************************************************
 * バッファ設定（フライト中用）
 ***************************************************************/
#define BATCH_SIZE 50    // SDアクセス回数削減のためにバッチサイズを50に拡大

/***************************************************************
 * 前段記録用関連設定
 ***************************************************************/
#define PREFLIGHT_SENSOR_INTERVAL 50   // pre-flightセンサーデータ取得間隔（50ms）
#define PREFLIGHT_BUFFER_MAX 200         // 前段記録バッファの最大件数（概ね10秒分）
unsigned long previousPreFlightSensorMillis = 0;  // 前段記録用タイミング管理

// 前段記録用の循環バッファ（各エントリ：取得時刻とCSV形式データ）
unsigned long preFlightTimeBuffer[PREFLIGHT_BUFFER_MAX];
String preFlightDataBuffer[PREFLIGHT_BUFFER_MAX];
int preFlightBufferCount = 0;

/***************************************************************
 * フライトピン用ピン定義
 ***************************************************************/
#define PIN_FLIGHT_OUTPUT 8
#define PIN_FLIGHT_INPUT  9

// フライトイベントフラグ
bool flightStarted = false;
// flightStartTime: フライトピンが切られた時点（離陸開始時刻）
unsigned long flightStartTime = 0;

/***************************************************************
 * グローバル変数（センサ／ログ）
 ***************************************************************/
unsigned long previousBme280Millis = 0;
unsigned long previousMpu6050Millis = 0;
unsigned long previousSerialMillis = 0;
unsigned long startTime = 0;
SDClass SD;
SpGnssAddon Gnss;
Adafruit_BME280 bme;
MPU6050 mpu;
SpNavData gnssData;
static Servo s_servo;
String csvFilename;
String logFilename;
String preFlightFilename;
static String csvBuffer[BATCH_SIZE];
static int batchIndex = 0;
bool sdErrorHappened = false;
bool flashInitialized = false;

/***************************************************************
 * 追加：前段記録中の加速度平均用変数（そのまま）
 ***************************************************************/
float preFlightAccSum = 0.0;
int preFlightAccCount = 0;

/***************************************************************
 * 追加：パラシュート展開用
 ***************************************************************/
#define FREEFALL_THRESHOLD 0.8  // g 以下なら自由落下とみなす
bool parachuteDeployed = false;
bool apexDetected = false;
uint32_t apexTime = 0;
bool deployPending = false;       // パラシュート展開保留フラグ
unsigned long deployStartTime = 0;  // 展開要求時刻
String deployMethod = "";           // 展開条件の理由

/***************************************************************
 * 追加：気圧トレンド検出用変数
 ***************************************************************/
int consecutivePressureDecrease = 0;
int consecutivePressureIncrease = 0;
float lastPressure = 0.0;
bool ascendingDetected = false;

/***************************************************************
 * センサーデータ構造体定義（高速化のため、1回の取得で全データをまとめて扱う）
 ***************************************************************/
struct SensorData {
  float time_s;
  float temperature;
  float humidity;
  float pressure;
  float latitude;
  float longitude;
  float altitude;
  bool fix;
  int satellites;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float totalAccel;
};

// センサーデータを取得する関数（複数回の呼び出しを防ぐ）
SensorData getSensorData() {
  SensorData data;
  data.time_s = (millis() - startTime) / 1000.0f;
  data.temperature = bme.readTemperature();
  data.humidity    = bme.readHumidity();
  data.pressure    = bme.readPressure() / 100.0f;
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float rawAccelX = ax / 16384.0f;
  float rawAccelY = ay / 16384.0f;
  float rawAccelZ = az / 16384.0f;
  // 補正：センサ取り付け方向に合わせる
  data.accelX = -rawAccelZ;
  data.accelY = rawAccelY;
  data.accelZ = rawAccelX;
  data.gyroX = gx / 131.0f;
  data.gyroY = gy / 131.0f;
  data.gyroZ = gz / 131.0f;
  data.totalAccel = sqrt(data.accelX * data.accelX +
                         data.accelY * data.accelY +
                         data.accelZ * data.accelZ);
  // GNSSデータは既に更新されている前提
  data.fix = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
  data.latitude  = gnssData.latitude;
  data.longitude = gnssData.longitude;
  data.altitude  = gnssData.altitude;
  data.satellites = gnssData.numSatellites;
  return data;
}

// センサーデータ構造体をCSV形式の1行に変換する関数
String sensorDataToCSVLine(const SensorData &data) {
  String line;
  line += String(data.time_s, 3) + ",";
  line += String(data.temperature, 2) + ",";
  line += String(data.humidity, 2) + ",";
  line += String(data.pressure, 2) + ",";
  line += String(data.latitude, 6) + ",";
  line += String(data.longitude, 6) + ",";
  line += String(data.altitude, 2) + ",";
  line += (data.fix ? "1" : "0") + ",";
  line += String(data.satellites) + ",";
  line += String(data.accelX, 4) + ",";
  line += String(data.accelY, 4) + ",";
  line += String(data.accelZ, 4) + ",";
  line += String(data.gyroX, 4) + ",";
  line += String(data.gyroY, 4) + ",";
  line += String(data.gyroZ, 4) + ",";
  line += String(data.totalAccel, 3);
  return line;
}

/***************************************************************
 * recordPreFlightSensorData()
 * 前段記録用にセンサーデータを取得し、循環バッファに保存する関数
 * 古い（10秒以上前の）データは自動的に削除
 ***************************************************************/
void recordPreFlightSensorData() {
  SensorData data = getSensorData();
  String line = sensorDataToCSVLine(data);
  unsigned long currentMillis = millis();
  // バッファに追加（空きがあれば末尾に追加、満杯の場合はシフト）
  if (preFlightBufferCount < PREFLIGHT_BUFFER_MAX) {
    preFlightTimeBuffer[preFlightBufferCount] = currentMillis;
    preFlightDataBuffer[preFlightBufferCount] = line;
    preFlightBufferCount++;
  } else {
    for (int i = 1; i < preFlightBufferCount; i++) {
      preFlightTimeBuffer[i-1] = preFlightTimeBuffer[i];
      preFlightDataBuffer[i-1] = preFlightDataBuffer[i];
    }
    preFlightTimeBuffer[preFlightBufferCount-1] = currentMillis;
    preFlightDataBuffer[preFlightBufferCount-1] = line;
  }
  // 古いデータ（10秒以上前）を削除
  while (preFlightBufferCount > 0 && (currentMillis - preFlightTimeBuffer[0]) > 10000) {
    for (int i = 1; i < preFlightBufferCount; i++) {
      preFlightTimeBuffer[i-1] = preFlightTimeBuffer[i];
      preFlightDataBuffer[i-1] = preFlightDataBuffer[i];
    }
    preFlightBufferCount--;
  }
}

/***************************************************************
 * flushPreFlightBufferToCSV()
 * 前段記録用バッファの内容をCSVファイルへ書き出す関数
 ***************************************************************/
void flushPreFlightBufferToCSV() {
  File csvFile = SD.open(csvFilename, FILE_WRITE);
  if (csvFile) {
    for (int i = 0; i < preFlightBufferCount; i++) {
      csvFile.println(preFlightDataBuffer[i]);
    }
    csvFile.close();
    preFlightBufferCount = 0;
  } else {
    handleError(2);
    sdErrorHappened = true;
  }
}

/***************************************************************
 * readAndLogSensors()
 * フライト中にセンサーデータを取得し、バッファに追加する関数
 ***************************************************************/
void readAndLogSensors() {
  // 1回のセンサーデータ取得で全データをまとめて処理
  SensorData data = getSensorData();
  String line = sensorDataToCSVLine(data);
  
  // 気圧トレンド検出（pressureはdata.pressure）
  if (lastPressure == 0.0) {
    lastPressure = data.pressure;
  } else {
    if (data.pressure < lastPressure) {
      consecutivePressureDecrease++;
      consecutivePressureIncrease = 0;
    } else if (data.pressure > lastPressure) {
      consecutivePressureIncrease++;
      consecutivePressureDecrease = 0;
    }
    lastPressure = data.pressure;
  }
  if (consecutivePressureDecrease >= 5) {
    ascendingDetected = true;
  }
  if (ascendingDetected && consecutivePressureIncrease >= 5) {
    if ((millis() - startTime) >= 1500) {
      event("下降検知（気圧変化による）条件成立");
      deployMethod = "気圧変化による";
      deployParachute();
    } else {
      event("下降検知（発射初期段階のためパラシュート未展開）");
    }
  }
  
  // 加速度による自由落下検知
  if (!parachuteDeployed && data.totalAccel < FREEFALL_THRESHOLD) {
    event("加速度検知（条件成立）：自由落下と判断");
    deployMethod = "加速度検知による";
    deployParachute();
  }
  
  // バッファ（csvBuffer）に追加し、BATCH_SIZE件ごとにSD/Flashへ書き込み
  csvBuffer[batchIndex] = line;
  batchIndex++;
  if (batchIndex >= BATCH_SIZE) {
    flushCsvBuffer();
  }
  
  // シリアル出力（必要なら残すが、処理高速化のため必要最低限に）
  printSensorDataToSerial(
    data.time_s, data.temperature, data.humidity, data.pressure,
    data.latitude, data.longitude, data.altitude,
    data.fix, data.satellites,
    data.accelX, data.accelY, data.accelZ,
    data.gyroX, data.gyroY, data.gyroZ,
    data.totalAccel
  );
  
  // 安全機構：離陸後3秒経過後に自動展開
  if (!parachuteDeployed && (millis() - flightStartTime >= 2000)) {
    event("安全機構条件成立：離陸から3秒経過");
    deployMethod = "安全機構による";
    deployParachute();
  }
}

/***************************************************************
 * flushCsvBuffer()
 * バッファ内のCSVデータをSDカードまたはFlashに書き出す関数
 ***************************************************************/
void flushCsvBuffer() {
  if (batchIndex == 0) return;
  if (sdErrorHappened) {
    File flashFile = Flash.open("sensor_data.csv", FILE_WRITE);
    if (flashFile) {
      for (int i = 0; i < batchIndex; i++) {
        flashFile.println(csvBuffer[i]);
      }
      flashFile.close();
      batchIndex = 0;
    } else {
      handleError(9);
      return;
    }
    event("Flushed 50 lines to CSV (Flash)");
    return;
  }
  File csvFile = SD.open(csvFilename, FILE_WRITE);
  if (csvFile) {
    for (int i = 0; i < batchIndex; i++) {
      csvFile.println(csvBuffer[i]);
    }
    csvFile.close();
    batchIndex = 0;
  } else {
    handleError(2);
    sdErrorHappened = true;
    return;
  }
  event("Flushed 50 lines to CSV (SD)");
}

/***************************************************************
 * event()
 * SD/Flashへのログ出力（シリアル出力も追加）
 ***************************************************************/
void event(String msg) {
  float t = (millis() - startTime) / 1000.0f;
  String s = String(t, 3) + ": " + msg;
  Serial.println(s);
  if (sdErrorHappened) {
    File f = Flash.open("event.txt", FILE_WRITE);
    if (f) {
      f.println(s);
      f.close();
    } else {
      handleError(9);
      return;
    }
    return;
  }
  File lf = SD.open(logFilename, FILE_WRITE);
  if (lf) {
    lf.println(s);
    lf.close();
  } else {
    handleError(2);
    sdErrorHappened = true;
  }
}

/***************************************************************
 * handleError()
 * エラーコードに応じたエラー出力とLED点滅
 ***************************************************************/
void handleError(int errCode) {
  String errDetail;
  switch(errCode) {
    case 1: errDetail = "SDカード初期化失敗"; break;
    case 2: errDetail = "CSV/LOGファイル作成失敗"; break;
    case 3: errDetail = "BME280センサーが見つからない"; break;
    case 4: errDetail = "MPU6050接続失敗"; break;
    case 5: errDetail = "GNSS初期化失敗"; break;
    case 9: errDetail = "Flashへの書き込み失敗"; break;
    default: errDetail = "不明なエラー"; break;
  }
  event("Error occurred, code = " + String(errCode) + ": " + errDetail);
  for (int i = 0; i < errCode; i++) {
    digitalWrite(PIN_LED3, HIGH);
    delay(300);
    digitalWrite(PIN_LED3, LOW);
    delay(300);
  }
  if (!continueOnError) {
    errorLoop(errCode, errDetail);
  }
}

/***************************************************************
 * errorLoop()
 * continueOnErrorがfalseの場合の無限ループ
 ***************************************************************/
void errorLoop(int errCode, String errMsg) {
  while (true) {
    for (int i = 0; i < errCode; i++) {
      digitalWrite(PIN_LED3, HIGH);
      delay(300);
      digitalWrite(PIN_LED3, LOW);
      delay(300);
    }
    delay(1000);
  }
}

/***************************************************************
 * LED制御関数
 ***************************************************************/
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

/***************************************************************
 * printSensorDataToSerial()
 * センサーデータをシリアルモニターに出力する関数
 ***************************************************************/
void printSensorDataToSerial(
  float time_s,
  float temperature, float humidity, float pressure,
  float latitude, float longitude, float altitude,
  bool fix, int satellites,
  float accelX, float accelY, float accelZ,
  float gyroX, float gyroY, float gyroZ,
  float totalAccel
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
  Serial.print(", AccelX: ");
  Serial.print(accelX, 4);
  Serial.print(" g, AccelY: ");
  Serial.print(accelY, 4);
  Serial.print(" g, AccelZ: ");
  Serial.print(accelZ, 4);
  Serial.print(" g, GyroX: ");
  Serial.print(gyroX, 4);
  Serial.print(" dps, GyroY: ");
  Serial.print(gyroY, 4);
  Serial.print(" dps, GyroZ: ");
  Serial.print(gyroZ, 4);
  Serial.print(" dps, TotalAccel: ");
  Serial.print(totalAccel, 3);
  Serial.println(" g");
}

/***************************************************************
 * deployParachute()
 * 非ブロッキングでパラシュート展開要求を行う関数
 ***************************************************************/
void deployParachute() {
  if (parachuteDeployed || deployPending) return;
  deployPending = true;
  deployStartTime = millis();
}

/***************************************************************
 * getNextFileIndex()
 * CSV/LOGファイルの次の連番を決定する関数
 ***************************************************************/
int getNextFileIndex() {
  int idx = 0;
  while (true) {
    String csvCandidate, logCandidate;
    if (idx == 0) {
      csvCandidate = String(CSV_BASE) + String(CSV_EXT);
      logCandidate = String(LOG_BASE) + String(LOG_EXT);
    } else {
      csvCandidate = String(CSV_BASE) + String(idx) + String(CSV_EXT);
      logCandidate = String(LOG_BASE) + String(idx) + String(LOG_EXT);
    }
    if (!SD.exists(csvCandidate) && !SD.exists(logCandidate))
      return idx;
    idx++;
  }
}

/***************************************************************
 * getNextPreFlightFileIndex()
 * 前段記録用ファイルの次の連番を決定する関数
 ***************************************************************/
int getNextPreFlightFileIndex() {
  int idx = 0;
  while (true) {
    String candidate;
    if (idx == 0)
      candidate = String(PRE_FLIGHT_BASE) + String(PRE_FLIGHT_EXT);
    else
      candidate = String(PRE_FLIGHT_BASE) + String(idx) + String(PRE_FLIGHT_EXT);
    if (!SD.exists(candidate))
      return idx;
    idx++;
  }
}

/***************************************************************
 * initFlash()
 * Flash初期化処理（必要ならフォーマットも実施）
 ***************************************************************/
void initFlash() {
  if (formatFlashOnBoot) {
    if (!Flash.format()) {
      // Flashフォーマット失敗時の処理
    }
  }
  if (!Flash.begin()) {
    // Flash初期化失敗時の処理
  }
  flashInitialized = true;
}

/***************************************************************
 * initGNSS()
 * GNSS初期化処理
 ***************************************************************/
void initGNSS() {
  int ret = Gnss.begin();
  if (ret != 0) handleError(5);
  Gnss.setInterval(GNSS_RATE);
  ret = Gnss.start();
  if (ret != 0) handleError(5);
}

/***************************************************************
 * initBME280()
 * BME280センサー初期化処理
 ***************************************************************/
void initBME280() {
  if (!bme.begin(0x76))
    handleError(3);
}

/***************************************************************
 * initMPU6050()
 * MPU6050センサー初期化処理
 ***************************************************************/
void initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection())
    handleError(4);
}

/***************************************************************
 * initSDandCSV()
 * SDカード初期化とCSV/LOGファイル作成処理
 ***************************************************************/
void initSDandCSV() {
  if (!SD.begin()) {
    handleError(1);
    sdErrorHappened = true;
    return;
  }
  int idx = getNextFileIndex();
  String csvCandidate, logCandidate;
  if (idx == 0) {
    csvCandidate = String(CSV_BASE) + String(CSV_EXT);
    logCandidate = String(LOG_BASE) + String(LOG_EXT);
  } else {
    csvCandidate = String(CSV_BASE) + String(idx) + String(CSV_EXT);
    logCandidate = String(LOG_BASE) + String(idx) + String(LOG_EXT);
  }
  File csvFile = SD.open(csvCandidate, FILE_WRITE);
  if (csvFile) {
    csvFile.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s,TotalAccel");
    csvFile.close();
    csvFilename = csvCandidate;
  } else {
    handleError(2);
    sdErrorHappened = true;
  }
  File lf = SD.open(logCandidate, FILE_WRITE);
  if (lf) {
    lf.println("Time_s:Event");
    lf.close();
    logFilename = logCandidate;
  } else {
    handleError(2);
    sdErrorHappened = true;
  }
}

/***************************************************************
 * initPreFlightCSV()
 * 前段記録用CSVファイル作成処理
 ***************************************************************/
void initPreFlightCSV() {
  if (!SD.begin())
    return;
  int idx = getNextPreFlightFileIndex();
  String candidate;
  if (idx == 0)
    candidate = String(PRE_FLIGHT_BASE) + String(PRE_FLIGHT_EXT);
  else
    candidate = String(PRE_FLIGHT_BASE) + String(idx) + String(PRE_FLIGHT_EXT);
  File pfFile = SD.open(candidate, FILE_WRITE);
  if (pfFile) {
    pfFile.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s,TotalAccel");
    pfFile.close();
    preFlightFilename = candidate;
  } else {
    handleError(2);
  }
}

/***************************************************************
 * setup()
 * 各種初期化処理
 ***************************************************************/
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  Wire.begin();
  pinMode(PIN_FLIGHT_OUTPUT, OUTPUT);
  digitalWrite(PIN_FLIGHT_OUTPUT, LOW);
  pinMode(PIN_FLIGHT_INPUT, INPUT_PULLUP);
  initFlash();
  startTime = millis();
  initGNSS();
  initBME280();
  initMPU6050();
  initSDandCSV();
  initPreFlightCSV();
  s_servo.attach(PIN_D12);
  s_servo.write(90);
  const int buff_num = 2;
  theCamera.begin(buff_num, CAM_VIDEO_FPS_60, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, CAM_IMAGE_PIX_FMT_JPG, 5);
  event("All initialization completed. Waiting for flight event...");
}

/***************************************************************
 * loop()
 * メインループ：フライト前は前段記録、フライト検知後は通常の記録＆動画処理
 ***************************************************************/
void loop() {
  unsigned long currentMillis = millis();
  Led_isActive();
  
  if (Gnss.waitUpdate(0)) {
    Gnss.getNavData(&gnssData);
    bool fixState = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
    Led_isPosfix(fixState);
  }
  
  // フライト前：50ms間隔で前段記録用データを蓄積（常に直近10秒分のみ保持）
  if (!flightStarted) {
    if (currentMillis - previousPreFlightSensorMillis >= PREFLIGHT_SENSOR_INTERVAL) {
      previousPreFlightSensorMillis = currentMillis;
      recordPreFlightSensorData();
    }
    // フライトピンが切られたと判断
    if (digitalRead(PIN_FLIGHT_INPUT) == HIGH) {
      flightStarted = true;
      flightStartTime = millis();
      event("Flight event detected: Flight pin cut. Starting sensor logging and video recording.");
      // バッファ内の直近10秒分のデータをCSVに出力
      flushPreFlightBufferToCSV();
      // 動画記録用初期化
      String newFilename = baseFilename + String(segmentIndex) + ".avi";
      theSD.remove(newFilename);
      File aviFile = theSD.open(newFilename, FILE_WRITE);
      pAvi = new AviLibrary();
      pAvi->begin(aviFile, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V);
      overallStartTime_video = millis();
      segmentStartTime = overallStartTime_video;
      theCamera.startStreaming(true, CamCB);
      pAvi->startRecording();
      // フライト開始後は前段記録処理は終了
    }
    return;
  }
  
  // フライト中：各センサ更新間隔に従って処理
  if (currentMillis - previousBme280Millis >= BME280_INTERVAL)
    previousBme280Millis = currentMillis;
  if (currentMillis - previousMpu6050Millis >= MPU6050_INTERVAL)
    previousMpu6050Millis = currentMillis;
  
  readAndLogSensors();
  
  if (currentMillis - previousSerialMillis >= SERIAL_PRINT_INTERVAL)
    previousSerialMillis = currentMillis;
  
  // 非ブロッキング展開処理：1秒後に実際の展開処理を実施
  if (deployPending && (millis() - deployStartTime >= 1000)) {
    event("Parachute deployed via " + deployMethod + " (delayed 1 sec).");
    s_servo.write(0);
    deployPending = false;
    parachuteDeployed = true;
  }
}
