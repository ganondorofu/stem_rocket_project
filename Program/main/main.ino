/***************************************************************
 * Spresense アドオンボード用 GNSS + BME280 + MPU6050
 * 取得データを SD カードへ直接書き込み（バッファ使用）
 * GNSSは非同期処理 (waitUpdate(0)) で、ほかの処理をブロックしないようにする
 *
 * 変更点:
 *   1) 既存ファイルがあれば連番付きファイル名を生成（CSV, LOG, 前段記録用 befot_flight.csv）
 *   2) センサーデータを10回分バッファにため、まとめて書き込む（フライト開始後）
 *   3) 書き込みエラーがあった場合はバッファを保持し、次回以降リトライ
 *   4) 書き込み完了後にイベントログへ記録
 *   5) bool continueOnError を追加。trueならエラー発生時も処理続行、falseなら停止
 *   6) パラシュート展開のロジック削除 → 今回は自由落下検知により展開する
 *   7) MPU6050_INTERVAL = 10ms (100Hz), BME280_INTERVAL = 50ms, GNSS = 10Hz
 *   8) SDエラー後は内蔵Flashへ書き込み
 *   9) eventログもFlashへ書き込み
 *   10) 起動時にFlashをフォーマットするかを変数で制御 (formatFlashOnBoot)
 *
 * ★ 追加変更点: 前段記録（Pre-flight logging）＋加速度測定
 *   - 起動後、フライトピンが切れるまで（ショート状態）の間、1秒間隔でセンサーデータを
 *     "befor_flight.csv" として連番付きで記録し、TotalAccel（加速度の大きさ）も記録する。
 *   - フライト中も TotalAccel を記録し、TotalAccel が FREEFALL_THRESHOLD 以下になった場合、
 *     加速度による自由落下検知としパラシュート展開のトリガーとする。
 *   - また、気圧センサのサンプルで5回連続で気圧が減少して「上昇中」と判定し、
 *     その後5回連続で気圧が上昇した場合、発射から1.5秒以降ならパラシュート展開を行い、
 *     ログにもその展開方法（自由落下検知による展開）を記録する。
 *   - ただし、発射から1.5秒未満で下降が検知された場合は、ログには下降検知と記録するが展開は行わない。
 *   - さらに、安全機構として、離陸から3秒経過してもまだ展開されていなければ
 *     自動的に deployParachute() を呼び出す（この関数内で1秒後に実際のログ出力とサーボ動作を行う）。
 *
 * ★ フライトピン実装（ピン8,9 使用）は前回実装の通り
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
#include "../lib/common.h"

/***************************************************************
 * ユーザー設定
 ***************************************************************/
bool formatFlashOnBoot = false;
bool continueOnError = true; // エラー発生時、true なら処理続行、false なら停止

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
#define BATCH_SIZE 10

/***************************************************************
 * 前段記録用間隔
 ***************************************************************/
#define PRE_FLIGHT_INTERVAL 1000   // 1秒間隔

/***************************************************************
 * フライトピン用ピン定義
 ***************************************************************/
#define PIN_FLIGHT_OUTPUT 8
#define PIN_FLIGHT_INPUT  9

// フライトイベントフラグ
bool flightStarted = false;

/***************************************************************
 * グローバル変数（センサ／ログ）
 ***************************************************************/
unsigned long previousBme280Millis = 0;
unsigned long previousMpu6050Millis = 0;
unsigned long previousSerialMillis = 0;
unsigned long previousPreFlightMillis = 0;
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
 * 追加：前段記録中の加速度平均用変数
 ***************************************************************/
float preFlightAccSum = 0.0;
int preFlightAccCount = 0;

/***************************************************************
 * 追加：パラシュート展開用
 ***************************************************************/
#define FREEFALL_THRESHOLD 0.6  // g 以下なら自由落下とみなす
bool parachuteDeployed = false;
bool apexDetected = false;
uint32_t apexTime = 0;

/***************************************************************
 * 追加：気圧トレンド検出用変数
 *  5回連続の変化で判定するためのカウンタと前回値
 ***************************************************************/
int consecutivePressureDecrease = 0;
int consecutivePressureIncrease = 0;
float lastPressure = 0.0;
bool ascendingDetected = false;  // 5回連続で気圧が減少して上昇中と判定

/***************************************************************
 * 関数プロトタイプ（センサ／ログ関連）
 ***************************************************************/
int  getNextFileIndex();
int  getNextPreFlightFileIndex();
void readAndLogSensors();
void flushCsvBuffer();
void readAndLogSensorsPreFlight();
void logEvent(String msg); // Provided by common library
void handleError(int errCode);
void errorLoop(int errCode, String errMsg);
void deployParachute();
void Led_isActive();
void Led_isPosfix(bool state);
void Led_isError(bool state);
void printSensorDataToSerial(
  float time_s,
  float temperature, float humidity, float pressure,
  float latitude, float longitude, float altitude,
  bool fix, int satellites,
  float accelX, float accelY, float accelZ,
  float gyroX, float gyroY, float gyroZ,
  float totalAccel
);

/***************************************************************
 * グローバル変数（カメラ・動画関連）
 ***************************************************************/
File aviFile;
AviLibrary* pAvi;
const uint32_t SEGMENT_DURATION_MS = 3000;
const uint32_t TOTAL_DURATION_MS   = 50000;
uint32_t overallStartTime_video = 0;
uint32_t segmentStartTime = 0;
int segmentIndex = 0;
unsigned long frameCount = 0;
String baseFilename = "movie";
uint32_t lastFrameTime = 0;

/***************************************************************
 * カメラの画像取得時コールバック（動画記録用）
 ***************************************************************/
void CamCB(CamImage img) {
  if (!img.isAvailable()) return;
  uint32_t currentTime = millis();
  if (lastFrameTime != 0) {
    uint32_t frameGap = currentTime - lastFrameTime;
    if (frameGap > 100) {
      Serial.println("注意: 動画が途切れた可能性があります。途切れ時間: " + String(frameGap) + " ms");
    }
  }
  lastFrameTime = currentTime;
  uint32_t overallElapsed = currentTime - overallStartTime_video;
  uint32_t segmentElapsed = currentTime - segmentStartTime;
  if (overallElapsed >= TOTAL_DURATION_MS) {
    pAvi->endRecording();
    pAvi->end();
    Serial.println("最終セグメント（" + String(segmentIndex) + "）保存完了 - 経過時間: " + String(overallElapsed) + " ms");
    Serial.println("Movie saved");
    Serial.println(" Movie width:    " + String(pAvi->getWidth()));
    Serial.println(" Movie height:   " + String(pAvi->getHeight()));
    Serial.println(" File size (kB): " + String(pAvi->getFileSize()));
    Serial.println(" Captured Frame: " + String(pAvi->getTotalFrame()));
    Serial.println(" Duration (sec): " + String(pAvi->getDuration()));
    Serial.println(" Frame per sec : " + String(pAvi->getFps()));
    Serial.println(" Max data rate : " + String(pAvi->getMaxDataRate()));
    theCamera.end();
    while (true) {
      digitalWrite(PIN_LED0, HIGH);
      delay(100);
      digitalWrite(PIN_LED0, LOW);
      delay(100);
    }
  }
  if (segmentElapsed >= SEGMENT_DURATION_MS) {
    pAvi->endRecording();
    pAvi->end();
    Serial.println("セグメント " + String(segmentIndex) + " 保存完了 - 経過時間: " + String(overallElapsed) + " ms");
    delete pAvi;
    segmentIndex++;
    String newFilename = baseFilename + String(segmentIndex) + ".avi";
    theSD.remove(newFilename);
    aviFile = theSD.open(newFilename, FILE_WRITE);
    pAvi = new AviLibrary();
    pAvi->begin(aviFile, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V);
    pAvi->startRecording();
    segmentStartTime = currentTime;
  }
  uint32_t encodeStart = millis();
  pAvi->addFrame(img.getImgBuff(), img.getImgSize());
  uint32_t encodeEnd = millis();
  uint32_t encodeTime = encodeEnd - encodeStart;
  frameCount++;
  Serial.println("フレーム " + String(frameCount) + " を記録。エンコード時間: " + String(encodeTime) + " ms");
}

/***************************************************************
 * readAndLogSensors() 関数（フライト中用）
 * TotalAccel を計算し、CSV に記録する。
 * さらに、気圧センサからのサンプルを連続して取得し、5回連続の
 * 減少なら「上昇中」、5回連続の増加なら「下降中」と判定する。
 * 上昇状態が検知された後に下降が検知された場合、
 * 発射から1.5秒以降なら deployParachute() を呼び出し、ログに記録する。
 * ただし、発射から1.5秒未満で下降が検知された場合はログには下降検知と記録するが展開は行わない。
 * さらに、安全機構として、離陸から3秒経過してもまだ展開されていなければ
 * 自動的に deployParachute() を呼び出す（deployParachute() 内で1秒後に実際の処理を実施）。
 ***************************************************************/
void readAndLogSensors() {
  float nowSec = (millis() - startTime) / 1000.0f;
  float temperature = bme.readTemperature();
  float humidity    = bme.readHumidity();
  float pressure    = bme.readPressure() / 100.0f;  // hPa
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;
  float gyroX  = gx / 131.0f;
  float gyroY  = gy / 131.0f;
  float gyroZ  = gz / 131.0f;
  bool fix = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
  float latitude  = gnssData.latitude;
  float longitude = gnssData.longitude;
  float altitude  = gnssData.altitude;
  int satellites  = gnssData.numSatellites;
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  
  // 気圧トレンドの検出
  if (lastPressure == 0.0) {
    lastPressure = pressure;
  } else {
    if (pressure < lastPressure) {
      consecutivePressureDecrease++;
      consecutivePressureIncrease = 0;
    } else if (pressure > lastPressure) {
      consecutivePressureIncrease++;
      consecutivePressureDecrease = 0;
    }
    lastPressure = pressure;
  }
  if (consecutivePressureDecrease >= 5) {
    ascendingDetected = true;
  }
  if (ascendingDetected && consecutivePressureIncrease >= 5) {
    // 下降検知：発射から1.5秒以降ならパラシュート展開
    if ((millis() - startTime) >= 1500) {
      logEvent("下降検知（気圧変化による）");
      deployParachute();
    } else {
      logEvent("下降検知（発射初期段階のためパラシュート未展開）");
    }
  }
  
  // 加速度による自由落下検知：TotalAccel が閾値以下ならパラシュート展開
  if (!parachuteDeployed && totalAccel < FREEFALL_THRESHOLD) {
    logEvent("加速度検知: 自由落下と判断 (TotalAccel = " + String(totalAccel, 3) + " g)");
    deployParachute();
  }
  
  // CSV行作成
  String line;
  line += String(nowSec, 3);       line += ",";
  line += String(temperature, 2);  line += ",";
  line += String(humidity, 2);     line += ",";
  line += String(pressure, 2);     line += ",";
  line += String(latitude, 6);     line += ",";
  line += String(longitude, 6);    line += ",";
  line += String(altitude, 2);     line += ",";
  line += (fix ? "1" : "0");       line += ",";
  line += String(satellites);      line += ",";
  line += String(accelX, 4);       line += ",";
  line += String(accelY, 4);       line += ",";
  line += String(accelZ, 4);       line += ",";
  line += String(gyroX, 4);        line += ",";
  line += String(gyroY, 4);        line += ",";
  line += String(gyroZ, 4);        line += ",";
  line += String(totalAccel, 3);
  
  if (batchIndex < BATCH_SIZE) {
    csvBuffer[batchIndex] = line;
    batchIndex++;
  } else {
    Serial.println("Warning: CSV buffer is full, discarding new data");
  }
  if (batchIndex >= BATCH_SIZE) {
    flushCsvBuffer();
  }
  printSensorDataToSerial(
    nowSec,
    temperature, humidity, pressure,
    latitude, longitude, altitude,
    fix, satellites,
    accelX, accelY, accelZ,
    gyroX, gyroY, gyroZ,
    totalAccel
  );
  
  // 安全機構: 離陸から3秒経過してもまだパラシュートが展開されていなければ自動展開
  if (!parachuteDeployed && (millis() - startTime >= 3000)) {
    logEvent("安全機構: 離陸から3秒経過したためパラシュート展開");
    deployParachute();
  }
}

/***************************************************************
 * readAndLogSensorsPreFlight() 関数（フライト前用）
 * TotalAccel を計算し、CSV に記録するとともに、5サンプルごとに平均値をシリアル出力する
 ***************************************************************/
void readAndLogSensorsPreFlight() {
  float nowSec = (millis() - startTime) / 1000.0f;
  float temperature = bme.readTemperature();
  float humidity    = bme.readHumidity();
  float pressure    = bme.readPressure() / 100.0f;
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;
  float gyroX  = gx / 131.0f;
  float gyroY  = gy / 131.0f;
  float gyroZ  = gz / 131.0f;
  bool fix = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
  float latitude  = gnssData.latitude;
  float longitude = gnssData.longitude;
  float altitude  = gnssData.altitude;
  int satellites  = gnssData.numSatellites;
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  String line;
  line += String(nowSec, 3);       line += ",";
  line += String(temperature, 2);  line += ",";
  line += String(humidity, 2);     line += ",";
  line += String(pressure, 2);     line += ",";
  line += String(latitude, 6);     line += ",";
  line += String(longitude, 6);    line += ",";
  line += String(altitude, 2);     line += ",";
  line += (fix ? "1" : "0");       line += ",";
  line += String(satellites);      line += ",";
  line += String(accelX, 4);       line += ",";
  line += String(accelY, 4);       line += ",";
  line += String(accelZ, 4);       line += ",";
  line += String(gyroX, 4);        line += ",";
  line += String(gyroY, 4);        line += ",";
  line += String(gyroZ, 4);        line += ",";
  line += String(totalAccel, 3);
  if (sdErrorHappened) {
    File flashFile = Flash.open(preFlightFilename, FILE_WRITE);
    if (!flashFile) {
      Serial.println("Error: Could not open Flash for pre-flight data");
      handleError(9);
      return;
    }
    flashFile.println(line);
    flashFile.close();
  } else {
    File pfFile = SD.open(preFlightFilename, FILE_WRITE);
    if (!pfFile) {
      Serial.println("Error: Could not open pre-flight CSV file for writing");
      handleError(2);
      return;
    }
    pfFile.println(line);
    pfFile.close();
  }
  Serial.print("Pre-flight data: ");
  Serial.println(line);
  preFlightAccSum += totalAccel;
  preFlightAccCount++;
  if (preFlightAccCount >= 5) {
    float avgAcc = preFlightAccSum / preFlightAccCount;
    Serial.print("Average Acceleration (pre-flight over ");
    Serial.print(preFlightAccCount);
    Serial.print(" samples): ");
    Serial.print(avgAcc, 3);
    Serial.println(" g");
    preFlightAccSum = 0;
    preFlightAccCount = 0;
  }
}

/***************************************************************
 * flushCsvBuffer() 関数
 ***************************************************************/
void flushCsvBuffer() {
  if (batchIndex == 0) return;
  if (sdErrorHappened) {
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
    logEvent("Flushed 10 lines to CSV (Flash)");
    return;
  }
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
  logEvent("Flushed 10 lines to CSV (SD)");
}


/***************************************************************
 * handleError() 関数
 * エラー発生時は、エラーコードに応じたエラー内容を出力し、LEDを点滅させる
 ***************************************************************/
void handleError(int errCode) {
  String errDetail;
  switch(errCode) {
    case 1:
      errDetail = "SDカード初期化失敗";
      break;
    case 2:
      errDetail = "CSV/LOGファイル作成失敗";
      break;
    case 3:
      errDetail = "BME280センサーが見つからない";
      break;
    case 4:
      errDetail = "MPU6050接続失敗";
      break;
    case 5:
      errDetail = "GNSS初期化失敗";
      break;
    case 9:
      errDetail = "Flashへの書き込み失敗";
      break;
    default:
      errDetail = "不明なエラー";
      break;
  }
  logEvent("Error occurred, code = " + String(errCode) + ": " + errDetail);
  
  // LEDをエラーコード回数分点滅
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
 * errorLoop() 関数
 * continueOnError が false の場合、無限ループでエラー内容を表示し、LEDを点滅する
 ***************************************************************/
void errorLoop(int errCode, String errMsg) {
  while (true) {
    Serial.println("Error Code " + String(errCode) + ": " + errMsg);
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
 * printSensorDataToSerial() 関数
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
  Serial.print(", Accel(g): ");
  Serial.print(accelX, 4); Serial.print("/");
  Serial.print(accelY, 4); Serial.print("/");
  Serial.print(accelZ, 4);
  Serial.print(", Gyro(deg/s): ");
  Serial.print(gyroX, 4); Serial.print("/");
  Serial.print(gyroY, 4); Serial.print("/");
  Serial.print(gyroZ, 4);
  Serial.print(", TotalAccel: ");
  Serial.print(totalAccel, 3);
  Serial.println();
}

/***************************************************************
 * deployParachute() 関数
 * パラシュート展開処理を実行する関数
 * ※ 呼ばれてから1秒後にログ出力とサーボ動作を実施する
 ***************************************************************/
void deployParachute() {
  if (parachuteDeployed) return; // 重複実行防止
  parachuteDeployed = true;  // 展開済みとマーク
  // 1秒待機してから実際の処理を実施
  delay(1000);
  logEvent("Parachute deployed via free-fall or safety timer (delayed 1 sec).");
  // サーボ等によるパラシュート展開処理（例）
  s_servo.write(0);
}

/***************************************************************
 * getNextFileIndex() 関数
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
 * getNextPreFlightFileIndex() 関数
 ***************************************************************/
int getNextPreFlightFileIndex() {
  int idx = 0;
  while (true) {
    String candidate;
    if (idx == 0) {
      candidate = String(PRE_FLIGHT_BASE) + String(PRE_FLIGHT_EXT);
    } else {
      candidate = String(PRE_FLIGHT_BASE) + String(idx) + String(PRE_FLIGHT_EXT);
    }
    if (!SD.exists(candidate)) {
      return idx;
    }
    idx++;
  }
}

/***************************************************************
 * setup() 関数
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
  initFlash(formatFlashOnBoot, flashInitialized);
  startTime = millis();
  initGNSS(Gnss);
  initBME280(bme);
  initMPU6050(mpu);
  initSDandCSV(SD, csvFilename, logFilename, sdErrorHappened,
               CSV_BASE, CSV_EXT, LOG_BASE, LOG_EXT, handleError);
  initPreFlightCSV(SD, preFlightFilename, PRE_FLIGHT_BASE, PRE_FLIGHT_EXT,
                   handleError);
  s_servo.attach(PIN_D12);
  s_servo.write(90);
  const int buff_num = 2;
  theCamera.begin(buff_num, CAM_VIDEO_FPS_60, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, CAM_IMAGE_PIX_FMT_JPG, 5);
  logEvent("All initialization completed. Waiting for flight event...");
}

/***************************************************************
 * loop() 関数
 ***************************************************************/
void loop() {
  unsigned long currentMillis = millis();
  Led_isActive();
  if (Gnss.waitUpdate(0)) {
    Gnss.getNavData(&gnssData);
    bool fixState = (gnssData.posDataExist && (gnssData.posFixMode != FixInvalid));
    Led_isPosfix(fixState);
  }
  if (!flightStarted) {
    if (currentMillis - previousPreFlightMillis >= PRE_FLIGHT_INTERVAL) {
      previousPreFlightMillis = currentMillis;
      readAndLogSensorsPreFlight();
    }
    if (digitalRead(PIN_FLIGHT_INPUT) == HIGH) {
      flightStarted = true;
      logEvent("Flight event detected: Flight pin cut. Starting sensor logging and video recording.");
      String newFilename = baseFilename + String(segmentIndex) + ".avi";
      theSD.remove(newFilename);
      aviFile = theSD.open(newFilename, FILE_WRITE);
      pAvi = new AviLibrary();
      pAvi->begin(aviFile, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V);
      overallStartTime_video = millis();
      segmentStartTime = overallStartTime_video;
      theCamera.startStreaming(true, CamCB);
      pAvi->startRecording();
    }
    return;
  }
  if (currentMillis - previousBme280Millis >= BME280_INTERVAL) {
    previousBme280Millis = currentMillis;
  }
  if (currentMillis - previousMpu6050Millis >= MPU6050_INTERVAL) {
    previousMpu6050Millis = currentMillis;
  }
  readAndLogSensors();
  if (currentMillis - previousSerialMillis >= SERIAL_PRINT_INTERVAL) {
    previousSerialMillis = currentMillis;
  }
}
