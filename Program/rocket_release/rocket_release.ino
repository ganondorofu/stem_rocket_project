/***************************************************************
 * Spresense アドオンボード用 GNSS + BME280 + MPU6050
 * 取得データを SD カードへ直接書き込み（バッファ使用）
 * GNSSは非同期処理 (waitUpdate(0)) で、ほかの処理をブロックしないようにする
 *
 * 変更点:
 *   ・フライトピン検知を解除し、加速度による離陸検知に変更
 *   ・その他従来通り
 ***************************************************************/

#include <Arduino.h>
#include <SDHCI.h> // SDカード制御 (Spresense)
#include <File.h>  // SDカードファイル操作
#include <Wire.h>  // I2C (BME280, MPU6050)
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> // BME280
#include <MPU6050.h>         // MPU6050
#include <GNSS.h>            // GNSS Addon
#include <Flash.h>           // SpresenseのFlash制御クラス

#include <Camera.h>     // カメラ制御用ライブラリ
#include "AviLibrary.h" // AVI動画生成用ライブラリ

#define theSD SD
#include <math.h> // sqrt 用

/***************************************************************
 * ユーザー設定
 ***************************************************************/
bool formatFlashOnBoot = false;
bool continueOnError = true;

/***************************************************************
 * デバッグ出力設定
 ***************************************************************/
bool debugPrintSensors = true;
bool debugPrintBME280 = true;
bool debugPrintMPU6050 = true;
bool debugPrintGNSS = true;
bool debugPrintEvents = true;

/***************************************************************
 * 機能ON/OFF設定
 ***************************************************************/
bool useBME280 = true;
bool useMPU6050 = true;
bool useGNSS = true;
bool useCamera = true;

/***************************************************************
 * ファイル名関連定数
 ***************************************************************/
#define CSV_BASE "sensor_data"
#define CSV_EXT ".csv"
#define LOG_BASE "event_log"
#define LOG_EXT ".txt"
#define PRE_FLIGHT_BASE "before_flight"
#define PRE_FLIGHT_EXT ".csv"

/***************************************************************
 * LEDピン定義
 ***************************************************************/
#define PIN_LED0 2
#define PIN_LED1 3
#define PIN_LED3 4

/***************************************************************
 * センサー更新インターバル
 ***************************************************************/
#define BME280_INTERVAL 50
#define MPU6050_INTERVAL 10
#define SERIAL_PRINT_INTERVAL 1000

/***************************************************************
 * GNSS設定
 ***************************************************************/
#define GNSS_RATE SpInterval_10Hz

/***************************************************************
 * バッファ設定（フライト中用）
 ***************************************************************/
#define BATCH_SIZE 50

/***************************************************************
 * 前段記録用設定
 ***************************************************************/
#define PREFLIGHT_SENSOR_INTERVAL 50
#define PREFLIGHT_BUFFER_MAX 200

unsigned long previousPreFlightSensorMillis = 0;
unsigned long preFlightTimeBuffer[PREFLIGHT_BUFFER_MAX];
String preFlightDataBuffer[PREFLIGHT_BUFFER_MAX];
int preFlightBufferCount = 0;

/***************************************************************
 * フライトイベント閾値
 ***************************************************************/
#define FREEFALL_THRESHOLD 0.8     // g
#define LAUNCH_ACCEL_THRESHOLD 3.0 // g

bool flightStarted = false;
unsigned long flightStartTime = 0;

/***************************************************************
 * グローバル変数
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

String csvFilename;
String logFilename;
String preFlightFilename;

static String csvBuffer[BATCH_SIZE];
static int batchIndex = 0;
bool sdErrorHappened = false;

float preFlightAccSum = 0.0;
int preFlightAccCount = 0;

File aviFile;
AviLibrary *pAvi;
const uint32_t SEGMENT_DURATION_MS = 3000;
const uint32_t TOTAL_DURATION_MS = 50000;
uint32_t overallStartTime_video = 0;
uint32_t segmentStartTime = 0;
int segmentIndex = 0;
unsigned long frameCount = 0;
String baseFilename = "movie";
uint32_t lastFrameTime = 0;

bool parachuteDeployed = false;
String deployMethod;

int consecutivePressureDecrease = 0;
int consecutivePressureIncrease = 0;
float lastPressure = 0.0;
bool ascendingDetected = false;

bool flashInitialized = false;

// パラシュート展開を有効にするか（falseならログのみ）
bool useParachuteDeployment = true;
// フライト検知後、展開までの待ち時間（ms）
const unsigned long parachuteSafeDelay = 1000;

/***************************************************************
 * センサーデータ構造体
 ***************************************************************/
struct SensorData
{
  float time_s;
  float temperature, humidity, pressure;
  float latitude, longitude, altitude;
  bool fix;
  int satellites;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float totalAccel;
  bool flightFlag;       // フライト中か
  String parachuteState; // "NONE", "DEPLOYED", "DISABLED"
};

/***************************************************************
 * SensorUtil: データ取得 & CSV化
 ***************************************************************/
class SensorUtil
{
public:
  static SensorData getSensorData()
  {
    SensorData d;
    d.time_s = (millis() - startTime) / 1000.0f;
    if (useBME280)
    {
      d.temperature = bme.readTemperature();
      d.humidity = bme.readHumidity();
      d.pressure = bme.readPressure() / 100.0f;
    }
    else
      d.temperature = d.humidity = d.pressure = 0;
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (useMPU6050)
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rawX = ax / 16384.0f, rawY = ay / 16384.0f, rawZ = az / 16384.0f;
    d.accelX = -rawZ;
    d.accelY = rawY;
    d.accelZ = rawX;
    d.gyroX = gx / 131.0f;
    d.gyroY = gy / 131.0f;
    d.gyroZ = gz / 131.0f;
    d.fix = (gnssData.posDataExist && gnssData.posFixMode != FixInvalid);
    d.latitude = gnssData.latitude;
    d.longitude = gnssData.longitude;
    d.altitude = gnssData.altitude;
    d.satellites = gnssData.numSatellites;
    d.totalAccel = sqrt(d.accelX * d.accelX + d.accelY * d.accelY + d.accelZ * d.accelZ);
    // フライト中フラグとパラシュート状態を埋める
    d.flightFlag = flightStarted;
    if (!useParachuteDeployment)
      d.parachuteState = "DISABLED";
    else if (parachuteDeployed)
      d.parachuteState = "DEPLOYED";
    else
      d.parachuteState = "NONE";
    return d;
  }

  static String toCSV(const SensorData &d)
  {
    String s;
    s += String(d.time_s, 3) + ",";
    s += String(d.temperature, 2) + ",";
    s += String(d.humidity, 2) + ",";
    s += String(d.pressure, 2) + ",";
    s += String(d.latitude, 6) + ",";
    s += String(d.longitude, 6) + ",";
    s += String(d.altitude, 2) + ",";
    s += (d.fix ? "1," : "0,");
    s += String(d.satellites) + ",";
    s += String(d.accelX, 4) + ",";
    s += String(d.accelY, 4) + ",";
    s += String(d.accelZ, 4) + ",";
    s += String(d.gyroX, 4) + ",";
    s += String(d.gyroY, 4) + ",";
    s += String(d.gyroZ, 4) + ",";
    s += String(d.totalAccel, 3) + ",";
    s += (d.flightFlag ? "1" : "0") + String(",") + d.parachuteState;
    return s;
  }
};

/***************************************************************
 * プロトタイプ宣言
 ***************************************************************/
int getNextFileIndex();
int getNextPreFlightFileIndex();
void initFlash();
void initGNSS();
void initBME280();
void initMPU6050();
void initSDandCSV();
void initPreFlightCSV();
void recordPreFlightSensorData();
void readAndLogSensors();
void flushCsvBuffer();
void flushPreFlightBuffer();
void startFlight(String reason);
void event(String msg);
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
    float totalAccel);

/***************************************************************
 * setup()
 ***************************************************************/
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  Wire.begin();
  initFlash();
  startTime = millis();
  initGNSS();
  initBME280();
  initMPU6050();
  initSDandCSV();
  initPreFlightCSV();
  if (useCamera)
  {
    theCamera.begin(2, CAM_VIDEO_FPS_60,
                    CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V,
                    CAM_IMAGE_PIX_FMT_JPG, 5);
  }
  event("Initialization completed. Waiting for launch...");
}

/***************************************************************
 * loop()
 ***************************************************************/
void loop()
{
  unsigned long now = millis();
  Led_isActive();
  if (useGNSS && Gnss.waitUpdate(0))
  {
    Gnss.getNavData(&gnssData);
    bool f = (gnssData.posDataExist && gnssData.posFixMode != FixInvalid);
    Led_isPosfix(f);
  }

  if (!flightStarted)
  {
    // 前段記録
    if (now - previousPreFlightSensorMillis >= PREFLIGHT_SENSOR_INTERVAL)
    {
      previousPreFlightSensorMillis = now;
      recordPreFlightSensorData();
      // 加速度による離陸検知用呼び出し
      SensorData pf = SensorUtil::getSensorData();
      if (pf.totalAccel > LAUNCH_ACCEL_THRESHOLD)
      {
        startFlight("Acceleration threshold exceeded");
      }
    }
    return;
  }

  // フライト中
  if (now - previousBme280Millis >= BME280_INTERVAL)
    previousBme280Millis = now;
  if (now - previousMpu6050Millis >= MPU6050_INTERVAL)
    previousMpu6050Millis = now;

  readAndLogSensors();

  if (now - previousSerialMillis >= SERIAL_PRINT_INTERVAL)
    previousSerialMillis = now;
}

/***************************************************************
 * initFlash：Flash初期化
 ***************************************************************/
void initFlash()
{
  if (formatFlashOnBoot)
    Flash.format();
  Flash.begin();
  flashInitialized = true;
}

/***************************************************************
 * initGNSS：GNSS初期化
 ***************************************************************/
void initGNSS()
{
  if (!useGNSS)
    return;
  if (Gnss.begin() != 0)
    handleError(5);
  Gnss.setInterval(GNSS_RATE);
  if (Gnss.start() != 0)
    handleError(5);
}

/***************************************************************
 * initBME280：BME280初期化
 ***************************************************************/
void initBME280()
{
  if (!useBME280)
    return;
  if (!bme.begin(0x76))
  {
    handleError(3);
    useBME280 = false;
  }
}

/***************************************************************
 * initMPU6050：MPU6050初期化
 ***************************************************************/
void initMPU6050()
{
  if (!useMPU6050)
    return;
  mpu.initialize();
  if (!mpu.testConnection())
  {
    handleError(4);
    useMPU6050 = false;
  }
}

/***************************************************************
 * initSDandCSV(): SDカード待機＋CSV/LOGファイル作成
 ***************************************************************/
void initSDandCSV()
{
  // 最初に一回だけカード挿入を促す
  Serial.println("Insert SD card!");

  // SDスロット0で初期化を試みる。カードが挿入されるまで2秒間隔で再試行するが
  // プロンプトはもう出さない。
  while (!SD.begin(0))
  {
    delay(2000);
  }

  // カード挿入後は以降プロンプトなしで次の処理へ
  int idx = getNextFileIndex();
  String cfn = (idx == 0
                    ? String(CSV_BASE) + CSV_EXT
                    : String(CSV_BASE) + String(idx) + CSV_EXT);
  String lfn = (idx == 0
                    ? String(LOG_BASE) + LOG_EXT
                    : String(LOG_BASE) + String(idx) + LOG_EXT);

  // CSVヘッダを書き込む
  {
    File f = SD.open(cfn, FILE_WRITE);
    if (!f)
    {
      handleError(2);
      sdErrorHappened = true;
    }
    else
    {
      f.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s,TotalAccel,FlightStarted,ParachuteState");
      f.close();
      csvFilename = cfn;
    }
  }

  // イベントログのヘッダを書き込む
  {
    File f = SD.open(lfn, FILE_WRITE);
    if (!f)
    {
      handleError(2);
      sdErrorHappened = true;
    }
    else
    {
      f.println("Time_s:Event");
      f.close();
      logFilename = lfn;
    }
  }

  // 動画ファイル名のベースを設定
  baseFilename = String("movie") + String(idx) + "_";
}

/***************************************************************
 * initPreFlightCSV：前段記録CSV作成
 ***************************************************************/
void initPreFlightCSV() {
  int idx = getNextPreFlightFileIndex();
  String pfn = (idx==0
    ? String(PRE_FLIGHT_BASE)+PRE_FLIGHT_EXT
    : String(PRE_FLIGHT_BASE)+String(idx)+PRE_FLIGHT_EXT);

  File f = SD.open(pfn, FILE_WRITE);
  if (!f) { handleError(2); return; }
  // 末尾に FlightStarted と ParachuteState を追加
  f.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s,TotalAccel,FlightStarted,ParachuteState");
  f.close();
  preFlightFilename = pfn;
}

/***************************************************************
 * getNextFileIndex：CSV/LOGの次の連番取得
 ***************************************************************/
int getNextFileIndex()
{
  int i = 0;
  while (true)
  {
    String c = (i == 0
                    ? String(CSV_BASE) + CSV_EXT
                    : String(CSV_BASE) + String(i) + CSV_EXT);
    String l = (i == 0
                    ? String(LOG_BASE) + LOG_EXT
                    : String(LOG_BASE) + String(i) + LOG_EXT);
    if (!SD.exists(c) && !SD.exists(l))
      return i;
    i++;
  }
}

/***************************************************************
 * getNextPreFlightFileIndex：前段記録の連番取得
 ***************************************************************/
int getNextPreFlightFileIndex()
{
  int i = 0;
  while (true)
  {
    String p = (i == 0
                    ? String(PRE_FLIGHT_BASE) + PRE_FLIGHT_EXT
                    : String(PRE_FLIGHT_BASE) + String(i) + PRE_FLIGHT_EXT);
    if (!SD.exists(p))
      return i;
    i++;
  }
}

/***************************************************************
 * recordPreFlightSensorData：前段記録用バッファ蓄積
 ***************************************************************/
void recordPreFlightSensorData()
{
  SensorData data = SensorUtil::getSensorData();
  String line = SensorUtil::toCSV(data);
  unsigned long now = millis();

  if (preFlightBufferCount < PREFLIGHT_BUFFER_MAX)
  {
    preFlightTimeBuffer[preFlightBufferCount] = now;
    preFlightDataBuffer[preFlightBufferCount] = line;
    preFlightBufferCount++;
  }
  else
  {
    for (int i = 1; i < preFlightBufferCount; i++)
    {
      preFlightTimeBuffer[i - 1] = preFlightTimeBuffer[i];
      preFlightDataBuffer[i - 1] = preFlightDataBuffer[i];
    }
    preFlightTimeBuffer[PREFLIGHT_BUFFER_MAX - 1] = now;
    preFlightDataBuffer[PREFLIGHT_BUFFER_MAX - 1] = line;
  }

  while (preFlightBufferCount > 0 && (now - preFlightTimeBuffer[0]) > 10000)
  {
    for (int i = 1; i < preFlightBufferCount; i++)
    {
      preFlightTimeBuffer[i - 1] = preFlightTimeBuffer[i];
      preFlightDataBuffer[i - 1] = preFlightDataBuffer[i];
    }
    preFlightBufferCount--;
  }
}

/***************************************************************
 * CamCB：カメラコールバック（動画管理）
 ***************************************************************/
void CamCB(CamImage img)
{
  if (!useCamera)
    return;
  if (!img.isAvailable())
    return;
  uint32_t now = millis();

  if (now - overallStartTime_video >= TOTAL_DURATION_MS)
  {
    pAvi->endRecording();
    pAvi->end();
    theCamera.end();
    while (true)
    {
      digitalWrite(PIN_LED0, HIGH);
      delay(100);
      digitalWrite(PIN_LED0, LOW);
      delay(100);
    }
  }
  if (now - segmentStartTime >= SEGMENT_DURATION_MS)
  {
    pAvi->endRecording();
    pAvi->end();
    delete pAvi;
    segmentIndex++;
    String fn = baseFilename + String(segmentIndex) + ".avi";
    theSD.remove(fn);
    aviFile = theSD.open(fn, FILE_WRITE);
    pAvi = new AviLibrary();
    pAvi->begin(aviFile, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V);
    pAvi->startRecording();
    segmentStartTime = now;
  }
  pAvi->addFrame(img.getImgBuff(), img.getImgSize());
  frameCount++;
}

/***************************************************************
 * readAndLogSensors：フライト中センサー取得・バッファ書き込み
 ***************************************************************/
void readAndLogSensors()
{
  SensorData data = SensorUtil::getSensorData();

  // 気圧トレンド検出
  if (lastPressure == 0.0)
  {
    lastPressure = data.pressure;
  }
  else
  {
    if (data.pressure < lastPressure)
    {
      consecutivePressureDecrease++;
      consecutivePressureIncrease = 0;
    }
    else if (data.pressure > lastPressure)
    {
      consecutivePressureIncrease++;
      consecutivePressureDecrease = 0;
    }
    lastPressure = data.pressure;
  }
  if (consecutivePressureDecrease >= 5)
    ascendingDetected = true;
  if (ascendingDetected && consecutivePressureIncrease >= 5)
  {
    if (millis() - startTime >= 1500)
    {
      event("下降検知（気圧変化）");
      deployMethod = "気圧変化";
      deployParachute();
    }
    else
    {
      event("下降検知（初期段階）");
    }
  }

  // CSVバッファ
  csvBuffer[batchIndex++] = SensorUtil::toCSV(data);
  if (batchIndex >= BATCH_SIZE)
    flushCsvBuffer();

  // シリアル出力
  printSensorDataToSerial(
      data.time_s,
      data.temperature, data.humidity, data.pressure,
      data.latitude, data.longitude, data.altitude,
      data.fix, data.satellites,
      data.accelX, data.accelY, data.accelZ,
      data.gyroX, data.gyroY, data.gyroZ,
      data.totalAccel);

  // ↓↓↓ 修正：有効時のみ、安全遅延後に展開
  if (useParachuteDeployment && !parachuteDeployed && millis() - flightStartTime >= parachuteSafeDelay)
  {
    event("安全機構展開");
    deployMethod = "安全機構";
    deployParachute();
  }
}

/***************************************************************
 * flushCsvBuffer：バッファ50行をSD/Flashに書き込み
 ***************************************************************/
void flushCsvBuffer()
{
  if (batchIndex == 0)
    return;

  if (sdErrorHappened)
  {
    File f = Flash.open("sensor_data.csv", FILE_WRITE);
    if (!f)
    {
      handleError(9);
      return;
    }
    for (int i = 0; i < batchIndex; i++)
      f.println(csvBuffer[i]);
    f.close();
    event("Flushed to Flash");
  }
  else
  {
    File f = SD.open(csvFilename, FILE_WRITE);
    if (!f)
    {
      handleError(2);
      sdErrorHappened = true;
      return;
    }
    for (int i = 0; i < batchIndex; i++)
      f.println(csvBuffer[i]);
    f.close();
    event("Flushed to SD");
  }

  batchIndex = 0;
}

/***************************************************************
 * flushPreFlightBuffer：前段記録フラッシュ
 ***************************************************************/
void flushPreFlightBuffer()
{
  if (sdErrorHappened)
  {
    File f = Flash.open(preFlightFilename, FILE_WRITE);
    if (f)
    {
      f.flush();
      f.close();
    }
  }
  else
  {
    File f = SD.open(preFlightFilename, FILE_WRITE);
    if (f)
    {
      f.flush();
      f.close();
    }
  }
}

/***************************************************************
 * event：イベントログ記録
 ***************************************************************/
void event(String msg)
{
  float t = (millis() - startTime) / 1000.0f;
  String s = String(t, 3) + ": " + msg;
  if (debugPrintEvents)
    Serial.println(s);
  if (sdErrorHappened)
  {
    File f = Flash.open("event.txt", FILE_WRITE);
    if (!f)
    {
      handleError(9);
      return;
    }
    f.println(s);
    f.close();
  }
  else
  {
    File f = SD.open(logFilename, FILE_WRITE);
    if (!f)
    {
      handleError(2);
      sdErrorHappened = true;
      return;
    }
    f.println(s);
    f.close();
  }
}

/***************************************************************
 * deployParachute：パラシュート展開ログ
 ***************************************************************/
void deployParachute()
{
  if (parachuteDeployed)
    return;
  parachuteDeployed = true;
  if (useParachuteDeployment)
  {
    event("Parachute via " + deployMethod);
  }
  else
  {
    event("Parachute SKIPPED (disabled) via " + deployMethod);
  }
}

/***************************************************************
 * startFlight：離陸検知共通処理
 ***************************************************************/
void startFlight(String reason)
{
  if (flightStarted)
    return;
  flightStarted = true;
  flightStartTime = millis();
  event("Flight start: " + reason);
  flushPreFlightBuffer();
  if (useCamera)
  {
    String fn = baseFilename + String(segmentIndex) + ".avi";
    theSD.remove(fn);
    aviFile = theSD.open(fn, FILE_WRITE);
    pAvi = new AviLibrary();
    pAvi->begin(aviFile, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V);
    overallStartTime_video = millis();
    segmentStartTime = overallStartTime_video;
    theCamera.startStreaming(true, CamCB);
    pAvi->startRecording();
  }
}

/***************************************************************
 * handleError/errorLoop：エラー処理
 ***************************************************************/
void handleError(int errCode)
{
  String detail;
  switch (errCode)
  {
  case 1:
    detail = "SD初期化失敗";
    break;
  case 2:
    detail = "ファイル作成失敗";
    break;
  case 3:
    detail = "BME280未検出";
    break;
  case 4:
    detail = "MPU6050接続失敗";
    break;
  case 5:
    detail = "GNSS初期化失敗";
    break;
  case 9:
    detail = "Flash書込失敗";
    break;
  default:
    detail = "不明";
    break;
  }
  event("Error" + String(errCode) + ": " + detail);
  for (int i = 0; i < errCode; i++)
  {
    digitalWrite(PIN_LED3, HIGH);
    delay(300);
    digitalWrite(PIN_LED3, LOW);
    delay(300);
  }
  if (!continueOnError)
    errorLoop(errCode, detail);
}
void errorLoop(int errCode, String errMsg)
{
  while (true)
  {
    for (int i = 0; i < errCode; i++)
    {
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
void Led_isActive()
{
  static unsigned long last = 0;
  static bool st = false;
  unsigned long now = millis();
  if (now - last >= 500)
  {
    st = !st;
    digitalWrite(PIN_LED0, st ? HIGH : LOW);
    last = now;
  }
}
void Led_isPosfix(bool st) { digitalWrite(PIN_LED1, st ? HIGH : LOW); }
void Led_isError(bool st) { digitalWrite(PIN_LED3, st ? HIGH : LOW); }

/***************************************************************
 * printSensorDataToSerial：シリアル詳細出力
 ***************************************************************/
void printSensorDataToSerial(
    float time_s,
    float temperature, float humidity, float pressure,
    float latitude, float longitude, float altitude,
    bool fix, int satellites,
    float accelX, float accelY, float accelZ,
    float gyroX, float gyroY, float gyroZ,
    float totalAccel)
{
  if (!debugPrintSensors)
    return;
  Serial.print("T=");
  Serial.print(time_s, 3);
  Serial.print("s");
  if (debugPrintBME280)
  {
    Serial.print(" Temp=");
    Serial.print(temperature, 2);
    Serial.print("C Hum=");
    Serial.print(humidity, 2);
    Serial.print("% Pres=");
    Serial.print(pressure, 2);
    Serial.print("hPa");
  }
  if (debugPrintGNSS)
  {
    Serial.print(" Lat=");
    Serial.print(latitude, 6);
    Serial.print(" Lng=");
    Serial.print(longitude, 6);
    Serial.print(" Alt=");
    Serial.print(altitude, 2);
    Serial.print(" Fix=");
    Serial.print(fix);
    Serial.print(" Sat=");
    Serial.print(satellites);
  }
  if (debugPrintMPU6050)
  {
    Serial.print(" AX=");
    Serial.print(accelX, 4);
    Serial.print(" AY=");
    Serial.print(accelY, 4);
    Serial.print(" AZ=");
    Serial.print(accelZ, 4);
    Serial.print(" GX=");
    Serial.print(gyroX, 4);
    Serial.print(" GY=");
    Serial.print(gyroY, 4);
    Serial.print(" GZ=");
    Serial.print(gyroZ, 4);
  }
  Serial.print(" TotA=");
  Serial.print(totalAccel, 3);
  Serial.println("g");
}
