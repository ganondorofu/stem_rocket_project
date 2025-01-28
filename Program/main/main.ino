#include <Arduino.h>
#include <SDHCI.h>
#include <File.h>
#include <GNSS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>

// ファイル名の定義
#define CSV_BASE_FILENAME "sensor_data"
#define CSV_EXTENSION ".csv"
#define LOG_BASE_FILENAME "event_log"
#define LOG_EXTENSION ".txt"

// バッファサイズと再起動サイクルの定義
#define STRING_BUFFER_SIZE 128
#define BUFFER_SIZE 50 // バッファに蓄積するデータ数
//#define RESTART_CYCLE (60 * 5) // 300ループごとに再起動（必要に応じて調整）

// LEDピンの定義
#define PIN_LED0 2 // アクティブLED
#define PIN_LED1 3 // Fix LED
#define PIN_LED3 4 // エラーLED

// センサー更新インターバル（ミリ秒）
#define BME280_INTERVAL 50 // 50msごとにBME280を取得
#define MPU6050_INTERVAL 50  // 50msごとにMPU6050を取得
#define SERIAL_PRINT_INTERVAL 1000 // 1秒間隔でシリアル出力

// GNSS更新レート（Hz）
#define GNSS_UPDATE_RATE 10

// センサー更新タイミング管理用変数
unsigned long previousBme280Millis = 0;
unsigned long previousMpu6050Millis = 0;
unsigned long previousSerialMillis = 0;

// データバッファ
struct sensorDataMsg {
  float timeFromPowerOn;
  float temperature;
  float humidity;
  float pressure;
  float latitude;
  float longitude;
  float gpsAltitude;
  int fix;
  int satellites;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

sensorDataMsg dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// グローバル変数
SDClass SD;
File myFile;
SpGnss Gnss;
Adafruit_BME280 bme;
MPU6050 mpu;
unsigned long startTime;

// CSVおよびログファイル名を動的に管理するための変数
char currentCSVFilename[32] = {0};
char currentLOGFilename[32] = {0};

// GNSS衛星選択
enum ParamSat {
  eSatGps,
  eSatGlonass,
  eSatGpsSbas,
  eSatGpsGlonass,
  eSatGpsBeidou,
  eSatGpsGalileo,
  eSatGpsQz1c,
  eSatGpsGlonassQz1c,
  eSatGpsBeidouQz1c,
  eSatGpsGalileoQz1c,
  eSatGpsQz1cQz1S
};
static enum ParamSat satType = eSatGpsGlonass;

// 最新のGNSS情報を保持
static SpNavData g_lastNavData;

// プロトタイプ宣言
void initGNSS();
void initBME280();
void initMPU6050();
void initCSV();
void recordSensorData();
void writeToCSV(const sensorDataMsg &data);
void flushBufferToCSV();
void event(String event_msg);
void errorLoop(int num);
static void Led_isActive();
static void Led_isPosfix(bool state);
static void Led_isError(bool state);
static void print_pos(const SpNavData *pNavData);
static void print_condition(const SpNavData *pNavData);
void printSensorData(const sensorDataMsg &d);
// void restartGNSS(); // 再起動機能を無効化

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // LEDピンの設定
  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);

  startTime = millis();

  Wire.begin();

  initGNSS();
  initBME280();
  initMPU6050();

  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed.");
    errorLoop(1);
  }

  initCSV();
  event("All initialization completed");
}

void loop() {
  unsigned long currentMillis = millis();

  Led_isActive();

  // GNSSをノンブロッキングで更新チェック
  bool updated = Gnss.waitUpdate(0);
  if (updated) {
    Gnss.getNavData(&g_lastNavData);

    bool ledFix = (g_lastNavData.posDataExist && (g_lastNavData.posFixMode != FixInvalid));
    Led_isPosfix(ledFix);
  }

  // BME280のデータ取得
  if (currentMillis - previousBme280Millis >= BME280_INTERVAL) {
    previousBme280Millis = currentMillis;
    // BME280データはrecordSensorData内で取得
  }

  // MPU6050のデータ取得
  if (currentMillis - previousMpu6050Millis >= MPU6050_INTERVAL) {
    previousMpu6050Millis = currentMillis;
    // MPU6050データはrecordSensorData内で取得
  }

  // センサーデータを可能な限り高頻度で取得
  recordSensorData();

  // バッファが一杯になったらSDカードに書き込み
  if (bufferIndex >= BUFFER_SIZE) {
    flushBufferToCSV();
  }

  // 一定間隔でシリアル出力
  if (currentMillis - previousSerialMillis >= SERIAL_PRINT_INTERVAL) {
    previousSerialMillis = currentMillis;
    // バッファ内のデータをシリアルに出力
    for (int i = 0; i < bufferIndex; i++) {
      printSensorData(dataBuffer[i]);
    }
  }
}

// GNSS初期化
void initGNSS() {
  int err = 0;
  Gnss.setDebugMode(PrintInfo);

  if (Gnss.begin() != 0) {
    Serial.println("Gnss begin error!!");
    err = 1;
  } else {
    switch (satType) {
      case eSatGps: Gnss.select(GPS); break;
      case eSatGlonass: Gnss.select(GLONASS); break;
      case eSatGpsSbas: Gnss.select(GPS); Gnss.select(SBAS); break;
      case eSatGpsGlonass: Gnss.select(GPS); Gnss.select(GLONASS); break;
      case eSatGpsBeidou: Gnss.select(GPS); Gnss.select(BEIDOU); break;
      case eSatGpsGalileo: Gnss.select(GPS); Gnss.select(GALILEO); break;
      case eSatGpsQz1c: Gnss.select(GPS); Gnss.select(QZ_L1CA); break;
      case eSatGpsQz1cQz1S: Gnss.select(GPS); Gnss.select(QZ_L1CA); Gnss.select(QZ_L1S); break;
      case eSatGpsBeidouQz1c: Gnss.select(GPS); Gnss.select(BEIDOU); Gnss.select(QZ_L1CA); break;
      case eSatGpsGalileoQz1c: Gnss.select(GPS); Gnss.select(GALILEO); Gnss.select(QZ_L1CA); break;
      default:
        Gnss.select(GPS);
        Gnss.select(GLONASS);
        Gnss.select(QZ_L1CA);
        break;
    }
    if (Gnss.start(COLD_START) != 0) {
      Serial.println("Gnss start error!!");
      err = 1;
    } else {
      Serial.println("Gnss setup OK");
    }
  }

  if (err == 1) {
    Led_isError(true);
    while (1);
  }
}

// BME280初期化
void initBME280() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
    errorLoop(3);
  }
}

// MPU6050初期化
void initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    errorLoop(4);
  }
}

// CSVヘッダ初期化およびファイル名の設定
void initCSV() {
  int fileIndex = 0;
  bool fileExists = true;

  // ファイル名を決定
  while (fileExists && fileIndex < 1000) { // 最大999まで
    if (fileIndex == 0) {
      snprintf(currentCSVFilename, sizeof(currentCSVFilename), "%s%s", CSV_BASE_FILENAME, CSV_EXTENSION);
      snprintf(currentLOGFilename, sizeof(currentLOGFilename), "%s%s", LOG_BASE_FILENAME, LOG_EXTENSION);
    } else {
      snprintf(currentCSVFilename, sizeof(currentCSVFilename), "%s_%d%s", CSV_BASE_FILENAME, fileIndex, CSV_EXTENSION);
      snprintf(currentLOGFilename, sizeof(currentLOGFilename), "%s_%d%s", LOG_BASE_FILENAME, fileIndex, LOG_EXTENSION);
    }

    // CSVファイルが存在するかチェック
    fileExists = SD.exists(currentCSVFilename);
    if (fileExists) {
      fileIndex++;
    }
  }

  if (fileIndex >= 1000) {
    Serial.println("Error: Too many CSV files.");
    errorLoop(2);
  }

  // 新しいCSVファイルを作成
  myFile = SD.open(currentCSVFilename, FILE_WRITE);
  if (myFile) {
    // ヘッダーを書き込む
    myFile.println("TimeFromPowerOn(s),Temp(C),Humidity(%),Pressure(hPa),Lat,Lng,GPSAlt(m),Fix,Sats,AccelX(g),AccelY(g),AccelZ(g),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s)");
    myFile.close();
    Serial.print("Logging to: ");
    Serial.println(currentCSVFilename);
  } else {
    Serial.println("Error: Could not create CSV file");
    errorLoop(2);
  }

  // 新しいログファイルを作成
  File logFile = SD.open(currentLOGFilename, FILE_WRITE);
  if (logFile) {
    // ログファイルにヘッダーを書き込む（必要に応じて）
    logFile.println("TimeFromPowerOn(s): Event");
    logFile.close();
    Serial.print("Logging events to: ");
    Serial.println(currentLOGFilename);
  } else {
    Serial.println("Error: Could not create log file");
    errorLoop(2);
  }
}

// センサーデータ取得 & バッファに蓄積
void recordSensorData() {
  if (bufferIndex >= BUFFER_SIZE) {
    return; // バッファが一杯なら取得を一時停止
  }

  sensorDataMsg d;
  d.timeFromPowerOn = (millis() - startTime) / 1000.0;

  d.temperature = bme.readTemperature();
  d.humidity    = bme.readHumidity();
  d.pressure    = bme.readPressure() / 100.0F;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  d.accelX = ax / 16384.0;
  d.accelY = ay / 16384.0;
  d.accelZ = az / 16384.0;
  d.gyroX  = gx / 131.0;
  d.gyroY  = gy / 131.0;
  d.gyroZ  = gz / 131.0;

  d.latitude    = g_lastNavData.latitude;
  d.longitude   = g_lastNavData.longitude;
  d.gpsAltitude = g_lastNavData.altitude;
  d.fix         = g_lastNavData.posFixMode;
  d.satellites  = g_lastNavData.numSatellites;

  // バッファに追加
  dataBuffer[bufferIndex++] = d;
}

// バッファ内のデータをCSVに書き込み
void flushBufferToCSV() {
  File f = SD.open(currentCSVFilename, FILE_WRITE);
  if (f) {
    for (int i = 0; i < bufferIndex; i++) {
      char buf[256];
      int n = snprintf(
        buf,
        sizeof(buf),
        "%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
        dataBuffer[i].timeFromPowerOn,
        dataBuffer[i].temperature, dataBuffer[i].humidity, dataBuffer[i].pressure,
        dataBuffer[i].latitude, dataBuffer[i].longitude, dataBuffer[i].gpsAltitude,
        dataBuffer[i].fix, dataBuffer[i].satellites,
        dataBuffer[i].accelX, dataBuffer[i].accelY, dataBuffer[i].accelZ,
        dataBuffer[i].gyroX, dataBuffer[i].gyroY, dataBuffer[i].gyroZ
      );
      if (n > 0 && n < (int)sizeof(buf)) {
        f.println(buf);
      } else {
        Serial.println("Error: CSV snprintf failed");
      }
    }
    f.close();
    bufferIndex = 0; // バッファをクリア
  } else {
    Serial.println("Error: Could not open CSV file");
  }
}

// イベントログ
void event(String event_msg) {
  float t = (millis() - startTime) / 1000.0;
  String s = String(t, 3) + ": " + event_msg;
  Serial.println(s);

  File lf = SD.open(currentLOGFilename, FILE_WRITE);
  if (lf) {
    lf.println(s);
    lf.close();
  } else {
    Serial.println("Error: Could not open log file");
  }
}

// エラー時にLEDを点滅して停止
void errorLoop(int num) {
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

// LED点滅: ループ動作中
static void Led_isActive() {
  static unsigned long lastToggle = 0;
  static bool state = false;
  unsigned long current = millis();
  if (current - lastToggle >= 500) { // 500ms間隔
    if (state) {
      digitalWrite(PIN_LED0, LOW);
      state = false;
    } else {
      digitalWrite(PIN_LED0, HIGH);
      state = true;
    }
    lastToggle = current;
  }
}

// LED制御: GNSS Fix時
static void Led_isPosfix(bool state) {
  if (state) {
    digitalWrite(PIN_LED1, HIGH);
  } else {
    digitalWrite(PIN_LED1, LOW);
  }
}

// LED制御: エラー時
static void Led_isError(bool state) {
  if (state) {
    digitalWrite(PIN_LED3, HIGH);
  } else {
    digitalWrite(PIN_LED3, LOW);
  }
}

// GNSS位置情報の表示 (シリアル)
static void print_pos(const SpNavData *pNavData) {
  char buf[STRING_BUFFER_SIZE];
  snprintf(buf, STRING_BUFFER_SIZE, "%04d/%02d/%02d ",
           pNavData->time.year, pNavData->time.month, pNavData->time.day);
  Serial.print(buf);

  snprintf(buf, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06ld, ",
           pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
  Serial.print(buf);

  snprintf(buf, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->numSatellites);
  Serial.print(buf);

  if (pNavData->posFixMode == FixInvalid) {
    Serial.print("No-Fix, ");
  } else {
    Serial.print("Fix, ");
  }

  if (pNavData->posDataExist == 0) {
    Serial.print("No Position");
  } else {
    Serial.print("Lat=");
    Serial.print(pNavData->latitude, 6);
    Serial.print(", Lon=");
    Serial.print(pNavData->longitude, 6);
  }

  Serial.println("");
}

// 衛星情報の表示 (シリアル)
static void print_condition(const SpNavData *pNavData) {
  char buf[STRING_BUFFER_SIZE];
  snprintf(buf, STRING_BUFFER_SIZE, "numSatellites:%2d\n", pNavData->numSatellites);
  Serial.print(buf);

  for (unsigned long i = 0; i < pNavData->numSatellites; i++) {
    const char *typeStr = "---";
    SpSatelliteType st = pNavData->getSatelliteType(i);

    switch (st) {
      case GPS:       typeStr = "GPS"; break;
      case GLONASS:   typeStr = "GLN"; break;
      case QZ_L1CA:   typeStr = "QCA"; break;
      case SBAS:      typeStr = "SBA"; break;
      case QZ_L1S:    typeStr = "Q1S"; break;
      case BEIDOU:    typeStr = "BDS"; break;
      case GALILEO:   typeStr = "GAL"; break;
      default:        typeStr = "UKN"; break;
    }

    unsigned long Id  = pNavData->getSatelliteId(i);
    unsigned long Elv = pNavData->getSatelliteElevation(i);
    unsigned long Azm = pNavData->getSatelliteAzimuth(i);
    float sigLevel    = pNavData->getSatelliteSignalLevel(i);

    snprintf(buf, STRING_BUFFER_SIZE,
             "[%2ld] Type:%s, Id:%2ld, Elv:%2ld, Azm:%3ld, CN0:", 
             i, typeStr, Id, Elv, Azm);
    Serial.print(buf);
    Serial.println(sigLevel, 6);
  }
}

// センサーデータをシリアルモニタに出力
void printSensorData(const sensorDataMsg &d) {
  Serial.print("TimeFromPowerOn: ");
  Serial.print(d.timeFromPowerOn, 3);
  Serial.print(" s, Temp: ");
  Serial.print(d.temperature);
  Serial.print(" C, Humi: ");
  Serial.print(d.humidity);
  Serial.print(" %, Press: ");
  Serial.print(d.pressure);
  Serial.print(" hPa, Lat: ");
  Serial.print(d.latitude, 6);
  Serial.print(", Lng: ");
  Serial.print(d.longitude, 6);
  Serial.print(", GPS Alt: ");
  Serial.print(d.gpsAltitude);
  Serial.print(" m, Fix: ");
  Serial.print(d.fix);
  Serial.print(", Sats: ");
  Serial.print(d.satellites);
  Serial.print(", Accel: ");
  Serial.print(d.accelX, 4);
  Serial.print("/");
  Serial.print(d.accelY, 4);
  Serial.print("/");
  Serial.print(d.accelZ, 4);
  Serial.print(" g, Gyro: ");
  Serial.print(d.gyroX, 4);
  Serial.print("/");
  Serial.print(d.gyroY, 4);
  Serial.print("/");
  Serial.print(d.gyroZ, 4);
  Serial.println(" deg/s");
}