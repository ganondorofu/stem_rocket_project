#include <Arduino.h>
#include <GNSS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>
#include <math.h> // sqrt関数を使うのに必要

// LEDピンの定義
#define PIN_LED0 2 // アクティブLED
#define PIN_LED1 3 // GNSS Fix LED
#define PIN_LED3 4 // エラーLED

// GNSS更新レート（Hz）
#define GNSS_UPDATE_RATE 10

// パラシュート展開の閾値
#define DEPLOY_PRESSURE_RATE_THRESHOLD -0.5f  // hPa/s
#define GYRO_ROTATION_THRESHOLD 100.0f        // °/s

// パラシュート展開用のリレー接続ピン
const int RELAY_PIN = 5;

// GNSS, BME280, MPU6050 のインスタンス
SpGnss Gnss;
Adafruit_BME280 bme;
MPU6050 mpu;

// パラシュート展開フラグ
bool parachuteDeployed = false;

// 気圧変化率の管理用変数
float previousPressureValue = 0.0f;
unsigned long previousPressureTime = 0;

// センサー更新インターバル（ミリ秒）
#define BME280_INTERVAL 50
#define MPU6050_INTERVAL 50

// エラー時にLEDを点滅させて停止する関数
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

// LED制御関数
static void Led_isActive() {
  static unsigned long lastToggle = 0;
  static bool state = false;
  unsigned long current = millis();
  if (current - lastToggle >= 500) {
    digitalWrite(PIN_LED0, state ? LOW : HIGH);
    state = !state;
    lastToggle = current;
  }
}

static void Led_isPosfix(bool state) {
  digitalWrite(PIN_LED1, state ? HIGH : LOW);
}

static void Led_isError(bool state) {
  digitalWrite(PIN_LED3, state ? HIGH : LOW);
}

// GNSS初期化
void initGNSS() {
  if (Gnss.begin() != 0 || Gnss.start(COLD_START) != 0) {
    errorLoop(1);
  }
  Gnss.select(GPS);
  Gnss.select(GLONASS);
}

// BME280初期化
void initBME280() {
  if (!bme.begin(0x76)) {
    errorLoop(3);
  }
}

// MPU6050初期化
void initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    errorLoop(4);
  }
}

// setup
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // LEDピン設定
  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  
  // リレーピン設定
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Wire.begin();

  // センサー初期化
  initGNSS();
  initBME280();
  initMPU6050();

  // 初期気圧と時刻を記録（誤作動防止）
  delay(1000);
  previousPressureValue = bme.readPressure() / 100.0f; // hPa
  previousPressureTime = millis() + 1000;  // 初回変化率計算を防ぐ

  Serial.println("セットアップ完了");
}

// loop
void loop() {
  unsigned long currentMillis = millis();
  Led_isActive();

  // GNSSの更新チェック
  if (Gnss.waitUpdate(0)) {
    SpNavData NavData;
    Gnss.getNavData(&NavData);
    Led_isPosfix(NavData.posDataExist && (NavData.posFixMode != FixInvalid));
  }

  // BME280（気圧）の取得
  float currentPressure = bme.readPressure() / 100.0f; // hPa
  unsigned long currentTime = millis();

  // 時間経過チェック
  float deltaTime = (currentTime - previousPressureTime) / 1000.0f;
  if (deltaTime < 1.0f) {
    deltaTime = 1.0f; // 最低1秒以上
  }

  float deltaPressure = currentPressure - previousPressureValue;
  float rateOfPressureChange = deltaPressure / deltaTime;

  // MPU6050（ジャイロ）の取得
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 角速度（°/s）の計算
  float gyroX = gx / 131.0f;
  float gyroY = gy / 131.0f;
  float gyroZ = gz / 131.0f;
  float gyroMagnitude = sqrtf(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  // パラシュート展開条件
  if (!parachuteDeployed) {
    bool isFalling = (rateOfPressureChange <= DEPLOY_PRESSURE_RATE_THRESHOLD);
    bool isGyroDown = (gyroMagnitude > GYRO_ROTATION_THRESHOLD);

    if (isFalling || isGyroDown) {
      digitalWrite(RELAY_PIN, HIGH);
      parachuteDeployed = true;
      Serial.print("パラシュート展開：");
      if (isFalling) Serial.print("気圧変化率 ");
      if (isGyroDown) Serial.print("ジャイロ回転速度 ");
      Serial.println();

      // パラシュート展開後に一定時間経過後OFF
      delay(3000);
      digitalWrite(RELAY_PIN, LOW);
    }
  }

  // 前回の気圧と時間を更新
  previousPressureValue = currentPressure;
  previousPressureTime = currentTime;

  delay(10);
}
