#include <Arduino.h>
#include <GNSS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>
#include <math.h> // sqrt関数を使うのに必要

// LEDピンの定義（必要に応じて変更）
#define PIN_LED0 2 // アクティブLED
#define PIN_LED1 3 // GNSS Fix LED
#define PIN_LED3 4 // エラーLED

// GNSS更新レート（Hz）
#define GNSS_UPDATE_RATE 10

// パラシュート展開の閾値（気圧変化率 hPa/s）
// 例： -0.5 の場合、1秒あたり0.5 hPa以上の低下を「降下」とみなす
#define DEPLOY_PRESSURE_RATE_THRESHOLD -0.5f

// ジャイロ回転速度の閾値（°/s）
// どの軸かは分からない前提なので、3軸の回転ベクトルの大きさで判断します
// 例：100.0f なら、回転速度ベクトルの大きさが100°/sを超えたら展開
#define GYRO_ROTATION_THRESHOLD 100.0f

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

// タイミング管理用
unsigned long startTime = 0;
unsigned long previousBme280Millis = 0;
unsigned long previousMpu6050Millis = 0;

// センサー更新インターバル（ミリ秒）
#define BME280_INTERVAL 50    // 50msごとにBME280を取得
#define MPU6050_INTERVAL 50   // 50msごとにMPU6050を取得

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

// LED点滅: ループ動作中
static void Led_isActive() {
  static unsigned long lastToggle = 0;
  static bool state = false;
  unsigned long current = millis();
  if (current - lastToggle >= 500) { // 500ms間隔で点滅
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

// ----------------------------------------------------------------------------
// GNSS初期化
// ----------------------------------------------------------------------------
void initGNSS() {
  int err = 0;
  // GNSSの開始
  if (Gnss.begin() != 0) {
    err = 1;
  } else {
    // 衛星選択（例：GPS + GLONASS）
    Gnss.select(GPS);
    Gnss.select(GLONASS);
    // コールドスタート
    if (Gnss.start(COLD_START) != 0) {
      err = 1;
    }
  }

  if (err == 1) {
    Led_isError(true);
    // GNSS初期化エラー
    while (true);
  }
}

// ----------------------------------------------------------------------------
// BME280初期化
// ----------------------------------------------------------------------------
void initBME280() {
  if (!bme.begin(0x76)) {
    // BME280が見つからない
    errorLoop(3);
  }
}

// ----------------------------------------------------------------------------
// MPU6050初期化
// ----------------------------------------------------------------------------
void initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    // MPU6050接続失敗
    errorLoop(4);
  }
}

// ----------------------------------------------------------------------------
// setup
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // LEDピンの設定
  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);

  // リレー（パラシュート展開制御ピン）の設定
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Wire.begin();

  // センサー初期化
  initGNSS();
  initBME280();
  initMPU6050();

  // 開始時刻記録
  startTime = millis();

  // 初期気圧と時刻を記録
  previousPressureValue = bme.readPressure() / 100.0f; // hPa
  previousPressureTime = millis();

  Serial.println("セットアップ完了");
}

// ----------------------------------------------------------------------------
// loop
// ----------------------------------------------------------------------------
void loop() {
  unsigned long currentMillis = millis();

  // LED点滅（動作中サイン）
  Led_isActive();

  // GNSSの更新チェック（非ブロッキング）
  bool updated = Gnss.waitUpdate(0);
  if (updated) {
    // GNSSデータ取得
    SpNavData NavData;
    Gnss.getNavData(&NavData);

    // Fix状態に応じてLEDを制御
    bool ledFix = (NavData.posDataExist && (NavData.posFixMode != FixInvalid));
    Led_isPosfix(ledFix);
  }

  // BME280のデータ取得タイミング
  if (currentMillis - previousBme280Millis >= BME280_INTERVAL) {
    previousBme280Millis = currentMillis;
    // 必要なら BME280 を再度読み取る
  }

  // MPU6050のデータ取得タイミング
  if (currentMillis - previousMpu6050Millis >= MPU6050_INTERVAL) {
    previousMpu6050Millis = currentMillis;
    // 必要なら MPU6050 を再度読み取る
  }

  // --------------------------------------------------------------------------
  // 気圧変化率チェック
  // --------------------------------------------------------------------------
  float currentPressure = bme.readPressure() / 100.0f; // hPa
  unsigned long currentTime = millis();

  float deltaTime = (currentTime - previousPressureTime) / 1000.0f;
  if (deltaTime <= 0) {
    deltaTime = 1e-3; // ゼロ除算回避
  }

  float deltaPressure = currentPressure - previousPressureValue;
  float rateOfPressureChange = deltaPressure / deltaTime; // hPa/s

  // --------------------------------------------------------------------------
  // ジャイロ回転速度をチェック（姿勢崩れを推定）
  // --------------------------------------------------------------------------
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 生の角速度をスケーリング（131.0 で割ると [deg/s]）
  float gyroX = gx / 131.0f;
  float gyroY = gy / 131.0f;
  float gyroZ = gz / 131.0f;

  // 3軸の角速度ベクトルの大きさを計算
  float gyroMagnitude = sqrtf(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  // --------------------------------------------------------------------------
  // パラシュート展開条件
  // 1) 気圧変化率が閾値以下 -> 降下していると判断
  // 2) ジャイロ回転速度が閾値以上 -> 下向き or 姿勢崩れと判断
  // --------------------------------------------------------------------------
  if (!parachuteDeployed) {
    bool isFalling = (rateOfPressureChange <= DEPLOY_PRESSURE_RATE_THRESHOLD);
    bool isGyroDown = (gyroMagnitude > GYRO_ROTATION_THRESHOLD);

    // いずれかの条件を満たせば展開
    if (isFalling || isGyroDown) {
      digitalWrite(RELAY_PIN, HIGH);
      parachuteDeployed = true;

      // どのセンサーが原因だったかを表示
      if (isFalling && isGyroDown) {
        Serial.println("パラシュート展開：気圧変化率 & ジャイロ回転速度");
      } else if (isFalling) {
        Serial.println("パラシュート展開：気圧変化率");
      } else {
        Serial.println("パラシュート展開：ジャイロ回転速度");
      }
    }
  }

  // 前回の気圧と時間を更新
  previousPressureValue = currentPressure;
  previousPressureTime = currentTime;

  delay(10);
}
