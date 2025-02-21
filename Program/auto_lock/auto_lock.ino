/*
  このコードは、MPU6050のエラー発生時に
  ・角度検知（手動検知）を無効化し
  ・Arduino内蔵LED（ERROR_LED）を500ms周期で点滅させる
  ・一定時間（例：5000ms）ごとに再初期化を試み、成功すれば通常動作に復帰する
  という動作を実現しています。
*/

#include <Wire.h>              // I2C通信ライブラリ
#include <SoftwareSerial.h>    // ソフトウェアシリアル（ESP32との通信）
#include <Servo.h>             // サーボ制御ライブラリ
#include <Adafruit_MPU6050.h>  // MPU6050用ライブラリ
#include <Adafruit_Sensor.h>   // センサー用ライブラリ
#include <math.h>              // isnan()を使用するため

// 内蔵LEDの定義（ArduinoによってはLED_BUILTINが定義されていない場合があるので定義）
#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define ERROR_LED LED_BUILTIN  // エラー時に点滅させるLED

// ピン設定
#define BUTTON_PIN_3 11 // ボタン3のピン番号（閉める）
#define BUTTON_PIN_2 10 // ボタン2のピン番号（開ける）
#define LED_PIN_1    9  // 施錠時のLED
#define LED_PIN_2    8  // 解錠時のLED
#define MOTOR_PIN    6  // モーター制御ピン
#define SERVO_PIN    6  // サーボピン（同じく6番）

// ArduinoからESP32に送信: (TXピン=3), 受信: (RXピン=2)
SoftwareSerial espSerial(2, 3); // RX=2, TX=3

Servo myservo;           // サーボオブジェクト
Adafruit_MPU6050 mpu;    // MPU6050オブジェクト

// ロック状態を管理（0: UNLOCKED(開), 1: LOCKED(閉)）
int lockState = 0;

// LEDの状態（今回、直接digitalWriteで更新）
bool ledState1 = false; // 施錠時LED
bool ledState2 = false; // 解錠時LED

// 最後にボタンが押された時刻（ミリ秒）
// ※ボタン押下時に手動検知を無効化するために使用
unsigned long lastButtonPressTime = 0;

// 最後の動作時刻（状態変更、サーボ動作、通信など）
// この変数を使って、状態変化後一定期間は手動検知を行わない
unsigned long lastActionTime = 0;
const unsigned long disableManualDetectionTime = 500; // 500ms間手動検知無効

// 手動検知の連続カウントおよび種類を管理する変数
// manualDetectCounter：連続検知回数
// manualDetectType：現在の検知タイプ (-1: 未設定, 0: UNLOCK条件, 1: LOCK条件)
int manualDetectCounter = 0;
int manualDetectType = -1;

// MPU6050エラー発生フラグ（trueになったら角度検知は無効化し、エラーLED点滅）
bool mpuError = false;

// 再初期化チェック用タイマー（エラー状態時に一定間隔で再初期化を試みる）
unsigned long lastMPUCheckTime = 0;
const unsigned long mpuReinitInterval = 5000; // 5000ms毎に再初期化を試みる

// サーボ角度の補正（機構によって 0°↔180° 反転させる場合）
int correctedAngle(int angle) {
  return 180 - angle;
}

// エラー状態時に内蔵LED（ERROR_LED）を点滅させる関数
void errorMode() {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;
  // 500msごとにLED状態を反転
  if (millis() - lastBlinkTime >= 500) {
    lastBlinkTime = millis();
    ledState = !ledState;
    digitalWrite(ERROR_LED, ledState ? HIGH : LOW);
  }
}

// エラー状態時にMPU6050の再初期化を試みる関数
void checkMPURecovery() {
  if (millis() - lastMPUCheckTime >= mpuReinitInterval) {
    lastMPUCheckTime = millis();
    Serial.println("Attempting MPU6050 reinitialization...");
    if (mpu.begin()) {  // 再初期化に成功した場合
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
      mpuError = false;
      Serial.println("MPU6050 reinitialized successfully. Resuming normal operation.");
      // エラー解除時はERROR_LEDを消灯
      digitalWrite(ERROR_LED, LOW);
    } else {
      Serial.println("MPU6050 reinitialization failed.");
    }
  }
}

void setup() {
  // デバッグ用シリアル（Arduino IDEのシリアルモニタ）を初期化
  Serial.begin(115200);

  // ESP32とのシリアル通信開始
  espSerial.begin(9600);

  // MPU6050用I2C初期化
  Wire.begin();

  // 各ピンのモード設定
  pinMode(BUTTON_PIN_3, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(ERROR_LED, OUTPUT); // 内蔵LED（エラー表示用）

  // 初期状態の設定（LED消灯、モーターON）
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(MOTOR_PIN, HIGH); // モーターは常時ON
  digitalWrite(ERROR_LED, LOW);

  // サーボ初期化（初期位置は「開いている状態」：90度）
  myservo.attach(SERVO_PIN);
  myservo.write(correctedAngle(90));
  delay(500);
  myservo.detach();

  // MPU6050 初期化
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip during setup.");
    mpuError = true;
  } else {
    // センサーの設定
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }

  Serial.println("System initialized.");
}

void loop() {
  // (A) ESP32からの状態受信 → ロック状態（lockState）を更新
  receiveFromESP32();

  // (B) ボタン入力チェック → ロック状態（lockState）を更新
  checkButtonPress();

  // (C) MPU6050による手動操作検知
  // エラー状態の場合は再初期化チェックとLED点滅を行い、角度検知はスキップ
  if (mpuError) {
    checkMPURecovery();
    errorMode();
  } else {
    detectManualMovement();
  }

  // (D) ロック状態に応じたサーボ制御とLED更新
  controlServo();
  updateLEDs();

  delay(100);
}

// ======================================================
// (A) ESP32からの状態を受信する関数
// ======================================================
void receiveFromESP32() {
  if (espSerial.available()) {
    String received = espSerial.readStringUntil('\n');
    received.trim();
    if (received == "LOCKED") {
      if (lockState != 1) {
        lockState = 1;
        Serial.println("Received from ESP32: LOCKED");
        lastActionTime = millis(); // 手動検知を無効化
      }
    } else if (received == "UNLOCKED") {
      if (lockState != 0) {
        lockState = 0;
        Serial.println("Received from ESP32: UNLOCKED");
        lastActionTime = millis(); // 手動検知を無効化
      }
    }
  }
}

// ======================================================
// (B) ボタンが押されたときの処理を行う関数
// ======================================================
void checkButtonPress() {
  bool button3State = digitalRead(BUTTON_PIN_3); // 閉めるボタン
  bool button2State = digitalRead(BUTTON_PIN_2); // 開けるボタン

  // ボタン2が押された場合 → lockStateを1 (LOCKED)に変更
  if (button2State == LOW && lockState != 1) {
    lockState = 1;
    Serial.println("Button 2 pressed: Changing state to LOCKED");
    espSerial.println("LOCKED"); // ESP32に通知
    lastButtonPressTime = millis();
    lastActionTime = millis(); // 手動検知を無効化
    manualDetectCounter = 0;
    manualDetectType = -1;
    delay(200); // チャタリング防止
  }

  // ボタン3が押された場合 → lockStateを0 (UNLOCKED)に変更
  if (button3State == LOW && lockState != 0) {
    lockState = 0;
    Serial.println("Button 3 pressed: Changing state to UNLOCKED");
    espSerial.println("UNLOCKED"); // ESP32に通知
    lastButtonPressTime = millis();
    lastActionTime = millis(); // 手動検知を無効化
    manualDetectCounter = 0;
    manualDetectType = -1;
    delay(200); // チャタリング防止
  }
}

// ======================================================
// (C) MPU6050による手動操作検知を行い、ロック状態を更新する関数
// ======================================================
// センサー値がNaNの場合はエラーとみなし、mpuErrorフラグをtrueにする。
// 正常時は、加速度センサーの値に基づいて連続カウントを行い、条件を満たした場合に状態変更する。
void detectManualMovement() {
  // 最近の状態変更やボタン押下、サーボ動作後は手動検知を無効化
  if (millis() - lastActionTime < disableManualDetectionTime) {
    Serial.println("Skipping manual movement detection due to recent action.");
    return;
  }

  bool button3State = digitalRead(BUTTON_PIN_3);
  bool button2State = digitalRead(BUTTON_PIN_2);

  // どちらかのボタンが押されている場合は検知をスキップし、カウンタをリセットする
  if (button2State == LOW || button3State == LOW) {
    Serial.println("Button is pressed, skipping manual movement detection.");
    manualDetectCounter = 0;
    manualDetectType = -1;
    return;
  }

  // MPU6050から加速度センサーの値を取得する
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // センサー値がNaNの場合、エラー状態にする
  if (isnan(accel.acceleration.x) || isnan(accel.acceleration.y) || isnan(accel.acceleration.z)) {
    if (!mpuError) {
      mpuError = true;
      Serial.println("MPU6050 error detected! Disabling manual detection and entering error mode.");
    }
    return;
  }

  Serial.print("Acceleration X: ");
  Serial.println(accel.acceleration.x);

  // LOCK条件：X軸加速度が8.5より大きい場合
  bool lockCondition = (accel.acceleration.x > 8.5);
  // UNLOCK条件：X軸加速度が2より小さい場合
  bool unlockCondition = (accel.acceleration.x < 2);

  // 条件を満たす場合、連続カウントを行う
  if (lockCondition) {
    if (manualDetectType != 1) {
      manualDetectType = 1;
      manualDetectCounter = 1;
    } else {
      manualDetectCounter++;
    }
    Serial.print("LOCK condition count: ");
    Serial.println(manualDetectCounter);
    if (manualDetectCounter >= 3 && lockState != 1) {
      lockState = 1;
      Serial.println("Manual movement detected (LOCK): Changing state to LOCKED");
      espSerial.println("LOCKED"); // ESP32に通知
      manualDetectCounter = 0;
      manualDetectType = -1;
      lastActionTime = millis(); // 手動検知を無効化
    }
  }
  else if (unlockCondition) {
    if (manualDetectType != 0) {
      manualDetectType = 0;
      manualDetectCounter = 1;
    } else {
      manualDetectCounter++;
    }
    Serial.print("UNLOCK condition count: ");
    Serial.println(manualDetectCounter);
    if (manualDetectCounter >= 3 && lockState != 0) {
      lockState = 0;
      Serial.println("Manual movement detected (UNLOCK): Changing state to UNLOCKED");
      espSerial.println("UNLOCKED"); // ESP32に通知
      manualDetectCounter = 0;
      manualDetectType = -1;
      lastActionTime = millis(); // 手動検知を無効化
    }
  }
  else {
    if (manualDetectCounter > 0) {
      Serial.println("Manual detection condition lost, resetting counter.");
    }
    manualDetectCounter = 0;
    manualDetectType = -1;
  }
}

// ======================================================
// (D) サーボ制御を行う関数（状態変化があったときのみ作動）
// ======================================================
void controlServo() {
  static int previousState = -1; // 前回のlockStateを記憶

  if (lockState != previousState) {
    myservo.attach(SERVO_PIN);
    if (lockState == 1) {
      // LOCKED (閉める)の場合：サーボを補正角度で180度に移動
      myservo.write(correctedAngle(180));
      Serial.println("Servo moving to LOCKED (0°)");
    } else {
      // UNLOCKED (開ける)の場合：サーボを補正角度で90度に移動
      myservo.write(correctedAngle(90));
      Serial.println("Servo moving to UNLOCKED (90°)");
    }
    delay(500);  // サーボ動作時間
    myservo.detach();
    lastActionTime = millis(); // サーボ動作後、手動検知を無効化
    previousState = lockState;
  }
}

// ======================================================
// (E) LEDの更新を行う関数
// ======================================================
void updateLEDs() {
  digitalWrite(LED_PIN_1, (lockState == 1) ? HIGH : LOW);
  digitalWrite(LED_PIN_2, (lockState == 0) ? HIGH : LOW);
}
