#include <Wire.h>              // I2C通信ライブラリ
#include <SoftwareSerial.h>    // ソフトウェアシリアル（ESP32との通信）
#include <Servo.h>             // サーボ制御ライブラリ
#include <Adafruit_MPU6050.h>  // MPU6050用ライブラリ
#include <Adafruit_Sensor.h>   // センサー用ライブラリ

// ピン設定
#define BUTTON_PIN_3 11 // ボタン3のピン番号（閉める）
#define BUTTON_PIN_2 10 // ボタン2のピン番号（開ける）
#define LED_PIN_1 9     // 施錠時のLED
#define LED_PIN_2 8     // 解錠時のLED
#define MOTOR_PIN 6     // モーター制御ピン
#define SERVO_PIN 6     // サーボピン（同じく6番）

// ArduinoからESP32に送信: (TXピン=3), 受信: (RXピン=2)
SoftwareSerial espSerial(2, 3); // RX=2, TX=3

Servo myservo;           // サーボオブジェクト
Adafruit_MPU6050 mpu;    // MPU6050オブジェクト

// ロック状態を管理（0: UNLOCKED(開), 1: LOCKED(閉)）
int lockState = 0;

// LEDの状態（各LEDの状態管理用だが、今回の処理では直接digitalWriteを使用）
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

// サーボ角度の補正（機構によって 0°↔180° 反転させる場合）
int correctedAngle(int angle) {
  return 180 - angle;
}

void setup() {
  // デバッグ用シリアル（ArduinoIDEのシリアルモニタ）を初期化
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

  // 初期状態の設定（LED消灯、モーターON）
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(MOTOR_PIN, HIGH); // モーターは常時ON

  // サーボ初期化（初期位置は「開いている状態」：90度）
  myservo.attach(SERVO_PIN);
  myservo.write(correctedAngle(90));
  delay(500);
  myservo.detach();

  // MPU6050 初期化（失敗時は無限ループ）
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // センサーの設定
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("System initialized.");
}

void loop() {
  // 1. ESP32からのデータ受信 → ロック状態（lockState）を更新
  receiveFromESP32();

  // 2. ボタン入力をチェック → ロック状態（lockState）を更新
  checkButtonPress();

  // 3. MPU6050による加速度センサーで手動操作を検知 → ロック状態（lockState）を更新
  detectManualMovement();

  // 4. ロック状態に応じたサーボ制御とLED更新
  controlServo();
  updateLEDs();

  delay(100);
}

// ======================================================
// (A) ESP32からの状態を受信する関数
// ======================================================
// ESP32から送られてくる文字列に基づいて、ロック状態を更新する。
// 状態が変化した場合、lastActionTimeを更新して手動検知を一定期間無効化する。
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
// ボタンが押された場合にロック状態を更新し、ESP32に通知する。
// また、ボタン押下時刻およびlastActionTimeを記録し、手動検知カウンタをリセットする。
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
// 手動検知では、加速度センサーの値が一定の条件（LOCK: X > 8.5、UNLOCK: X < 2）を
// 連続して3回満たす場合にのみロック状態を変更する。
// なお、lastActionTimeから一定時間内は手動検知をスキップする。
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

  Serial.print("Acceleration X: ");
  Serial.println(accel.acceleration.x);

  // LOCK条件：X軸加速度が8.5より大きい場合
  bool lockCondition = (accel.acceleration.x > 8.5);
  // UNLOCK条件：X軸加速度が2より小さい場合
  bool unlockCondition = (accel.acceleration.x < 2);

  // どちらかの条件を満たす場合、連続カウントを行う
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
// ロック状態が変化した場合に、対応するサーボの動作を行う。
// 動作後はlastActionTimeを更新して、サーボが動いている間は手動検知を無効化する。
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
    delay(500);  // サーボが動作する時間
    myservo.detach();
    lastActionTime = millis(); // サーボ動作後、手動検知を無効化
    previousState = lockState;
  }
}

// ======================================================
// (E) LEDの更新を行う関数
// ======================================================
// ロック状態に応じて、対応するLEDを点灯または消灯する
void updateLEDs() {
  digitalWrite(LED_PIN_1, (lockState == 1) ? HIGH : LOW);
  digitalWrite(LED_PIN_2, (lockState == 0) ? HIGH : LOW);
}
