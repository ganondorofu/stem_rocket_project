#include <Servo.h> // サーボ制御ライブラリ
#include <Wire.h> // I2C通信ライブラリ
#include <Adafruit_MPU6050.h> // MPU6050用ライブラリ
#include <Adafruit_Sensor.h> // センサー用ライブラリ

#define BUTTON_PIN_3 11 // ボタン3のピン番号（閉める）
#define BUTTON_PIN_2 10 // ボタン2のピン番号（開ける）
#define LED_PIN_1 9     // 9ピンのLED（施鎖時）
#define LED_PIN_2 8     // 8ピンのLED（開鎖時）
#define MOTOR_PIN 6    // モーター制御ピン

Servo myservo;  // サーボオブジェクト
Adafruit_MPU6050 mpu; // MPU6050オブジェクト

// ロック状態を管理（0: 開いている, 1: 閉まっている）
int lockState = 0;

// LEDの状態
bool ledState1 = false; // 施錠時のLED
bool ledState2 = false; // 解錠時のLED

// サーボの補正（角度反転）
int correctedAngle(int angle) {
  return 180 - angle;
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN_3, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(MOTOR_PIN, HIGH); // モーター常時ON

  myservo.attach(6);
  myservo.write(correctedAngle(90)); // 初期位置（開いている状態）
  delay(500);
  myservo.detach();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("System initialized.");
}

void loop() {
  bool button3State = digitalRead(BUTTON_PIN_3); // 閉める
  bool button2State = digitalRead(BUTTON_PIN_2); // 開ける

  // ボタンを押したら状態を変更（直接サーボは動かさない）
  if (button2State == LOW) { // 閉める
    lockState = 1;
    Serial.println("Button 2 pressed: Changing state to LOCKED");
    delay(200); // チャタリング防止
  }
  if (button3State == LOW) { // 開ける
    lockState = 0;
    Serial.println("Button 3 pressed: Changing state to UNLOCKED");
    delay(200); // チャタリング防止
  }

  // ボタンが押されていない場合のみ手動操作を検出
  if (button2State == HIGH && button3State == HIGH) {
    detectManualMovement();
  } else {
    Serial.println("Button is pressed, skipping manual movement detection.");
  }

  // 状態に応じてサーボを制御
  controlServo();

  // LEDの更新
  digitalWrite(LED_PIN_1, lockState == 1 ? HIGH : LOW);
  digitalWrite(LED_PIN_2, lockState == 0 ? HIGH : LOW);

  delay(100);
}

// サーボを制御する関数（状態変化があった時のみ動作）
void controlServo() {
  static int previousState = -1; // 前回の状態を記憶

  Serial.print("Current lockState: ");
  Serial.println(lockState);
  
  if (lockState != previousState) { // 状態が変化した時のみ動作
    myservo.attach(6);
    if (lockState == 1) {
      myservo.write(correctedAngle(180)); // 閉める
      Serial.println("Servo moving to LOCKED (0°)");
    } else {
      myservo.write(correctedAngle(90)); // 開ける
      Serial.println("Servo moving to UNLOCKED (90°)");
    }
    delay(200);
    myservo.detach();
    
    previousState = lockState; // 状態を更新
  }
}

// 手動操作を検出して状態を更新
void detectManualMovement() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Serial.print("Acceleration X: "); 
  Serial.println(accel.acceleration.x);

  if (accel.acceleration.x > 8.5) { // 施錠方向に手動で動かした
    if (lockState != 1) {
      lockState = 1;
      Serial.println("Manual movement detected: Changing state to LOCKED");
    }
  } else if (accel.acceleration.x < 2) { // 解錠方向に手動で動かした
    if (lockState != 0) {
      lockState = 0;
      Serial.println("Manual movement detected: Changing state to UNLOCKED");
    }
  }
}
