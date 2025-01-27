#include <Servo.h> // サーボ制御ライブラリ
#include <Wire.h> // I2C通信ライブラリ
#include <Adafruit_MPU6050.h> // MPU6050用ライブラリ
#include <Adafruit_Sensor.h> // センサー用ライブラリ

#define BUTTON_PIN_3 11 // ボタン3のピン番号（閉める）
#define BUTTON_PIN_2 10 // ボタン2のピン番号（開ける）
#define LED_PIN_1 9     // 9ピンのLED（施鎖時）
#define LED_PIN_2 8     // 8ピンのLED（開鎖時）
#define MOTOR_PIN 12    // モーター制御ピン

bool ledState1 = false; // 9ピンLEDの状態（施鎖時）
bool ledState2 = false; // 8ピンLEDの状態（開鎖時）

Servo myservo;  // サーボオブジェクトを作成
Adafruit_MPU6050 mpu; // MPU6050オブジェクト

// サーボの現在位置を保持する
int currentAngle = 90; // 初期状態は水平（鍵が抜ける状態）

// サーボ補正用の反転関数
int correctedAngle(int angle) {
  return 180 - angle; // サーボモーターの角度を左右反転
}

void setup() {
  // ボタンピンを入力プルアップモードで設定
  pinMode(BUTTON_PIN_3, INPUT_PULLUP); // ボタン3: プルアップ設定
  pinMode(BUTTON_PIN_2, INPUT_PULLUP); // ボタン2: プルアップ設定

  // LEDピンを出力モードに設定
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);

  // モーター制御ピンを出力モードに設定
  pinMode(MOTOR_PIN, OUTPUT);

  // 初期状態でLEDを消燃、モーターを動作開始
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(MOTOR_PIN, HIGH); // モーターを常にONに設定

  // サーボモーターをピン6に接続
  myservo.attach(6);

  // MPU6050の初期化
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // MPU6050の設定
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // サーボを初期位置（水平）に設定
  myservo.write(correctedAngle(currentAngle));
  delay(500); // 初期位置に移動後の安定のため
  myservo.detach(); // 初期状態でサーボを空転状態にする

  // シリアル通信の初期化
  Serial.begin(115200);
  Serial.println("Button, LED, Motor, and Servo with MPU6050 Initialized...");
}

void loop() {
  // ボタンの状態を読み取る
  bool button3State = digitalRead(BUTTON_PIN_3); // ボタン3の状態（閉める）
  bool button2State = digitalRead(BUTTON_PIN_2); // ボタン2の状態（開ける）

  // ボタン2（10ピン）が押された場合（鍵を閉める）
  if (button2State == LOW) { 
    if (currentAngle != 180) { // 現在の角度が180°でない場合
      myservo.attach(6); // サーボ制御を有効化
      currentAngle = 180; // 鍵を閉めるために右に90°回転
      myservo.write(correctedAngle(currentAngle));
      Serial.println("Button 2 pressed: Servo set to 0° (locked)");
      ledState1 = true;   // 9ピンLEDをONにする（施鎖）
      ledState2 = false;  // 8ピンLEDをOFFにする
      delay(700);         // サーボが動作を完了するまで待機
      myservo.detach();   // サーボを空転状態にする

      // サーボ移動完了後にMPU6050からデータを取得
      getMPUData();
    }
  }

  // ボタン3（11ピン）が押された場合（鍵を開ける）
  if (button3State == LOW) { 
    if (currentAngle != 90) { // 現在の角度が90°でない場合
      myservo.attach(6); // サーボ制御を有効化
      currentAngle = 90; // 鍵を開けるために左に90°回転
      myservo.write(correctedAngle(currentAngle));
      Serial.println("Button 3 pressed: Servo set to 90° (unlocked)");
      ledState1 = false;  // 9ピンLEDをOFFにする
      ledState2 = true;   // 8ピンLEDをONにする（開鎖）
      delay(700);         // サーボが動作を完了するまで待機
      myservo.detach();   // サーボを空転状態にする

      // サーボ移動完了後にMPU6050からデータを取得
      getMPUData();
    }
  }

  // 手動操作を検出し、状態を更新
  detectManualMovement();

  // LEDの状態を更新
  digitalWrite(LED_PIN_1, ledState1 ? HIGH : LOW);
  digitalWrite(LED_PIN_2, ledState2 ? HIGH : LOW);

  // 状態表示のための短い過渡
  delay(100);
}

// MPU6050からデータを取得して表示
void getMPUData() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // データをシリアルモニタに表示
  Serial.print("Accelerometer (m/s^2): X=");
  Serial.print(accel.acceleration.x);
  Serial.print(", Y=");
  Serial.print(accel.acceleration.y);
  Serial.print(", Z=");
  Serial.println(accel.acceleration.z);

  Serial.print("Gyroscope (rad/s): X=");
  Serial.print(gyro.gyro.x);
  Serial.print(", Y=");
  Serial.print(gyro.gyro.y);
  Serial.print(", Z=");
  Serial.println(gyro.gyro.z);

  Serial.print("Temperature (°C): ");
  Serial.println(temp.temperature);
  Serial.println("------------------------------------");
}

// 手動操作を検出して状態を更新
void detectManualMovement() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // 簡易的にX軸の加速度で角度を判断
  if (accel.acceleration.x > 8.5) { // 閾値を調整して施鎖状態に対応
    if (currentAngle != 180) {
      currentAngle = 180;
      Serial.println("Manual movement detected: Locked position");
      ledState1 = true;
      ledState2 = false;
    }
  } else if (accel.acceleration.x < 2) { // 閾値を調整して開鎖状態に対応
    if (currentAngle != 90) {
      currentAngle = 90;
      Serial.println("Manual movement detected: Unlocked position");
      ledState1 = false;
      ledState2 = true;
    }
  }
}
