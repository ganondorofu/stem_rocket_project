#include <Servo.h> // サーボ制御ライブラリ

#define BUTTON_PIN_3 11 // ボタン3のピン番号（閉める）
#define BUTTON_PIN_2 10 // ボタン2のピン番号（開ける）
#define LED_PIN_1 9     // 9ピンのLED（開錠時）
#define LED_PIN_2 8     // 8ピンのLED（施錠時）
#define MOTOR_PIN 12    // モーター制御ピン

bool ledState1 = false; // 9ピンLEDの状態（開錠時）
bool ledState2 = false; // 8ピンLEDの状態（施錠時）

Servo myservo;  // サーボオブジェクトを作成

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

  // 初期状態でLEDを消灯、モーターを動作開始
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(MOTOR_PIN, HIGH); // モーターを常にONに設定

  // サーボモーターをピン6に接続
  myservo.attach(6);

  // サーボを初期位置（水平）に設定
  myservo.write(correctedAngle(currentAngle));
  delay(500); // 初期位置に移動後の安定のため
  myservo.detach(); // 初期状態でサーボを空転状態にする

  // シリアル通信の初期化
  Serial.begin(115200);
  Serial.println("Button, LED, Motor, and Servo Test Initialized...");
}

void loop() {
  // ボタンの状態を読み取る
  bool button3State = digitalRead(BUTTON_PIN_3); // ボタン3の状態（閉める）
  bool button2State = digitalRead(BUTTON_PIN_2); // ボタン2の状態（開ける）

  // ボタン2（10ピン）が押された場合（鍵を開ける）
  if (button2State == LOW) { 
    if (currentAngle != 90) { // 現在の角度が0°でない場合
      myservo.attach(6); // サーボ制御を有効化
      currentAngle = 90; // 鍵を開けるために左に90°回転
      myservo.write(correctedAngle(currentAngle));
      Serial.println("Button 2 pressed: Servo set to 90° (locked)");
      ledState1 = true;   // 9ピンLEDをONにする（開錠）
      ledState2 = false;  // 8ピンLEDをOFFにする
      delay(500);         // サーボが動作を完了するまで待機
      myservo.detach();   // サーボを空転状態にする
    }
  }

  // ボタン3（11ピン）が押された場合（鍵を閉める）
  if (button3State == LOW) { 
    if (currentAngle != 0) { // 現在の角度が90°でない場合
      myservo.attach(6); // サーボ制御を有効化
      currentAngle =0; // 鍵を閉めるために右に90°回転
      myservo.write(correctedAngle(currentAngle));
      Serial.println("Button 3 pressed: Servo set to 0° (unlocked)");
      ledState1 = false;  // 9ピンLEDをOFFにする
      ledState2 = true;   // 8ピンLEDをONにする（施錠）
      delay(500);         // サーボが動作を完了するまで待機
      myservo.detach();   // サーボを空転状態にする
    }
  }

  // LEDの状態を更新
  digitalWrite(LED_PIN_1, ledState1 ? HIGH : LOW);
  digitalWrite(LED_PIN_2, ledState2 ? HIGH : LOW);

  // 状態表示のための短い遅延
  delay(100);
}
