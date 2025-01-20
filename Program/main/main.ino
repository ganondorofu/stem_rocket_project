#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> // BME280センサー制御用ライブラリ
#include <Servo.h> // サーボモーター制御用ライブラリ

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme; // BME280センサーのインスタンス
Servo servo; // サーボモーターのインスタンス

const int servoPin = 9; // サーボモーター接続ピン
const float thresholdG = 15.0; // 上方向のG検知の閾値 (m/s^2)
const int ledPin = PIN_LED0; // SpresenseのオンボードLEDピン（PIN_LED0などボード仕様に応じる）
const int inputPin = 10; // 入力ピン

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // シリアルポートが準備できるまで待機
  }

  // サーボモーターの初期化
  servo.attach(servoPin);
  servo.write(90); // 初期位置を90度に設定

  // オンボードLEDピンを出力モードに設定
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // 初期状態でLEDをオフに設定

  // 入力ピンを入力モードに設定
  pinMode(inputPin, INPUT);

  // MPU6050を初期化
  if (!mpu.begin()) {
    Serial.println("MPU6050の初期化に失敗しました。接続を確認してください。");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050の初期化に成功しました！");

  // BME280を初期化
  if (!bme.begin(0x76)) { // I2Cアドレスは通常0x76または0x77
    Serial.println("BME280の初期化に失敗しました。接続を確認してください。");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BME280の初期化に成功しました！");

  // センサー設定の調整（必要に応じて変更可能）
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100); // センサーの安定化のため少し待つ
}

void loop() {
  /* 加速度センサーのデータを取得 */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 加速度データを表示 (m/s^2)
  Serial.print("加速度 X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  // ジャイロデータを表示 (°/s)
  Serial.print("ジャイロ X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" °/s");

  // 温度データを表示 (°C)
  Serial.print("温度: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.println("----------------------------------");

  // BME280センサーのデータを取得
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F; // hPaに変換
  float humidity = bme.readHumidity();

  // BME280センサーのデータを表示
  Serial.print("BME280 温度: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("BME280 気圧: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("BME280 湿度: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println("----------------------------------");

  // 上方向の強いGを検知
  if (a.acceleration.z > thresholdG) {
    Serial.println("強いGを検知しました！サーボモーターを動かします。");

    servo.write(0);  // サーボモーターを0度に動かす
    delay(1000);     // 1秒待機
    servo.write(90); // サーボモーターを元の位置（90度）に戻す
  }

  // 入力ピンの状態を読み取る
  int inputState = digitalRead(inputPin);

  // 入力ピンの状態に応じてオンボードLEDを制御
  if (inputState == HIGH) {
    digitalWrite(ledPin, LOW); // LEDをオフ
  } else {
    digitalWrite(ledPin, HIGH); // LEDをオン
  }

  delay(100); // 0.1秒ごとに更新
}
