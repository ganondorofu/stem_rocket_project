#include <Camera.h>
#include <SDHCI.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

SDClass theSD;
MPU6050 mpu;
Servo servo;

String baseFilename = "frame_";  // ファイル名のベース
int frameCount = 0;  // 保存されたフレーム数のカウント
bool isLifted = false;  // 持ち上げた状態を表すフラグ
int16_t prevAz = 0;  // 前回のZ軸加速度

void CamCB(CamImage img) {
  const static uint32_t recording_time_in_ms = 60000;  /* msec */
  static uint32_t start_time = millis();

  if (img.isAvailable()) {
    uint32_t duration = millis() - start_time;
    if (duration > recording_time_in_ms) {
      // 録画終了
      theCamera.end();
      Serial.println("Recording ended.");
      while (true) {
        digitalWrite(LED0, HIGH);
        delay(100);
        digitalWrite(LED0, LOW);
        delay(100);
      }
    } else {
      // フレームごとに画像を保存
      String filename = baseFilename + String(frameCount) + ".jpg";  // 例: frame_0.jpg, frame_1.jpg
      File imgFile = theSD.open(filename, FILE_WRITE);

      if (imgFile) {
        imgFile.write(img.getImgBuff(), img.getImgSize());  // 画像バッファをSDカードに保存
        imgFile.close();
        Serial.println("Saved: " + filename);
        frameCount++;  // フレームカウントを更新
      } else {
        Serial.println("Failed to save: " + filename);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  // SDカードのチェック
  while (!theSD.begin()) {
    Serial.println("Insert SD card");
    delay(1000);
  }

  // Cameraの設定
  const int buff_num = 2;
  theCamera.begin(buff_num, CAM_VIDEO_FPS_30, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, CAM_IMAGE_PIX_FMT_JPG, 5);

  // ストリーミングを開始
  theCamera.startStreaming(true, CamCB);
  Serial.println("Start capturing frames...");

  // MPU6050の設定
  Wire.begin();  // SDA=D14, SCL=D15を使用
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 connected");

  // Servoの設定
  servo.attach(8);  // Servoを8番ピンに接続
  servo.write(0);  // 初期位置
}

void loop() {
  // MPU6050から値を取得
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Z軸の変化量を計算
  int16_t deltaAz = abs(az - prevAz);

  // 閾値（変化量）で判定
  if (deltaAz > 2000 && !isLifted) {  // 変化量の閾値は調整可能
    isLifted = true;
    Serial.println("Lift detected, moving servo");

    // Servoを動かす
    servo.write(90);  // 90度の位置
    delay(1000);  // 1秒間位置を保持
    servo.write(0);  // 元の位置に戻す
    delay(1000);
    isLifted = false;
  }

  // 現在の加速度を保存
  prevAz = az;
}
