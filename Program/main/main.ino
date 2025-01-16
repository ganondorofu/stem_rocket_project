#include <Camera.h>
#include <SDHCI.h>

SDClass theSD;

String baseFilename = "frame_";  // ファイル名のベース
int frameCount = 0;  // 保存されたフレーム数のカウント

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
  while (!theSD.begin()) {
    Serial.println("Insert SD card");
    delay(1000);
  }

  const int buff_num = 2;
  theCamera.begin(buff_num, CAM_VIDEO_FPS_30, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, CAM_IMAGE_PIX_FMT_JPG, 5);

  // ストリーミングを開始
  theCamera.startStreaming(true, CamCB);

  Serial.println("Start capturing frames...");
}

void loop(){

}