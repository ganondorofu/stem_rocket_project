#include <Camera.h>
#include <SDHCI.h>
#include "AviLibrary.h"

SDClass theSD;
File aviFile;

AviLibrary theAvi;

String filename = "movie.avi";

void CamCB(CamImage img) {
  const static uint32_t recording_time_in_ms = 600000;  /* msec */ // ここの値を変えると録画時間が変わる
  static uint32_t start_time = millis();

  if (img.isAvailable()) {
   uint32_t duration = millis() - start_time;
    if (duration > recording_time_in_ms) {
      theAvi.endRecording();
      theAvi.end();
      Serial.println("Movie saved");
      Serial.println(" Movie width:    " + String(theAvi.getWidth()));
      Serial.println(" Movie height:   " + String(theAvi.getHeight()));
      Serial.println(" File size (kB): " + String(theAvi.getFileSize()));
      Serial.println(" Captured Frame: " + String(theAvi.getTotalFrame())); 
      Serial.println(" Duration (sec): " + String(theAvi.getDuration()));
      Serial.println(" Frame per sec : " + String(theAvi.getFps()));
      Serial.println(" Max data rate : " + String(theAvi.getMaxDataRate()));
      theCamera.end();
      while (true) {
        digitalWrite(LED0, HIGH);
        delay(100);
        digitalWrite(LED0, LOW);
        delay(100);
      }
    } else {  
      theAvi.addFrame(img.getImgBuff(), img.getImgSize());
    }
  }
}


void setup() {
  Serial.begin(115200);
  while (!theSD.begin()) { Serial.println("insert SD card"); }

  const int buff_num = 2;
  theCamera.begin(buff_num, CAM_VIDEO_FPS_30 
    ,CAM_IMGSIZE_QVGA_H ,CAM_IMGSIZE_QVGA_V ,CAM_IMAGE_PIX_FMT_JPG ,5);

  theSD.remove(filename);
  aviFile = theSD.open(filename, FILE_WRITE);

  theAvi.begin(aviFile, CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V);
  theCamera.startStreaming(true, CamCB);

  Serial.println("Start recording...");
  theAvi.startRecording();
}

void loop() {}


/*
参照元
https://ja.stackoverflow.com/questions/91100/%E5%8B%95%E7%94%BB%E6%92%AE%E5%BD%B1%E3%81%AE%E9%AB%98%E9%80%9F%E5%8C%96

AviLibraryは手動で入れないと行けないお
stackoverflowに載ってるgithubからコードのzipをdl
IDEのスケッチ->ライブラリのインクルード->zip
でdlしたファイルを選択
*/