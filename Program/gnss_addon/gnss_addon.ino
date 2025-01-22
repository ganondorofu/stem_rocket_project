#include <GNSS.h>

#define STRING_BUFFER_SIZE  128       /**< %Buffer size */

#define RESTART_CYCLE       (60 * 5)  /**< positioning test term */

static SpGnssAddon Gnss;

void setup() {
  int error_flag = 0;

  // シリアル通信の初期化
  Serial.begin(115200);

  // LEDの初期化
  ledOn(PIN_LED0);
  ledOn(PIN_LED1);
  ledOn(PIN_LED2);
  ledOn(PIN_LED3);

  // デバッグモードの設定
  Gnss.setDebugMode(PrintInfo);

  // GNSSデバイスのアクティベート
  if (Gnss.begin() != 0) {
    Serial.println("Gnss begin error!!");
    error_flag = 1;
  } else {
    // 測位の開始
    if (Gnss.start() != 0) {
      Serial.println("Gnss start error!!");
      error_flag = 1;
    } else {
      Serial.println("Gnss setup OK");
    }
  }

  // 測位間隔の設定（1Hz）
  Gnss.setInterval(SpInterval_1Hz);

  // LEDの消灯
  ledOff(PIN_LED0);
  ledOff(PIN_LED1);
  ledOff(PIN_LED2);
  ledOff(PIN_LED3);

  // エラーが発生した場合の処理
  if (error_flag == 1) {
    exit(0);
  }
}

void loop() {
  // 測位データの更新待ち
  if (Gnss.waitUpdate(-1)) {
    // 測位データの取得
    SpNavData NavData;
    Gnss.getNavData(&NavData);

    // 測位データの表示
    print_pos(&NavData);
  } else {
    Serial.println("Data not updated");
  }
}

static void print_pos(SpNavData *pNavData) {
  char StringBuffer[STRING_BUFFER_SIZE];

  // 日時の表示
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d %02d:%02d:%02d.%06ld, ",
           pNavData->time.year, pNavData->time.month, pNavData->time.day,
           pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
  Serial.print(StringBuffer);

  // 衛星数の表示
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->numSatellites);
  Serial.print(StringBuffer);

  // 測位状態の表示
  if (pNavData->posFixMode == FixInvalid) {
    Serial.print("No-Fix, ");
  } else {
    Serial.print("Fix, ");
  }

  // 位置情報の表示
  if (pNavData->posDataExist == 0) {
    Serial.print("No Position");
  } else {
    Serial.print("Lat=");
    Serial.print(pNavData->latitude, 6);
    Serial.print(", Lon=");
    Serial.print(pNavData->longitude, 6);
    Serial.print(", Alt=");
    Serial.print(pNavData->altitude, 2);
    Serial.print(" m");
  }

  Serial.println("");
}
