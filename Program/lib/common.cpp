#include "common.h"

void initFlash(bool formatOnBoot, bool &flashInitialized) {
  if (formatOnBoot) {
    Serial.println("Formatting internal Flash... (on boot)");
    if (!Flash.format()) {
      Serial.println("Warning: Flash.format() failed!");
    } else {
      Serial.println("Flash format done.");
    }
  } else {
    Serial.println("Skipping Flash format on boot.");
  }
  if (!Flash.begin()) {
    Serial.println("Warning: Flash.begin() failed!");
  } else {
    Serial.println("Flash.begin() success.");
  }
  flashInitialized = true;
}

bool initGNSS(SpGnssAddon &Gnss) {
  int ret = Gnss.begin();
  if (ret != 0) {
    Serial.println("Error: Gnss.begin() fail");
    return false;
  }
  Gnss.setInterval(SpInterval_10Hz);
  ret = Gnss.start();
  if (ret != 0) {
    Serial.println("Error: Gnss.start() fail");
    return false;
  }
  Serial.println("GNSS setup OK (10Hz)");
  return true;
}

bool initBME280(Adafruit_BME280 &bme) {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
    return false;
  }
  Serial.println("BME280 setup OK (50ms)");
  return true;
}

bool initMPU6050(MPU6050 &mpu) {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    return false;
  }
  Serial.println("MPU6050 setup OK (10ms)");
  return true;
}

bool initSDandCSV(SDClass &SD, String &csvFilename, String &logFilename,
                  bool &sdErrorHappened,
                  const char *CSV_BASE, const char *CSV_EXT,
                  const char *LOG_BASE, const char *LOG_EXT,
                  void (*handleError)(int)) {
  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed.");
    if (handleError) handleError(1);
    sdErrorHappened = true;
    return false;
  }
  Serial.println("SD card setup OK");
  int idx = 0;
  while (true) {
    String csvCandidate;
    String logCandidate;
    if (idx == 0) {
      csvCandidate = String(CSV_BASE) + String(CSV_EXT);
      logCandidate = String(LOG_BASE) + String(LOG_EXT);
    } else {
      csvCandidate = String(CSV_BASE) + String(idx) + String(CSV_EXT);
      logCandidate = String(LOG_BASE) + String(idx) + String(LOG_EXT);
    }
    bool csvExist = SD.exists(csvCandidate);
    bool logExist = SD.exists(logCandidate);
    if (!csvExist && !logExist) {
      File csvFile = SD.open(csvCandidate, FILE_WRITE);
      if (!csvFile) {
        Serial.println("Error: Could not create CSV file");
        if (handleError) handleError(2);
        sdErrorHappened = true;
        return false;
      }
      csvFile.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s,TotalAccel");
      csvFile.close();
      csvFilename = csvCandidate;

      File lf = SD.open(logCandidate, FILE_WRITE);
      if (!lf) {
        Serial.println("Error: Could not create LOG file");
        if (handleError) handleError(2);
        sdErrorHappened = true;
        return false;
      }
      lf.println("Time_s:Event");
      lf.close();
      logFilename = logCandidate;

      Serial.print("Logging sensor data to: ");
      Serial.println(csvFilename);
      Serial.print("Logging events to: ");
      Serial.println(logFilename);
      return true;
    }
    idx++;
  }
}

bool initPreFlightCSV(SDClass &SD, String &preFlightFilename,
                      const char *PRE_FLIGHT_BASE, const char *PRE_FLIGHT_EXT,
                      void (*handleError)(int)) {
  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed for pre-flight logging.");
    return false;
  }
  int idx = 0;
  while (true) {
    String candidate;
    if (idx == 0) {
      candidate = String(PRE_FLIGHT_BASE) + String(PRE_FLIGHT_EXT);
    } else {
      candidate = String(PRE_FLIGHT_BASE) + String(idx) + String(PRE_FLIGHT_EXT);
    }
    if (!SD.exists(candidate)) {
      File pfFile = SD.open(candidate, FILE_WRITE);
      if (!pfFile) {
        Serial.println("Error: Could not create pre-flight CSV file");
        if (handleError) handleError(2);
        return false;
      }
      pfFile.println("Time_s,Temperature_C,Humidity_%,Pressure_hPa,Lat,Lng,Alt_m,Fix,Sats,AccelX_g,AccelY_g,AccelZ_g,GyroX_deg_s,GyroY_deg_s,GyroZ_deg_s,TotalAccel");
      pfFile.close();
      preFlightFilename = candidate;
      Serial.print("Logging pre-flight sensor data to: ");
      Serial.println(preFlightFilename);
      return true;
    }
    idx++;
  }
}

void logEvent(SDClass &SD, const String &logFilename, bool &sdErrorHappened,
              unsigned long startTime, const String &msg,
              void (*handleError)(int)) {
  float t = (millis() - startTime) / 1000.0f;
  String s = String(t, 3) + ": " + msg;
  Serial.println(s);
  if (sdErrorHappened) {
    File f = Flash.open("event.txt", FILE_WRITE);
    if (!f) {
      Serial.println("Error: Could not open event.txt in Flash");
      if (handleError) handleError(9);
      return;
    }
    f.println(s);
    f.close();
    return;
  }
  File lf = SD.open(logFilename, FILE_WRITE);
  if (lf) {
    lf.println(s);
    lf.close();
  } else {
    Serial.println("Error: Could not open log file");
    if (handleError) handleError(2);
    sdErrorHappened = true;
  }
}

