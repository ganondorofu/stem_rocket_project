#include <Arduino.h>
#include <SDHCI.h>
#include <File.h>
#include <Camera.h>
#include <GNSS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define CSV_FILENAME "sensor_data.csv"
#define LOG_FILENAME "event_log.txt"
#define STRING_BUFFER_SIZE  128
#define RESTART_CYCLE       (60 * 5)

SDClass SD;
File myFile;
SpGnss Gnss;
Adafruit_BME280 bme;
MPU6050 mpu;

int imageCounter = 0;
unsigned long startTime;

enum ParamSat {
  eSatGps,
  eSatGlonass,
  eSatGpsSbas,
  eSatGpsGlonass,
  eSatGpsBeidou,
  eSatGpsGalileo,
  eSatGpsQz1c,
  eSatGpsGlonassQz1c,
  eSatGpsBeidouQz1c,
  eSatGpsGalileoQz1c,
  eSatGpsQz1cQz1S,
};

static enum ParamSat satType = eSatGpsGlonass;

struct sensorDataMsg {
  float temperature;
  float humidity;
  float pressure;
  float altitude;
  float latitude;
  float longitude;
  float gpsAltitude;
  int fix;
  int satellites;
  char imageFilename[32];
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  startTime = millis();

  Wire.begin();
  initCamera();
  initGNSS();
  initBME280();
  initMPU6050();

  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed. Please check the SD card.");
    while (1);
  }

  initCSV();

  event("All initialization completed");
}

static int LoopCount = 0;
static int LastPrintMin = 0;

void loop() {
  Led_isActive();

  if (Gnss.waitUpdate(-1)) {
    SpNavData NavData;
    Gnss.getNavData(&NavData);

    bool LedSet = (NavData.posDataExist && (NavData.posFixMode != FixInvalid));
    Led_isPosfix(LedSet);

    if (NavData.time.minute != LastPrintMin) {
      print_condition(&NavData);
      LastPrintMin = NavData.time.minute;
    }

    print_pos(&NavData);
    
    recordSensorData(NavData);
    captureImage();
  } else {
    Serial.println("GNSS data not updated");
  }

  LoopCount++;
  if (LoopCount >= RESTART_CYCLE) {
    restartGNSS();
    LoopCount = 0;
  }

  delay(1000);
}

void initCamera() {
  CamErr err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
    errorLoop(1);
  }

  err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QUADVGA_H,
    CAM_IMGSIZE_QUADVGA_V,
    CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
    errorLoop(1);
  }
}

void initGNSS() {
  int error_flag = 0;

  Gnss.setDebugMode(PrintInfo);

  int result = Gnss.begin();
  if (result != 0) {
    Serial.println("Gnss begin error!!");
    error_flag = 1;
  } else {
    switch (satType) {
      case eSatGps:
        Gnss.select(GPS);
        break;
      case eSatGlonass:
        Gnss.select(GLONASS);
        break;
      case eSatGpsSbas:
        Gnss.select(GPS);
        Gnss.select(SBAS);
        break;
      case eSatGpsGlonass:
        Gnss.select(GPS);
        Gnss.select(GLONASS);
        break;
      case eSatGpsBeidou:
        Gnss.select(GPS);
        Gnss.select(BEIDOU);
        break;
      case eSatGpsGalileo:
        Gnss.select(GPS);
        Gnss.select(GALILEO);
        break;
      case eSatGpsQz1c:
        Gnss.select(GPS);
        Gnss.select(QZ_L1CA);
        break;
      case eSatGpsQz1cQz1S:
        Gnss.select(GPS);
        Gnss.select(QZ_L1CA);
        Gnss.select(QZ_L1S);
        break;
      case eSatGpsBeidouQz1c:
        Gnss.select(GPS);
        Gnss.select(BEIDOU);
        Gnss.select(QZ_L1CA);
        break;
      case eSatGpsGalileoQz1c:
        Gnss.select(GPS);
        Gnss.select(GALILEO);
        Gnss.select(QZ_L1CA);
        break;
      case eSatGpsGlonassQz1c:
      default:
        Gnss.select(GPS);
        Gnss.select(GLONASS);
        Gnss.select(QZ_L1CA);
        break;
    }

    result = Gnss.start(COLD_START);
    if (result != 0) {
      Serial.println("Gnss start error!!");
      error_flag = 1;
    } else {
      Serial.println("Gnss setup OK");
    }
  }

  if (error_flag == 1) {
    Led_isError(true);
    while(1);
  }
}

void initBME280() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    errorLoop(3);
  }
}

void initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    errorLoop(4);
  }
}

void captureImage() {
  char filename[32];
  snprintf(filename, sizeof(filename), "IMG_%04d.jpg", imageCounter);

  CamImage img = theCamera.takePicture();
  if (img.isAvailable()) {
    File myFile = SD.open(filename, FILE_WRITE);
    if (!myFile) {
      Serial.println("Failed to write image file");
      return;
    }
    myFile.write(img.getImgBuff(), img.getImgSize());
    myFile.close();
    imageCounter++;
  } else {
    Serial.println("Failed to capture image");
  }
}

void recordSensorData(SpNavData &NavData) {
  struct sensorDataMsg data;

  data.temperature = bme.readTemperature();
  data.humidity = bme.readHumidity();
  data.pressure = bme.readPressure() / 100.0F;
  data.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  data.accelX = ax / 16384.0;
  data.accelY = ay / 16384.0;
  data.accelZ = az / 16384.0;
  data.gyroX = gx / 131.0;
  data.gyroY = gy / 131.0;
  data.gyroZ = gz / 131.0;

  data.latitude = NavData.latitude;
  data.longitude = NavData.longitude;
  data.gpsAltitude = NavData.altitude;
  data.fix = NavData.posFixMode;
  data.satellites = NavData.numSatellites;

  snprintf(data.imageFilename, sizeof(data.imageFilename), "IMG_%04d.jpg", imageCounter - 1);

  printSensorData(data);
  writeToCSV(data);
}

void printSensorData(const sensorDataMsg& data) {
  Serial.print("Temperature: ");
  Serial.print(data.temperature);
  Serial.print(" C, ");
  Serial.print("Humidity: ");
  Serial.print(data.humidity);
  Serial.print(" %, ");
  Serial.print("Pressure: ");
  Serial.print(data.pressure);
  Serial.print(" hPa, ");
  Serial.print("Estimated Altitude: ");
  Serial.print(data.altitude);
  Serial.print(" m, ");
  Serial.print("Lat: ");
  Serial.print(data.latitude, 6);
  Serial.print(", ");
  Serial.print("Lng: ");
  Serial.print(data.longitude, 6);
  Serial.print(", ");
  Serial.print("GPS Alt: ");
  Serial.print(data.gpsAltitude);
  Serial.print(" m, ");
  Serial.print("Fix: ");
  Serial.print(data.fix);
  Serial.print(", ");
  Serial.print("Satellites: ");
  Serial.print(data.satellites);
  Serial.print(", ");
  Serial.print("Image: ");
  Serial.print(data.imageFilename);
  Serial.print(", ");
  Serial.print("Accel X/Y/Z: ");
  Serial.print(data.accelX);
  Serial.print("/");
  Serial.print(data.accelY);
  Serial.print("/");
  Serial.print(data.accelZ);
  Serial.print(" g, ");
  Serial.print("Gyro X/Y/Z: ");
  Serial.print(data.gyroX);
  Serial.print("/");
  Serial.print(data.gyroY);
  Serial.print("/");
  Serial.print(data.gyroZ);
  Serial.println(" deg/s");
}

void initCSV() {
  SD.remove(CSV_FILENAME);
  File dataFile = SD.open(CSV_FILENAME, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time(s),Temp(C),Humidity(%),Pressure(hPa),EstAlt(m),Lat,Lng,GPSAlt(m),Fix,Sats,ImageFilename,AccelX(g),AccelY(g),AccelZ(g),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s)");
    dataFile.close();
  } else {
    Serial.println("Error: Could not open CSV file");
  }
}

void writeToCSV(const sensorDataMsg& data) {
  File dataFile = SD.open(CSV_FILENAME, FILE_WRITE);
  if (dataFile) {
    char buffer[512];
    unsigned long currentTime = millis();
    float elapsedSeconds = (currentTime - startTime) / 1000.0;

    int len = snprintf(buffer, sizeof(buffer), "%.3f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                       elapsedSeconds, data.temperature, data.humidity, data.pressure, data.altitude,
                       data.latitude, data.longitude, data.gpsAltitude, data.fix, data.satellites, data.imageFilename,
                       data.accelX, data.accelY, data.accelZ, data.gyroX, data.gyroY, data.gyroZ);

    if (len >= 0 && len < sizeof(buffer)) {
      dataFile.println(buffer);
    } else {
      Serial.println("Error: Failed to create string with snprintf");
    }
    dataFile.close();
  } else {
    Serial.println("Error: Could not open CSV file");
  }
}

void event(String event_msg) {
  unsigned long currentTime = millis();
  float elapsedSeconds = (currentTime - startTime) / 1000.0;

  String logMessage = String(elapsedSeconds, 3) + ": " + event_msg;
  Serial.println(logMessage);

  File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
  if (logFile) {
    logFile.println(logMessage);
    logFile.close();
  } else {
    Serial.println("Error: Could not open log file");
  }
}

void printError(enum CamErr err) {
  Serial.print("Camera error: ");
  switch (err) {
    case CAM_ERR_NO_DEVICE: Serial.println("No device"); break;
    case CAM_ERR_ILLEGAL_DEVERR: Serial.println("Illegal device error"); break;
    case CAM_ERR_ALREADY_INITIALIZED: Serial.println("Already initialized"); break;
    case CAM_ERR_NOT_INITIALIZED: Serial.println("Not initialized"); break;
    case CAM_ERR_NOT_STILL_INITIALIZED: Serial.println("Still image not initialized"); break;
    case CAM_ERR_CANT_CREATE_THREAD: Serial.println("Failed to create thread"); break;
    case CAM_ERR_INVALID_PARAM: Serial.println("Invalid parameter"); break;
    case CAM_ERR_NO_MEMORY: Serial.println("Out of memory"); break;
    case CAM_ERR_USR_INUSED: Serial.println("Buffer already in use"); break;
    case CAM_ERR_NOT_PERMITTED: Serial.println("Operation not permitted"); break;
    default: Serial.println("Unknown camera error"); break;
  }
}

void errorLoop(int num) {
  while (1) {
    for (int i = 0; i < num; i++) {
      digitalWrite(LED3, HIGH);
      delay(300);
      digitalWrite(LED3, LOW);
      delay(300);
    }
    delay(1000);
  }
}

static void Led_isActive() {
  static int state = 1;
  if (state == 1) {
    ledOn(PIN_LED0);
    state = 0;
  } else {
    ledOff(PIN_LED0);
    state = 1;
  }
}

static void Led_isPosfix(bool state) {
  if (state) {
    ledOn(PIN_LED1);
  } else {
    ledOff(PIN_LED1);
  }
}

static void Led_isError(bool state) {
  if (state) {
    ledOn(PIN_LED3);
  } else {
    ledOff(PIN_LED3);
  }
}

static void print_pos(SpNavData *pNavData) {
  char StringBuffer[STRING_BUFFER_SIZE];

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d ", pNavData->time.year, pNavData->time.month, pNavData->time.day);
  Serial.print(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06ld, ", pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
  Serial.print(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->numSatellites);
  Serial.print(StringBuffer);

  if (pNavData->posFixMode == FixInvalid) {
    Serial.print("No-Fix, ");
  } else {
    Serial.print("Fix, ");
  }
  if (pNavData->posDataExist == 0) {
    Serial.print("No Position");
  } else {
    Serial.print("Lat=");
    Serial.print(pNavData->latitude, 6);
    Serial.print(", Lon=");
    Serial.print(pNavData->longitude, 6);
  }

  Serial.println("");
}

static void print_condition(SpNavData *pNavData) {
  char StringBuffer[STRING_BUFFER_SIZE];
  unsigned long cnt;

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSatellites:%2d\n", pNavData->numSatellites);
  Serial.print(StringBuffer);

  for (cnt = 0; cnt < pNavData->numSatellites; cnt++) {
    const char *pType = "---";
    SpSatelliteType sattype = pNavData->getSatelliteType(cnt);

    switch (sattype) {
      case GPS:
        pType = "GPS";
        break;
      case GLONASS:
        pType = "GLN";
        break;
      case QZ_L1CA:
        pType = "QCA";
        break;
      case SBAS:
        pType = "SBA";
        break;
      case QZ_L1S:
        pType = "Q1S";
        break;
      case BEIDOU:
        pType = "BDS";
        break;
      case GALILEO:
        pType = "GAL";
        break;
      default:
        pType = "UKN";
        break;
    }

    unsigned long Id  = pNavData->getSatelliteId(cnt);
    unsigned long Elv = pNavData->getSatelliteElevation(cnt);
    unsigned long Azm = pNavData->getSatelliteAzimuth(cnt);
    float sigLevel = pNavData->getSatelliteSignalLevel(cnt);

    snprintf(StringBuffer, STRING_BUFFER_SIZE, "[%2ld] Type:%s, Id:%2ld, Elv:%2ld, Azm:%3ld, CN0:", cnt, pType, Id, Elv, Azm );
    Serial.print(StringBuffer);
    Serial.println(sigLevel, 6);
  }
}

void restartGNSS() {
  int error_flag = 0;

  ledOff(PIN_LED0);
  Led_isPosfix(false);

  if (Gnss.stop() != 0 || Gnss.end() != 0) {
    Serial.println("Gnss stop/end error!!");
    error_flag = 1;
  } else {
    Serial.println("Gnss stop OK.");
  }

  if (Gnss.begin() != 0 || Gnss.start(HOT_START) != 0) {
    Serial.println("Gnss begin/start error!!");
    error_flag = 1;
  } else {
    Serial.println("Gnss restart OK.");
  }

  if (error_flag == 1) {
    Led_isError(true);
    while(1);
  }
}