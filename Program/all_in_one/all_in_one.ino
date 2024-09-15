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

SDClass SD;
File myFile;
SpNavData navData;
SpGnss gnss;
Adafruit_BME280 bme;
MPU6050 mpu;

int imageCounter = 0;
unsigned long startTime;
bool isLifted = false;

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
  while (!Serial)
    ;

  startTime = millis();

  Wire.begin();
  initCamera();
  initGNSS();
  initBME280();
  initMPU6050();

  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed. Please check the SD card.");
    while (1)
      ;
  }

  initCSV();

  event("All initialization completed");
}

void loop() {
  satType = eSatGpsBeidou;
  recordSensorData();
  checkLifted();
  captureImage();
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
  int result = gnss.begin();
  if (result != 0) {
    Serial.println("GNSS start error");
    errorLoop(2);
  }

  // 衛星システムの選択
  switch (satType) {
    case eSatGps:
      gnss.select(GPS);
      break;
    case eSatGpsSbas:
      gnss.select(GPS);
      gnss.select(SBAS);
      break;
    case eSatGlonass:
      gnss.select(GLONASS);
      gnss.deselect(GPS);
      break;
    case eSatGpsGlonass:
      gnss.select(GPS);
      gnss.select(GLONASS);
      break;
    case eSatGpsBeidou:
      gnss.select(GPS);
      gnss.select(BEIDOU);
      break;
    case eSatGpsGalileo:
      gnss.select(GPS);
      gnss.select(GALILEO);
      break;
    case eSatGpsQz1c:
      gnss.select(GPS);
      gnss.select(QZ_L1CA);
      break;
    case eSatGpsQz1cQz1S:
      gnss.select(GPS);
      gnss.select(QZ_L1CA);
      gnss.select(QZ_L1S);
      break;
    case eSatGpsBeidouQz1c:
      gnss.select(GPS);
      gnss.select(BEIDOU);
      gnss.select(QZ_L1CA);
      break;
    case eSatGpsGalileoQz1c:
      gnss.select(GPS);
      gnss.select(GALILEO);
      gnss.select(QZ_L1CA);
      break;
    case eSatGpsGlonassQz1c:
    default:
      gnss.select(GPS);
      gnss.select(GLONASS);
      gnss.select(QZ_L1CA);
      break;
  }

  result = gnss.start(COLD_START);
  if (result != 0) {
    Serial.println("GNSS initialization error");
    errorLoop(2);
  }

  Serial.println("GNSS initialized successfully");
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

void recordSensorData() {
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

  gnss.getNavData(&navData);
  data.latitude = navData.latitude;
  data.longitude = navData.longitude;
  data.gpsAltitude = navData.altitude;
  data.fix = navData.posFixMode;
  data.satellites = navData.numSatellites;

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

void checkLifted() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float accelZ = az / 16384.0;

  if (!isLifted && accelZ < 0.8) {
    isLifted = true;
    event("Device lifted");
  } else if (isLifted && accelZ > 0.9) {
    isLifted = false;
    event("Device returned to rest");
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