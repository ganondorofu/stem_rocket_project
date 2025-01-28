#include <Arduino.h>
#include <SDHCI.h>
#include <File.h>
#include <GNSS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>

#define CSV_FILENAME "sensor_data.csv"
#define LOG_FILENAME "event_log.txt"
#define STRING_BUFFER_SIZE 128
#define RESTART_CYCLE (60 * 5)

SDClass SD;
File myFile;
SpGnss Gnss;
Adafruit_BME280 bme;
MPU6050 mpu;
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
  eSatGpsQz1cQz1S
};
static enum ParamSat satType = eSatGpsGlonass;

struct sensorDataMsg {
  float timeFromPowerOn;
  float temperature;
  float humidity;
  float pressure;
  float latitude;
  float longitude;
  float gpsAltitude;
  int fix;
  int satellites;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

static int LoopCount = 0;
static int LastPrintMin = 0;

void initGNSS();
void initBME280();
void initMPU6050();
void initCSV();
void recordSensorData(SpNavData &NavData);
void printSensorData(const sensorDataMsg &data);
void writeToCSV(const sensorDataMsg &data);
void event(String event_msg);
void errorLoop(int num);
static void Led_isActive();
static void Led_isPosfix(bool state);
static void Led_isError(bool state);
static void print_pos(SpNavData *pNavData);
static void print_condition(SpNavData *pNavData);
void restartGNSS();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  startTime = millis();
  Wire.begin();
  initGNSS();
  initBME280();
  initMPU6050();
  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed.");
    while (1);
  }
  initCSV();
  event("All initialization completed");
}

void loop() {
  Led_isActive();
  if (Gnss.waitUpdate(-1)) {
    SpNavData NavData;
    Gnss.getNavData(&NavData);
    bool ledFix = (NavData.posDataExist && (NavData.posFixMode != FixInvalid));
    Led_isPosfix(ledFix);
    if (NavData.time.minute != LastPrintMin) {
      print_condition(&NavData);
      LastPrintMin = NavData.time.minute;
    }
    print_pos(&NavData);
    recordSensorData(NavData);
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

void initGNSS() {
  int err = 0;
  Gnss.setDebugMode(PrintInfo);
  if (Gnss.begin() != 0) {
    Serial.println("Gnss begin error!!");
    err = 1;
  } else {
    switch (satType) {
      case eSatGps: Gnss.select(GPS); break;
      case eSatGlonass: Gnss.select(GLONASS); break;
      case eSatGpsSbas: Gnss.select(GPS); Gnss.select(SBAS); break;
      case eSatGpsGlonass: Gnss.select(GPS); Gnss.select(GLONASS); break;
      case eSatGpsBeidou: Gnss.select(GPS); Gnss.select(BEIDOU); break;
      case eSatGpsGalileo: Gnss.select(GPS); Gnss.select(GALILEO); break;
      case eSatGpsQz1c: Gnss.select(GPS); Gnss.select(QZ_L1CA); break;
      case eSatGpsQz1cQz1S: Gnss.select(GPS); Gnss.select(QZ_L1CA); Gnss.select(QZ_L1S); break;
      case eSatGpsBeidouQz1c: Gnss.select(GPS); Gnss.select(BEIDOU); Gnss.select(QZ_L1CA); break;
      case eSatGpsGalileoQz1c: Gnss.select(GPS); Gnss.select(GALILEO); Gnss.select(QZ_L1CA); break;
      default:
        Gnss.select(GPS);
        Gnss.select(GLONASS);
        Gnss.select(QZ_L1CA);
        break;
    }
    if (Gnss.start(COLD_START) != 0) {
      Serial.println("Gnss start error!!");
      err = 1;
    } else {
      Serial.println("Gnss setup OK");
    }
  }
  if (err == 1) {
    Led_isError(true);
    while (1);
  }
}

void initBME280() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
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

void initCSV() {
  SD.remove(CSV_FILENAME);
  File dataFile = SD.open(CSV_FILENAME, FILE_WRITE);
  if (dataFile) {
    dataFile.println("TimeFromPowerOn(s),Temp(C),Humidity(%),Pressure(hPa),Lat,Lng,GPSAlt(m),Fix,Sats,AccelX(g),AccelY(g),AccelZ(g),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s)");
    dataFile.close();
  } else {
    Serial.println("Error: Could not open CSV file");
  }
}

void recordSensorData(SpNavData &NavData) {
  sensorDataMsg d;
  d.timeFromPowerOn = (millis() - startTime) / 1000.0;
  d.temperature = bme.readTemperature();
  d.humidity = bme.readHumidity();
  d.pressure = bme.readPressure() / 100.0F;
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  d.accelX = ax / 16384.0;
  d.accelY = ay / 16384.0;
  d.accelZ = az / 16384.0;
  d.gyroX = gx / 131.0;
  d.gyroY = gy / 131.0;
  d.gyroZ = gz / 131.0;
  d.latitude = NavData.latitude;
  d.longitude = NavData.longitude;
  d.gpsAltitude = NavData.altitude;
  d.fix = NavData.posFixMode;
  d.satellites = NavData.numSatellites;
  printSensorData(d);
  writeToCSV(d);
}

void printSensorData(const sensorDataMsg &d) {
  Serial.print("TimeFromPowerOn: ");
  Serial.print(d.timeFromPowerOn, 3);
  Serial.print(" s, Temp: ");
  Serial.print(d.temperature);
  Serial.print(" C, Humi: ");
  Serial.print(d.humidity);
  Serial.print(" %, Press: ");
  Serial.print(d.pressure);
  Serial.print(" hPa, Lat: ");
  Serial.print(d.latitude, 6);
  Serial.print(", Lng: ");
  Serial.print(d.longitude, 6);
  Serial.print(", GPS Alt: ");
  Serial.print(d.gpsAltitude);
  Serial.print(" m, Fix: ");
  Serial.print(d.fix);
  Serial.print(", Sats: ");
  Serial.print(d.satellites);
  Serial.print(", Accel: ");
  Serial.print(d.accelX, 4);
  Serial.print("/");
  Serial.print(d.accelY, 4);
  Serial.print("/");
  Serial.print(d.accelZ, 4);
  Serial.print(" g, Gyro: ");
  Serial.print(d.gyroX, 4);
  Serial.print("/");
  Serial.print(d.gyroY, 4);
  Serial.print("/");
  Serial.print(d.gyroZ, 4);
  Serial.println(" deg/s");
}

void writeToCSV(const sensorDataMsg &d) {
  File f = SD.open(CSV_FILENAME, FILE_WRITE);
  if (f) {
    char buf[256];
    int n = snprintf(
      buf,
      sizeof(buf),
      "%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
      d.timeFromPowerOn,
      d.temperature, d.humidity, d.pressure,
      d.latitude, d.longitude, d.gpsAltitude,
      d.fix, d.satellites,
      d.accelX, d.accelY, d.accelZ,
      d.gyroX, d.gyroY, d.gyroZ
    );
    if (n > 0 && n < (int)sizeof(buf)) f.println(buf);
    else Serial.println("Error: CSV snprintf failed");
    f.close();
  } else {
    Serial.println("Error: Could not open CSV file");
  }
}

void event(String event_msg) {
  float t = (millis() - startTime) / 1000.0;
  String s = String(t, 3) + ": " + event_msg;
  Serial.println(s);
  File lf = SD.open(LOG_FILENAME, FILE_WRITE);
  if (lf) {
    lf.println(s);
    lf.close();
  } else {
    Serial.println("Error: Could not open log file");
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
  static bool st = false;
  if (st) {
    ledOff(PIN_LED0);
    st = false;
  } else {
    ledOn(PIN_LED0);
    st = true;
  }
}

static void Led_isPosfix(bool st) {
  if (st) ledOn(PIN_LED1);
  else    ledOff(PIN_LED1);
}

static void Led_isError(bool st) {
  if (st) ledOn(PIN_LED3);
  else    ledOff(PIN_LED3);
}

static void print_pos(SpNavData *p) {
  char buf[STRING_BUFFER_SIZE];
  snprintf(buf, STRING_BUFFER_SIZE, "%04d/%02d/%02d ", p->time.year, p->time.month, p->time.day);
  Serial.print(buf);
  snprintf(buf, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06ld, ",
           p->time.hour, p->time.minute, p->time.sec, p->time.usec);
  Serial.print(buf);
  snprintf(buf, STRING_BUFFER_SIZE, "numSat:%2d, ", p->numSatellites);
  Serial.print(buf);
  if (p->posFixMode == FixInvalid) Serial.print("No-Fix, ");
  else Serial.print("Fix, ");
  if (p->posDataExist == 0) Serial.print("No Position");
  else {
    Serial.print("Lat=");
    Serial.print(p->latitude, 6);
    Serial.print(", Lon=");
    Serial.print(p->longitude, 6);
  }
  Serial.println("");
}

static void print_condition(SpNavData *p) {
  char buf[STRING_BUFFER_SIZE];
  snprintf(buf, STRING_BUFFER_SIZE, "numSatellites:%2d\n", p->numSatellites);
  Serial.print(buf);
  for (unsigned long i = 0; i < p->numSatellites; i++) {
    const char *typeStr = "---";
    SpSatelliteType st = p->getSatelliteType(i);
    switch (st) {
      case GPS:       typeStr = "GPS"; break;
      case GLONASS:   typeStr = "GLN"; break;
      case QZ_L1CA:   typeStr = "QCA"; break;
      case SBAS:      typeStr = "SBA"; break;
      case QZ_L1S:    typeStr = "Q1S"; break;
      case BEIDOU:    typeStr = "BDS"; break;
      case GALILEO:   typeStr = "GAL"; break;
      default:        typeStr = "UKN"; break;
    }
    unsigned long Id  = p->getSatelliteId(i);
    unsigned long Elv = p->getSatelliteElevation(i);
    unsigned long Azm = p->getSatelliteAzimuth(i);
    float sig         = p->getSatelliteSignalLevel(i);
    snprintf(buf, STRING_BUFFER_SIZE, "[%2ld] Type:%s, Id:%2ld, Elv:%2ld, Azm:%3ld, CN0:", i, typeStr, Id, Elv, Azm);
    Serial.print(buf);
    Serial.println(sig, 6);
  }
}

void restartGNSS() {
  int err = 0;
  ledOff(PIN_LED0);
  Led_isPosfix(false);
  if (Gnss.stop() != 0 || Gnss.end() != 0) {
    Serial.println("Gnss stop/end error!!");
    err = 1;
  } else {
    Serial.println("Gnss stop OK.");
  }
  if (Gnss.begin() != 0 || Gnss.start(HOT_START) != 0) {
    Serial.println("Gnss begin/start error!!");
    err = 1;
  } else {
    Serial.println("Gnss restart OK.");
  }
  if (err == 1) {
    Led_isError(true);
    while(1);
  }
}
