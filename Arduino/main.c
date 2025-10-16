
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#include <Adafruit_BMP085.h>  
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"  


static const uint8_t GPS_RX = 4;  
static const uint8_t GPS_TX = 3;  
SoftwareSerial gpsSer(GPS_RX, GPS_TX);
TinyGPSPlus gps;
Adafruit_BMP085 bmp;
bool bmp_ok = false;
const double SEA_LEVEL_PA = 100900.0;

MPU6050 mpu(0x68);
const int INTERRUPT_PIN = 2;


bool dmpReady = false;
volatile bool mpuInterrupt = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
unsigned long lastPrint = 0;
const unsigned long PRINT_MS = 500; 

void dmpDataReady() { mpuInterrupt = true; }

void setup() {

  Serial.begin(115200);
  delay(200);
  Serial.println(F("BOOT: UNO + MPU6050(DMP) + BMP180 + NEO-6M"));

  Wire.begin();
  Wire.setClock(400000); 

  
  bmp_ok = bmp.begin();
  if (bmp_ok) Serial.println(F("BMP180 OK"));
  else        Serial.println(F("BMP180 NOT FOUND"));
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("MPU6050: Initializing I2C devices..."));
  mpu.initialize();

  Serial.print(F("MPU6050 connection... "));
  Serial.println(mpu.testConnection() ? F("OK") : F("FAILED"));

  Serial.println(F("MPU6050: Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println(F("Active offsets:"));
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP ready, waiting for data..."));
  } else {
    Serial.print(F("DMP init failed, code ")); Serial.println(devStatus);
  }


  gpsSer.begin(9600);  
  Serial.println(F("GPS started @9600"));

 
  Serial.println(F("time_ms,lat,lon,gps_alt_m,fix,sats,hdop,"
                   "bmp_temp_c,bmp_pres_hpa,bmp_alt_m,"
                   "yaw_deg,pitch_deg,roll_deg"));
}


void loop() {

  while (gpsSer.available()) {
    gps.encode(gpsSer.read());
  }

  
  static float yaw_d = NAN, pitch_d = NAN, roll_d = NAN;
  if (dmpReady) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yaw_d   = ypr[0] * 180.0 / M_PI;
      pitch_d = ypr[1] * 180.0 / M_PI;
      roll_d  = ypr[2] * 180.0 / M_PI;
    }
  }

 
  double bmp_temp = NAN, bmp_pres_hpa = NAN, bmp_alt_m = NAN;
  if (bmp_ok) {
    bmp_temp     = bmp.readTemperature();
    bmp_pres_hpa = bmp.readPressure() / 100.0;
    bmp_alt_m    = bmp.readAltitude(SEA_LEVEL_PA);
  }

 
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;

    Serial.print(now); Serial.print(',');

    if (gps.location.isValid()) {
      Serial.print(gps.location.lat(), 6); Serial.print(',');
      Serial.print(gps.location.lng(), 6); Serial.print(',');
    } else {
      Serial.print("NA,NA,");
    }
    if (gps.altitude.isValid()) Serial.print(gps.altitude.meters(), 2);
    else                        Serial.print("NA");
    Serial.print(',');

    Serial.print(gps.location.isValid() ? 1 : 0); Serial.print(',');
    Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0); Serial.print(',');
    if (gps.hdop.isValid()) Serial.print(gps.hdop.hdop(), 1);
    else                    Serial.print("NA");
    Serial.print(',');
    if (!isnan(bmp_temp))     Serial.print(bmp_temp, 2); else Serial.print("NA"); Serial.print(',');
    if (!isnan(bmp_pres_hpa)) Serial.print(bmp_pres_hpa, 2); else Serial.print("NA"); Serial.print(',');
    if (!isnan(bmp_alt_m))    Serial.print(bmp_alt_m, 2); else Serial.print("NA"); Serial.print(',');
    if (!isnan(yaw_d))   Serial.print(yaw_d, 2);   else Serial.print("NA"); Serial.print(',');
    if (!isnan(pitch_d)) Serial.print(pitch_d, 2); else Serial.print("NA"); Serial.print(',');
    if (!isnan(roll_d))  Serial.print(roll_d, 2);  else Serial.print("NA");
    Serial.println();
    if (gps.location.isValid()) {
      Serial.print(F("GMAPS: "));
      Serial.print(gps.location.lat(), 6);
      Serial.print(',');
      Serial.println(gps.location.lng(), 6);
    }
if (gps.location.isValid()) {
  Serial.print("{\"sensor\":\"gps\",\"lat\":");
  Serial.print(gps.location.lat(), 6);
  Serial.print(",\"lon\":");
  Serial.print(gps.location.lng(), 6);
  Serial.print(",\"alt\":");
  Serial.print(gps.altitude.isValid() ? gps.altitude.meters() : 0, 2);
  Serial.print(",\"fix\":\"3D\",\"sats\":");
  Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
  Serial.print(",\"hdop\":");
  if (gps.hdop.isValid()) Serial.print(gps.hdop.hdop(), 1); else Serial.print(0);
  Serial.println(",\"time\":\"UTC\"}");
}
Serial.print("{\"sensor\":\"baro\",\"pressure_hpa\":");
Serial.print(!isnan(bmp_pres_hpa) ? bmp_pres_hpa : 0, 2);
Serial.print(",\"alt_m\":");
Serial.print(!isnan(bmp_alt_m) ? bmp_alt_m : 0, 2);
Serial.print(",\"temp_c\":");
Serial.print(!isnan(bmp_temp) ? bmp_temp : 0, 2);
Serial.println("}");

Serial.print("{\"sensor\":\"temp\",\"temp_c\":");
Serial.print(!isnan(bmp_temp) ? bmp_temp : 0, 2);
Serial.println(",\"humidity\":0,\"status\":\"OK\"}");
  }
}


    

