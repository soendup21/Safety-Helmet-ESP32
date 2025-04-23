/// final one
#include <Wire.h>
#include <TridentTD_LineNotify.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define SSID "<your_wifi_ssid>"
#define PASSWORD "<your_wifi_password>"
#define LINE_TOKEN "<your_line_notify_token>"
#define PirPin 16
#define rxPin 39
#define txPin 36
#define BUZZER 15
#define BUTTON 27
#define detectorPin 13
#define LEDY 14
#define LEDG 12

char latitude[32], longitude[32];
TinyGPSPlus gps;

int GPSBaud = 9600;
SoftwareSerial gpsSerial(rxPin, txPin);

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
int prev_Amp = 0;
int Amp = 0;
int prev_angleChange = 0;
int angleChange = 0;
unsigned long prev_time = 0;
unsigned long cur_time = 0;
int slope_amp = 0;
int slope_angleChange = 0;
bool Accident = 0;
int val;  // variable to store result
bool trig = 1;
int beat, bpm, countStatus;
unsigned long millisBefore;


String alertMSG = "Crash Detected!!! Check the Location here: https://www.google.com/maps/search/?api=1&query=";

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  pinMode(BUZZER, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDY, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(detectorPin, INPUT_PULLUP);
  Wire.endTransmission(true);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  LINE.notify("Wifi Connected.");
  digitalWrite(LEDG, 1);
  delay(3000);
  digitalWrite(LEDG, 0);
  LINE.setToken(LINE_TOKEN);
  digitalWrite(BUZZER, 0);
  val = digitalRead(detectorPin);
  if (val == LOW) {
    LINE.notify("Helmet worn");
    Serial.println("Helmet worn");
  }
}

void loop() {
  mpu_read();
  getLOC();
  Serial.print("Latitude= ");
  Serial.print(latitude);
  Serial.print(" Longitude= ");
  Serial.println(longitude);
  val = digitalRead(detectorPin);
  if (val == LOW) {
    while (val == 0) {
      mpu_read();
      if (slope_amp > 40 || slope_angleChange > 83) {
        Serial.println("Crashed detected");
        Accident = 1;
        break;
      }
      val = digitalRead(detectorPin);
    }
    unsigned long start = millis();
    while (Accident == 1) {
      trig = digitalRead(BUTTON);
      // digitalWrite(BUZZER, 1);
      digitalWrite(LEDY, 1);
      getLOC();
      if (millis() - start > 5000) {
        Serial.print("Latitude= ");
        Serial.print(13.8212516);
        Serial.print(" Longitude= ");
        Serial.println(100.5135891);
        alertMSG = alertMSG + latitude;
        alertMSG = alertMSG + ",";
        alertMSG = alertMSG + longitude;
        LINE.notify(alertMSG);
        Serial.println("Notification Sent\n");
        digitalWrite(BUZZER, 0);
        digitalWrite(LEDY, 0);
        getBPM();
        Accident = 0;
      }
      if (trig == 0) {
        Serial.println("Notification Aborted\n");
        digitalWrite(BUZZER, 0);
        digitalWrite(LEDY, 0);
        Accident = 0;
      }
    }
  } else if (val == 1) {
    LINE.notify("Helmet removed");
    Serial.println("Helmet removed\n");
    while (val == 1) {
      val = digitalRead(detectorPin);
    }
    LINE.notify("Helmet worn");
    Serial.println("Helmet worn");
  }
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;
  prev_time = cur_time;
  cur_time = millis();
  prev_Amp = Amp;
  prev_angleChange = angleChange;
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
  angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);

  slope_amp = abs(round(atan2(float(Amp - prev_Amp), float(cur_time - prev_time)) * 180 / 3.14159265));
  slope_angleChange = abs(round(atan2(float(angleChange - prev_angleChange), float(cur_time - prev_time)) * 180 / 3.14159265));
  delay(20);
}

void getLOC() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      sprintf(latitude, "%f", gps.location.lat());
      sprintf(longitude, "%f", gps.location.lng());
    }
  }
}
void getBPM() {
  int sensorValue = analogRead(A0);
  if (countStatus == 0) {
    if (sensorValue > 516) {
      countStatus = 1;
      beat++;
    }
  } else {
    if (sensorValue < 510) {
      countStatus = 0;
    }
  }
  if (millis() - millisBefore > 15000) {
    bpm = beat * 30;
    beat = 0;
    Serial.print("BPM : ");
    Serial.println(bpm);
    LINE.notify("BPM: ");
    LINE.notify(bpm);
    millisBefore = millis();
  }
  delay(10);
  getBPM();
}
