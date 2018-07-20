#include "WStream.h"
#define Stream WStream

#include <Wire.h>
#include <BLEPeripheral.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_L3GD20_U.h"
#include "Adafruit_LSM303_U.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"

BLEPeripheral blep;
BLEService bless = BLEService("CCC0");
BLECharacteristic all_char("CCC1", BLERead | BLENotify, 16);

#define LED 13
int ledState = LOW;
unsigned long prev = 0;
unsigned long prev2 = 0;
unsigned long count = 0;

Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);

float mag_offsets[3]            = { -1.02F,  0.78F, -4.39F };

float mag_softiron_matrix[3][3] = { { 0.965, 0.007, 0.016 },
                                    { 0.007, 0.971, 0.000 },
                                    { 0.016, 0.000, 1.067 } }; 

float mag_field_strength        = 39.58F;

//Mahony filter;
Madgwick filter;

float heading, pitch, roll;
#define MEM_LEN 16
char databuf[MEM_LEN];

void setup()
{
  String def = "2222,2222,2222\0\0";
  def.toCharArray(databuf, 15);
  Serial.begin(9600);
  Serial.println(F("Adafruit AHRS Fusion Example"));
  Serial.println("");
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  blep.setLocalName("POP");
  blep.setDeviceName("POP");
  blep.setAdvertisedServiceUuid(bless.uuid());
  blep.addAttribute(bless);
  blep.addAttribute(all_char);
  blep.begin();

  if(!gyro.begin())
  {
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  
  if(!accel.begin())
  {
    Serial.println("Ooops, no L3M303DLHC accel detected ... Check your wiring!");
    while(1);
  }
  
  if(!mag.begin())
  {
    Serial.println("Ooops, no L3M303DLHC mag detected ... Check your wiring!");
    while(1);
  }

  filter.begin(50);
}

void loop(void)
{
  if (millis() - prev > 1000) {
    prev = millis();
    ledState = ~ledState;
    digitalWrite(LED, ledState);
  }
  
  blep.poll();
  
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  float gx = gyro_event.gyro.x * 57.2958F;
  float gy = gyro_event.gyro.y * 57.2958F;
  float gz = gyro_event.gyro.z * 57.2958F;

  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(heading);

  if (millis() - prev2 > 10) {
    prev2 = millis();
    if (count++ % 2 == 0) {
      Wire.requestFrom(0x66, 16);
      int rec = 0;
      while (Wire.available()) {
        databuf[rec++] = Wire.read();
      }
      char str[16];
      str[0] = 'p';
      str[15] = '\0';
      for (int i = 1 ; i < 15 ; i++) {
        str[i] = databuf[i-1];
      }
      all_char.setValue(str);
    }
    else {
      char str[16];
      sprintf(str, "o%+03d,%+03d,%+03d\0", (int)roll + 360, (int)pitch + 360, (int)heading);
      all_char.setValue(str);
    }
  }
  delay(10);
}
  
