
/*/   Mobile Robotic Arm Teleoperation Controller   /*/
// Code by Michael Fernandez, Nov 2024

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xCC, 0x50, 0xE3, 0xF4, 0x64, 0xC2};
// ESP NOW data transfer
typedef struct struct_message {
  uint16_t joyXdata;
  uint16_t joyYdata;
  bool gyroCtrlData;
  float gyroZdata;
  float gyroYdata;
  bool clawBtn1Data;
  bool clawBtn2Data;
} struct_message;
struct_message transferData;

//IMU sensor
Adafruit_MPU6050 mpu;
unsigned long previousTime = 0;
const unsigned int sensitivityFactor = 100;

// Servo
float accumulatedZAngle = 90;
float accumulatedYAngle = 90;
uint8_t clawServoAngle = 90;
float ZangleChange = 0;
float YangleChange = 0;

// Claw button
const uint8_t clawBtn1 = 0; // D3
const uint8_t clawBtn2 = 2; // D4

// Joystick
const uint8_t joystickBtn = 14; // D5
const uint8_t joyYreadCtrl = 12; // D6
const uint8_t joyXreadCtrl = 13; // D7
bool gyroCtrl = false;

// Battery check
bool battCheck = false;
const uint8_t battCtrl = 16; // D0
const float voltageDividerFactor = 2.0;
const float maxADCValue = 1024.0;
const float referenceVoltage = 3.2;

const uint8_t analogInput = A0;
const uint8_t warnLed = 15; // D8


// Gyro & claw button logic
void sendGryoValue() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate the time difference
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  float accelXAngle = a.acceleration.x;

  float ZgyroRate = g.gyro.z * sensitivityFactor;
  ZangleChange = ZgyroRate * deltaTime;
  
  float YgyroRate = g.gyro.y * sensitivityFactor;
  YangleChange = YgyroRate * deltaTime;

  accumulatedYAngle += YangleChange;
  accumulatedYAngle = constrain(accumulatedYAngle, 0, 180);

  transferData.gyroYdata = accumulatedYAngle; // used for yaw servo

  if (accelXAngle <= 7 && accelXAngle >= -7) {
    accumulatedZAngle += ZangleChange;
    accumulatedZAngle = constrain(accumulatedZAngle, 0, 180);

    transferData.gyroZdata = accumulatedZAngle; // used for pitch servo
  }
}

// Battery read logic
void checkBattery() {
  digitalWrite(battCtrl, HIGH);
  int battRawValue = analogRead(analogInput);
  digitalWrite(battCtrl, LOW);

  float Vout = (battRawValue / maxADCValue) * referenceVoltage;  // Calculate Vout
  float battVin = Vout * voltageDividerFactor;  // Calculate Vin (battery voltage)

  if (battVin <= 3.3) {
    static uint8_t ledLoopCount = 0;
    static bool warnLedState = false;

    if (ledLoopCount == 50) {
      warnLedState? digitalWrite(warnLed, LOW) : digitalWrite(warnLed, HIGH);
      warnLedState = !warnLedState;
      ledLoopCount = 0;
    }else{
      ledLoopCount++;
    }
  }else if (battVin > 3.4) {
    digitalWrite(warnLed, LOW);
  }
}

// Joystick read logic
void checkJoystick() {
  digitalWrite(joyXreadCtrl, HIGH);
  unsigned int joyXvalue = analogRead(analogInput);
  digitalWrite(joyXreadCtrl, LOW);
  transferData.joyXdata = joyXvalue;

  digitalWrite(joyYreadCtrl, HIGH);
  unsigned int joyYvalue = analogRead(analogInput);
  digitalWrite(joyYreadCtrl, LOW);
  transferData.joyYdata = joyYvalue;
}


void setup(void) {
  Serial.begin(115200);
  Wire.begin();

  // Pin setup
  pinMode(warnLed, OUTPUT);
  pinMode(joystickBtn, INPUT_PULLUP);
  pinMode(joyXreadCtrl, OUTPUT);
  pinMode(joyYreadCtrl, OUTPUT);
  pinMode(clawBtn1, INPUT_PULLUP);
  pinMode(clawBtn2, INPUT_PULLUP);
  pinMode(battCtrl, OUTPUT);

  // ESP NOW setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  
  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Configure the accelerometer and gyroscope
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // You can change to 2G, 4G, or 16G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // Set to 500 degrees/sec range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Record the initial time
  previousTime = millis();
}

void loop() {
  bool clawBtn1State = digitalRead(clawBtn1) == LOW;
  bool clawBtn2State = digitalRead(clawBtn2) == LOW;
  transferData.clawBtn1Data = clawBtn1State;
  transferData.clawBtn2Data = clawBtn2State;

  bool currentJoyState = digitalRead(joystickBtn) == LOW;
  checkJoystick();

  static bool lastJoyBtnState = false;
  if (currentJoyState && !lastJoyBtnState){
    gyroCtrl? gyroCtrl = false : gyroCtrl = true;
    gyroCtrl? digitalWrite(warnLed, HIGH) : digitalWrite(warnLed, LOW);
    transferData.gyroCtrlData = gyroCtrl;
  }
  lastJoyBtnState = currentJoyState;
  
  if (gyroCtrl) {
    sendGryoValue();
  }else {
    checkBattery();
    
    static uint8_t resetLoopCount = 0;
    switch (resetLoopCount) {
      case 0:
        if (clawBtn1State == HIGH && clawBtn2State == HIGH) {
          resetLoopCount++;
        }
        break;
      default:
        if (clawBtn1State == HIGH && clawBtn2State == HIGH && resetLoopCount < 100) {
          resetLoopCount++;
        }else if (clawBtn1State == LOW && clawBtn2State == LOW) {
          resetLoopCount = 0;
        }else {
          accumulatedZAngle = 90;
          accumulatedYAngle = 90;
          ZangleChange = 0;
          YangleChange = 0;
        }
        break;
    }
  }

  esp_now_send(broadcastAddress, (uint8_t *) &transferData, sizeof(transferData));
  delay(10);
}

