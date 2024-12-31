
/*/   Mobile Platform for Tele-operated Robotic Arm   /*/
// Code by Michael Fernandez, Nov 2024


#include <Servo.h>
#include <ESP8266WiFi.h>
#include <espnow.h>


// Driver pin
const uint8_t motorA = 14;  // D5
const uint8_t motorB = 16;  // D0
const uint8_t in1 = 2;      // D4
const uint8_t in2 = 0;      // D3
const uint8_t in3 = 4;      // D2
const uint8_t in4 = 5;      // D1


// Joystick variable
uint16_t joyXvalue = 533;
uint16_t joyYvalue = 533;
bool gyroCtrl = false;


// Servo
const uint8_t clawServoPin = 15;  // D8
const uint8_t pitchServoPin = 13; // D7
const uint8_t yawServoPin = 12;   // D6
bool clawBtn1 = false;
bool clawBtn2 = false;
uint8_t clawServoAngle = 90;

Servo pitchServo, yawServo, clawServo; 

uint8_t pitchServoAngle = 90;
uint8_t yawServoAngle = 90;
uint8_t lastPitchServoAngle = pitchServoAngle;
uint8_t lastYawServoAngle = yawServoAngle;


// Data Receiver
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


// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&transferData, incomingData, sizeof(transferData));
  joyXvalue = transferData.joyXdata;
  joyYvalue = transferData.joyYdata;
  gyroCtrl = transferData.gyroCtrlData;
  clawBtn1 = transferData.clawBtn1Data;
  clawBtn2 = transferData.clawBtn2Data;
  pitchServoAngle = transferData.gyroZdata;
  yawServoAngle = transferData.gyroYdata;
}


void checkClawBtn() {
  static uint8_t loopCount = 0;
  static bool timeout = false;

  if (clawBtn1 == HIGH && clawBtn2 == LOW && clawServoAngle > 0) {
    clawServoAngle -= 5;
    clawServo.attach(clawServoPin);
    clawServo.write(clawServoAngle);
    timeout = true;
    loopCount = 0;
  }else if (clawBtn1 == LOW && clawBtn2 == HIGH && clawServoAngle < 90) {
    clawServoAngle += 5;
    clawServo.attach(clawServoPin);
    clawServo.write(clawServoAngle);
    timeout = true;
    loopCount = 0;
  }

  if (timeout == true && loopCount >= 100) {
    clawServo.detach();
    timeout = false;
    loopCount = 0;
  }else if (timeout == true) {
    loopCount++;
  }
}

void AB_MtrForwad() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
}
void AB_MtrBackward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void AB_MtrRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void AB_MtrLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void motorLogic() {
  static bool xNeutral;
  static bool yNeutral;
  const uint16_t maxNeutral = 540;
  const uint16_t minNeutral = 525;

  if (joyXvalue >= minNeutral - 50 && joyXvalue <= maxNeutral + 50) {
    xNeutral = true;
  } else {
    xNeutral = false;
  }
  if (joyYvalue >= minNeutral - 50 && joyYvalue <= maxNeutral + 50) {
    yNeutral = true;
  } else {
    yNeutral = false;
  }

  // Turn off motor joystick neutral
  if (xNeutral && yNeutral) {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  // Go Forward or turn right/left
  else if (joyYvalue <= minNeutral && xNeutral) {  // Straight forward
    AB_MtrForwad();
    uint8_t motorSpeed = map(joyYvalue, minNeutral, 0, 125, 255);
    analogWrite(motorA, motorSpeed);
    analogWrite(motorB, motorSpeed);

  } else if (joyYvalue <= minNeutral) {  // Turning right forward
    AB_MtrForwad();
    uint8_t motorSpeed = map(joyYvalue, minNeutral, 0, 125, 255);
    if (joyXvalue <= minNeutral) {
      uint8_t motorGoRight = map(joyXvalue, minNeutral, 0, 125, 255);
      int8_t totalSpeed = motorSpeed - motorGoRight;
      uint8_t aSpeed = constrain(totalSpeed, 125, 255);
      
      analogWrite(motorA, aSpeed);
      analogWrite(motorB, motorSpeed);

    }else {
      uint8_t motorGoLeft = map(joyXvalue, maxNeutral, 1024, 125, 255);
      int8_t totalSpeed = motorSpeed - motorGoLeft;
      uint8_t bSpeed = constrain(totalSpeed, 125, 255);
      
      analogWrite(motorA, motorSpeed);
      analogWrite(motorB, bSpeed);
    }
  }

  // Go bacward or turn left/right
  else if (joyYvalue >= maxNeutral && xNeutral){  // Straight backward
    AB_MtrBackward();
    uint8_t motorSpeed = map(joyYvalue, maxNeutral, 1024, 125, 255);
    analogWrite(motorA, motorSpeed);
    analogWrite(motorB, motorSpeed);

  } else if (joyYvalue >= maxNeutral) { // Turn rigt backward
    AB_MtrBackward();
    uint8_t motorSpeed = map(joyYvalue, maxNeutral, 1024, 125, 255);
    if (joyXvalue <= minNeutral) {
      uint8_t motorGoRight = map(joyXvalue, minNeutral, 0, 125, 255);
      int16_t totalSpeed = motorSpeed - motorGoRight;
      uint8_t aSpeed = constrain(totalSpeed, 125, 255);
      
      analogWrite(motorA, aSpeed);
      analogWrite(motorB, motorSpeed);

    }else {
      uint8_t motorGoLeft = map(joyXvalue, maxNeutral, 1024, 125, 255);
      int8_t totalSpeed = motorSpeed - motorGoLeft;
      uint8_t bSpeed = constrain(totalSpeed, 125, 255);
      
      analogWrite(motorA, motorSpeed);
      analogWrite(motorB, bSpeed);
    }
  } 

  // Go Right or left (Neutral steering)
  else if (yNeutral && joyXvalue <= minNeutral) {  // Right neutral steering
    AB_MtrRight();
    uint8_t motorSpeed = map(joyXvalue, minNeutral, 0, 125, 200);
    analogWrite(motorA, motorSpeed);
    analogWrite(motorB, motorSpeed);

  }else if (yNeutral && joyXvalue >= maxNeutral) { // Left neutral steering
    AB_MtrLeft();
    uint8_t motorSpeed = map(joyXvalue, maxNeutral, 1024, 125, 200);
    analogWrite(motorA, motorSpeed);
    analogWrite(motorB, motorSpeed);
  }
}

void checkResetBtn() {
  static uint8_t resetLoopCount = 0;

  switch (resetLoopCount) {
    case 0:
      if (clawBtn1 == HIGH && clawBtn2 == HIGH) {
        resetLoopCount++;
      }
      break;

    default:
      if (clawBtn1 == HIGH && clawBtn2 == HIGH && resetLoopCount < 100) {
        resetLoopCount++;
      }else if (clawBtn1 == LOW && clawBtn2 == LOW) {
        resetLoopCount = 0;
      }else {
        lastPitchServoAngle = pitchServoAngle;
        lastYawServoAngle = yawServoAngle;
        pitchServo.attach(pitchServoPin);
        yawServo.attach(yawServoPin);
        bool done = false;

        for (; !done ;) {
          if (lastPitchServoAngle > 90) {
            lastPitchServoAngle --;
            pitchServo.write(lastPitchServoAngle);
          }else if (lastPitchServoAngle < 90) {
            lastPitchServoAngle ++;
            pitchServo.write(lastPitchServoAngle);
          }

          if (lastYawServoAngle > 90) {
            lastYawServoAngle --;
            yawServo.write(lastYawServoAngle);
          }else if (lastYawServoAngle < 90) {
            lastYawServoAngle ++;
            yawServo.write(lastYawServoAngle);
          }

          if (lastYawServoAngle == 90 && lastPitchServoAngle == 90) {
            resetLoopCount = 0;
            pitchServo.detach();
            yawServo.detach();
            done = true;
          }
        }
      }
      break;
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  clawServo.attach(clawServoPin);
  pitchServo.attach(pitchServoPin);
  yawServo.attach(yawServoPin);
  pitchServo.write(90);
  yawServo.write(90);
  clawServo.write(90);
  delay(20);
  pitchServo.detach();
  yawServo.detach();
  clawServo.detach();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  motorLogic();
  
  if (gyroCtrl) {
    checkClawBtn();

    pitchServo.attach(pitchServoPin);
    yawServo.attach(yawServoPin);
    uint8_t fltredPitchAngle = constrain(pitchServoAngle, 25, 180);
    pitchServo.write(fltredPitchAngle);
    yawServo.write(yawServoAngle);

  } else {
    pitchServo.detach();
    yawServo.detach();
    checkResetBtn();
  }

  delay(10);
}
