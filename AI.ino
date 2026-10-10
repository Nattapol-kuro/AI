#include <EasyKids3in1.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

const int numSensor = 6;
const uint16_t sensor_pin[7] = { 0, 1, 2, 3, 4, 5, 6 };
int ref_W[7] = { 0, 2100, 2200, 1800, 2100, 2500, 2100 };
int ref_B[7] = { 0, 600, 700, 500, 750, 800, 500 };
int ref[7];

float present_position = 0;
float setpoint = 2500.0;
float integral = 0;
uint32_t _lastPosition = 2500;

int last_speed2 = -999;
int last_speed3 = -999;
String data;

unsigned long startTime = 0;


void setMotors(int speed2, int speed3) {
  speed2 = constrain(speed2, -100, 100);
  speed3 = constrain(speed3, -100, 100);

  if (speed2 != last_speed2 || speed3 != last_speed3) {
    motor(2, speed2);
    motor(3, speed3);
    last_speed2 = speed2;
    last_speed3 = speed3;
  }
}

int position() {
  bool onLine = false;
  long avg = 0;
  long sum = 0;

  for (uint8_t i = 1; i <= numSensor; i++) {
    int raw = analog(sensor_pin[i]);
    long value = map(raw, ref_B[i], ref_W[i], 0, 1000);
    value = constrain(value, 0, 1000);

    if (value > 200) {
      onLine = true;
    }
    if (value > 50) {
      avg += (long)value * ((i - 1) * 1000);
      sum += value;
    }
  }

  if (!onLine) {
    if (_lastPosition < 2500) {
      return 0;
    } else {
      return 5000;
    }
  }

  _lastPosition = avg / sum;
  return _lastPosition;
}

void RunF(float Kp, float Ki, float Kd, int speed_max, unsigned long duration) {
  unsigned long cur = millis();
  while (millis() - cur <= duration) {
    present_position = position();
    errors = present_position - setpoint;
    integral = integral + errors;
    integral = constrain(integral, -1000, 1000);

    output = (Kp * errors) + (Ki * integral) + (Kd * (errors - previous_error));
    previous_error = errors;

    int LeftM = speed_max + output;
    int RightM = speed_max - output;

    LeftM = constrain(LeftM, -20, 100);
    RightM = constrain(RightM, -20, 100);

    setMotors(LeftM, RightM);
    delay(5);
  }
  setMotors(0, 0);
  motorStopAll();
}

void RunZ(float Kp, float Ki, float Kd, int speed_max) {
  while (analog(1) < ref[1] || analog(6) < ref[6]) {
    present_position = position();
    errors = present_position - setpoint;
    integral = integral + errors;
    integral = constrain(integral, -1000, 1000);

    output = (Kp * errors) + (Ki * integral) + (Kd * (errors - previous_error));
    previous_error = errors;

    output = constrain(output, -20, 20);

    int LeftM = speed_max + output;
    int RightM = speed_max - output;

    LeftM = constrain(LeftM, 5, 100);
    RightM = constrain(RightM, 5, 100);

    setMotors(LeftM, RightM);
    delay(5);
  }
  setMotors(0, 0);
  motorStopAll();
}
void RunT(float Kp, float Ki, float Kd, int speed_max) {
  while (analog(1) < ref[1] || analog(6) < ref[6]) {
    present_position = position();
    errors = present_position - setpoint;
    integral = integral + errors;
    integral = constrain(integral, -1000, 1000);

    output = (Kp * errors) + (Ki * integral) + (Kd * (errors - previous_error));
    previous_error = errors;

    int LeftM = speed_max + output;
    int RightM = speed_max - output;

    LeftM = constrain(LeftM, -20, 100);
    RightM = constrain(RightM, -20, 100);

    setMotors(LeftM, RightM);
    delay(5);
  }
  setMotors(0, 0);
  motorStopAll();
}

void RunR(float Kp, float Ki, float Kd, int speed_max) {
  while (analog(4) < ref[4] || analog(6) < ref[6]) {
    present_position = position();
    errors = present_position - setpoint;
    integral = integral + errors;
    integral = constrain(integral, -1000, 1000);

    output = (Kp * errors) + (Ki * integral) + (Kd * (errors - previous_error));
    previous_error = errors;

    int LeftM = speed_max + output;
    int RightM = speed_max - output;

    LeftM = constrain(LeftM, -20, 100);
    RightM = constrain(RightM, -20, 100);

    setMotors(LeftM, RightM);
    delay(5);
  }
  setMotors(0, 0);
  motorStopAll();
}

void RunL(float Kp, float Ki, float Kd, int speed_max) {
  while (analog(3) < ref[3] || analog(1) < ref[1]) {
    present_position = position();
    errors = present_position - setpoint;
    integral = integral + errors;
    integral = constrain(integral, -1000, 1000);

    output = (Kp * errors) + (Ki * integral) + (Kd * (errors - previous_error));
    previous_error = errors;

    int LeftM = speed_max + output;
    int RightM = speed_max - output;

    LeftM = constrain(LeftM, -30, 100);
    RightM = constrain(RightM, -30, 100);

    setMotors(LeftM, RightM);
    delay(5);
  }
  setMotors(0, 0);
  motorStopAll();
}

void L1() {
  setMotors(-40, 40);
  delay(160);
  while (analog(1) < ref[1]) {
    setMotors(-40, 40);
  }
  while (analog(2) > ref[2]) {
    setMotors(-30, 30);
  }
  setMotors(0, 0);
  motorStopAll();
}

void R1() {
  setMotors(40, -40);
  delay(160);
  while (analog(6) < ref[6]) {
    setMotors(40, -40);
  }
  while (analog(5) > ref[5]) {
    setMotors(30, -30);
  }
  setMotors(0, 0);
  motorStopAll();
}

void setReports(void) {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Error: Could not enable game vector");
  }
}

float getYaw() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      float siny_cosp = 2.0 * (qr * qk + qi * qj);
      float cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk);
      float yaw = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;

      if (yaw < 0) yaw += 360.0;
      return yaw;
    }
  }
  return -1.0;
}

float getFreshYaw() {
  float freshYaw = -1.0;
  for (int i = 0; i < 7; i++) {
    float y = getYaw();
    if (y >= 0) freshYaw = y;
    delay(5);
  }
  while (freshYaw < 0) {
    freshYaw = getYaw();
    delay(5);
  }
  return freshYaw;
}

float getAngleError(float target, float current) {
  float error = target - current;
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;
  return error;
}
void RunF_IMU_Time(int speed_base, unsigned long duration, float Kp = 0.2, float Ki = 0.0, float Kd = 0.008) {
  float targetYaw = getFreshYaw();
  float previous_error = 0;
  float integral = 0;
  unsigned long startTime = millis();

  while (millis() - startTime <= duration) {
    float currentYaw = getYaw();
    if (currentYaw < 0) continue;

    float error = getAngleError(targetYaw, currentYaw);

    integral = constrain(integral + error, -200, 200);
    float output = (Kp * error) + (Ki * integral) + (Kd * (error - previous_error));
    previous_error = error;

    int leftSpeed = constrain(speed_base - output, -100, 100);
    int rightSpeed = constrain(speed_base + output, -100, 100);

    setMotors(leftSpeed, rightSpeed);
    delay(5);
  }

  // ระบบฉุกเบรกย้อนทิศสั้นๆ ป้องกันการไถล
  setMotors(-speed_base, -speed_base);
  delay(20);

  setMotors(0, 0);
  motorStopAll();
  delay(50);
}

void turnToAngle(float targetYaw, float maxSpeed) {
  float Kp = 0.27;
  float Kd = 0.95;
  float minSpeed = 15;
  float tolerance = 1;

  float lastError = 0;
  unsigned long startTime = millis();

  float initialYaw = getFreshYaw();
  lastError = getAngleError(targetYaw, initialYaw);

  while (millis() - startTime < 3000) {
    float currentYaw = getYaw();
    if (currentYaw < 0) continue;

    float error = getAngleError(targetYaw, currentYaw);

    if (abs(error) <= tolerance) {
      break;
    }

    float derivative = error - lastError;
    lastError = error;

    float output = (error * Kp) + (derivative * Kd);
    float speed = abs(output);
    speed = constrain(speed, minSpeed, maxSpeed);

    if (output > 0) {
      setMotors(-speed, speed);
    } else {
      setMotors(speed, -speed);
    }

    delay(8);
  }

  setMotors(0, 0);
  motorStopAll();
  delay(100);
}

void turnAngle(float relativeAngle, float maxSpeed) {
  float startYaw = getFreshYaw();
  float targetYaw = startYaw + relativeAngle;
  if (targetYaw >= 360.0) targetYaw -= 360.0;
  if (targetYaw < 0.0) targetYaw += 360.0;

  turnToAngle(targetYaw, maxSpeed);
}

void turnLeft90(int maxSpeed) {
  turnAngle(90, maxSpeed);
}
void turnRight90(int maxSpeed) {
  turnAngle(-90, maxSpeed);
}

int angle = 90;
int direction = -1;

bool foundName = false;
bool sweeping = true;

unsigned long servoTime = 0;
int state = 1;
int tag = 0;

#define BLINK5(color) \
  { \
    unsigned long t0 = millis(); \
    while (millis() - t0 < 5000) { \
      ledFillColor(color); \
      delay(200); \
      ledFillColor(BLACK); \
      delay(200); \
    } \
  }

void setup() {
  EasyKids_Setup();
  Serial.begin(115200);
  beginSerialRX();
  ledSetBrightness(20);

  display3in1();
  display.setTextSize(4);

  displayClear();
  display.setCursor(0, 190);
  display.println("READY");

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) delay(10);
  }
  setReports();

  for (int i = 1; i <= 6; i++) {
    ref[i] = (ref_W[i] + ref_B[i]) / 2;
  }

  servo(1, 0);
  servo(2, 1);
  servo(6, 90);
}

void loop() {
  // readSensor();
  waitForStart();
  beep();
  // F1();
  CH();
  //   RunF(0.008, 0.0, 0.15, 60, 400);
  //   RunT(0.008, 0.0, 0.14, 40);
  //   R1();
  //   RunF(0.008, 0.0, 0.14, 40, 100);
  //   RunR(0.008, 0.0, 0.14, 80);
  //   RunZ(0.008, 0.0, 0.15, 80);
  //   setMotors(78, 80);
  //   delay(60);
  //   RunT(0.008, 0.0, 0.175, 80);
  //   setMotors(78, 80);
  //   delay(30);
  //   RunZ(0.008, 0.0, 0.16, 80);
  //   setMotors(78, 80);
  //   delay(50);
  //   RunR(0.008, 0.0, 0.15, 30);
  //   R1();
  //   RunF(0.008, 0.0, 0.16, 60, 400);
  // RunT(0.008, 0.0, 0.13, 40);
  //   /////////////////////////////////////////////////////
  //   bool foundName = false;

  //   while (!foundName) {
  //     for (int i = 120; i >= 60; i--) {
  //       servo(6, i);
  //       delay(27);

  //       if (Serial2.available()) {
  //         data = Serial2.readStringUntil('\n');
  //         data.trim();

  //         if (data == "Lufy" || data == "Gundam" || data == "Anya" || data == "Pikachu" || data == "Demean") {

  //           displayClear();
  //           display.setCursor(0, 190);
  //           display.println(data);

  //           foundName = true;
  //           delay(2000);
  //           break;
  //         }
  //       }
  //     }
  //   }
  //   bool foundNumber = false;

  // while (!foundNumber) {
  //     for (int i = 100; i >= 30; i--) {
  //       servo(6, i);
  //       delay(30);
  //       if (Serial2.available()) {
  //         data = Serial2.readStringUntil('\n');
  //         data.trim();
  //         displayClear();
  //         display.setCursor(0, 0);

  //         if (data == "1") {
  //           display.println("1");
  //           F1();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(RED);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           HO();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(BLUE);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           SC();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(YELLOW);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           SH();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(GREEN);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           CH();
  //         } else if (data == "2") {
  //           display.println("2");
  //           F1();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(RED);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           SC();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(BLUE);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           CH();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(YELLOW);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           HO();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(GREEN);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           SH();

  //         } else if (data == "3") {
  //           display.println("3");
  //           F1();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(RED);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           SH();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(BLUE);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           HO();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(YELLOW);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           CH();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(GREEN);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           SC();

  //         } else if (data == "4") {
  //           display.println("4");
  //           F1();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(RED);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           CH();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(BLUE);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           SH();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(YELLOW);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           SC();
  //           startTime = millis();
  //           while (millis() - startTime < 5000) {
  //             ledFillColor(GREEN);
  //             delay(200);
  //             ledFillColor(BLACK);
  //             delay(200);
  //           }
  //           HO();
  //         }
  //         if (data == "1" || data == "2" || data == "3" || data == "4") {
  //           foundNumber = true;
  //           break;
  //         }
  //       }
  //     }
  //   }
  setMotors(0, 0);
  motorStopAll();
  delay(100);
}