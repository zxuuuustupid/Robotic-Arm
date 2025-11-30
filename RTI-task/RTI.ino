// #include <avr/pgmspace.h>
#include <LobotServoController.h>
// #include "angles_and_xmove.h"

LobotServoController myse;
#define PUL 3
#define DIR 2

const float MM_PER_REV = 8.0;
const long PULSES_PER_REV = 3600L;
const float PULSES_PER_MM = PULSES_PER_REV / MM_PER_REV; // 450

const int MOVE_DURATION = 50;
const float START_X = 0.0;

uint8_t buffer[11];
int index = 0;
float last_x = 0.0f;

int angleToPulse(int id, float angle) {
  if (id == 0) {
    // 物理角度 0° 对应 2500μs，180° 对应 500μs
    angle = constrain(angle, 0, 180);
    return 2500 - (int)(angle / 180.0f * 2000.0f);
  }
  else if (id == 4) {
    // 1号舵机零点已校正：0° = 500μs, 180° = 2500μs
    float minA = 0, maxA = 180;
    angle = constrain(angle, minA, maxA);
    return 500 + (int)((angle - minA) / (maxA - minA) * 2000.0f);
  }
  else if (id == 8) {
    // 2号舵机：-90° = 500μs, 90° = 2500μs
    float minA = -90, maxA = 90;
    angle = constrain(angle, minA, maxA);
    return 500 + (int)((angle - minA) / (maxA - minA) * 2000.0f);
  }
  else {
    return 1500;
  }
}


void moveStepperRelative(float delta_mm, int duration_ms) {
  if (abs(delta_mm) < 0.01f) return;
  long pulses = (long)(abs(delta_mm) * PULSES_PER_MM);
  if (pulses == 0) return;

  digitalWrite(DIR, (delta_mm >= 0) ? HIGH : LOW);

  // 关键：总时间 = duration_ms，分配给 pulses 个脉冲
  long totalUs = (long)duration_ms * 1000L;
  long intervalUs = totalUs / pulses;  // 每个脉冲间隔（高+低）

  // 保证高电平至少 200us（驱动器要求）
  long highTime = 200;
  long lowTime = intervalUs - highTime;
  if (lowTime < 0) lowTime = 0;  // 防止负值（此时速度已达上限）

  for (long i = 0; i < pulses; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(highTime);
    digitalWrite(PUL, LOW);
    if (i < pulses - 1) delayMicroseconds(lowTime); // 最后一个脉冲后无需等待
  }
}

//
 void setup() {

  Serial.begin(9600);
  while (!Serial);

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(PUL, LOW);


}


void loop() {
  if (Serial.available() > 0) {
    buffer[index] = Serial.read();
    index++;
    
    if (index == 11) {
      if (buffer[0] == 0xAA && buffer[1] == 0x55) {
        int16_t pos = (buffer[3] << 8) | buffer[2];
        int16_t motor1 = (buffer[5] << 8) | buffer[4];
        int16_t motor2 = (buffer[7] << 8) | buffer[6];
        int16_t motor3 = (buffer[9] << 8) | buffer[8];
        // debug
        // Serial.print(pos / 100.0);
        // Serial.print(",");
        // Serial.print(motor1 / 10.0);
        // Serial.print(",");
        // Serial.print(motor2 / 10.0);
        // Serial.print(",");
        // Serial.println(motor3 / 10.0);

        // 还原为 float
        float x_abs = pos / 100.0f;    // mm
        float a1    = motor1 / 10.0f;    // degrees
        float a2    = motor2 / 10.0f;
        float a3    = motor3 / 10.0f;

        float delta_x = x_abs - last_x;
        int p0 = angleToPulse(0, a1);
        int p1 = angleToPulse(4, a2);
        int p2 = angleToPulse(8, a3);

        LobotServo s[3] = {{0, p0}, {4, p1}, {8, p2}};
        myse.moveServos(s, 3, MOVE_DURATION);
        moveStepperRelative(delta_x, MOVE_DURATION);
        last_x = x_abs;
        delay(MOVE_DURATION + 20);
      }
      index = 0;
    }
  }
}