#include <avr/pgmspace.h>
#include <LobotServoController.h>
#include "angles_and_xmove.h"

LobotServoController myse;
#define PUL 3
#define DIR 2

const float MM_PER_REV = 8.0;
const long PULSES_PER_REV = 3600L;
const float PULSES_PER_MM = PULSES_PER_REV / MM_PER_REV; // 450

const int MOVE_DURATION = 50;
const float START_X = 0.0;

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

// //
//  void setup() {

//   Serial.begin(9600);
//   while (!Serial);

//   pinMode(PUL, OUTPUT);
//   pinMode(DIR, OUTPUT);
//   digitalWrite(PUL, LOW);

//   // 读取第一个点作为初始位置（虚拟起点）
//   int16_t first_x100 = pgm_read_word(&JOINT_TRAJ[0][0]);
//   float initial_x = first_x100 / 100.0f;
//   float last_x = initial_x; 

//   for (int i = 0; i < TRAJ_NUM_POINTS; i++) {
//     // 从 PROGMEM 读取整数
//     int16_t x100   = pgm_read_word(&JOINT_TRAJ[i][0]);
//     int16_t a1_10  = pgm_read_word(&JOINT_TRAJ[i][1]);
//     int16_t a2_10  = pgm_read_word(&JOINT_TRAJ[i][2]);
//     int16_t a3_10  = pgm_read_word(&JOINT_TRAJ[i][3]);

//     // 还原为 float
//     float x_abs = x100 / 100.0f;    // mm
//     float a1    = a1_10 / 10.0f;    // degrees
//     float a2    = a2_10 / 10.0f;
//     float a3    = a3_10 / 10.0f;

//     float delta_x = x_abs - last_x;
//     int p0 = angleToPulse(0, a1);
//     int p1 = angleToPulse(4, a2);
//     int p2 = angleToPulse(8, a3);

//     LobotServo s[3] = {{0, p0}, {4, p1}, {8, p2}};
//     myse.moveServos(s, 3, MOVE_DURATION);
//     moveStepperRelative(delta_x, MOVE_DURATION);
//     last_x = x_abs;
//     delay(MOVE_DURATION + 20);
//   }

//   float total_travel = last_x - initial_x;
//   if (abs(total_travel) > 0.01f) {
//     Serial.print(F("↩️ Returning "));
//     Serial.print(total_travel);
//     Serial.println(F(" mm to start position..."));
//     moveStepperRelative(-total_travel, 1000); // 慢速回退，1秒
//     delay(1020); // 略大于回退时间
//   }
// }

//画正方形部分
void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(PUL, LOW);

  moveStepperRelative(150, 1000); // 慢速回退，1秒
  delay(1000);
  myse.runActionGroup(100,1);  //运行100号动作组 
  delay(8000);
  moveStepperRelative(-150, 1000); // 慢速回退，1秒
  delay(1000);
  myse.runActionGroup(100,1);  //运行100号动作组 
  delay(8000);
}

void loop() {}