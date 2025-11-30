#define STEPPER_CW             true
#define STEPPER_CCW            false
// #define STEP_PER_ROUND    3200
// #define DIST_PER_ROUND    80
#define STEPPER_DIR        2
#define STEPPER_PUL        3
#include<MsTimer2.h>

typedef struct {
  uint16_t steps;
  bool     dir;
} Stepper_Ctrl;

Stepper_Ctrl step42;

void setup() {
  StepperInit();
  X_Move(600, STEPPER_CCW);
  // Serial.begin(9600); 
  // X_Dist(100,1);

}
void loop() {
  
}

void StepperInit(){
  step42.steps = 0;
  step42.dir = STEPPER_CW;
  // init io
  pinMode (STEPPER_PUL, OUTPUT);
  pinMode (STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, STEPPER_CW);
  digitalWrite(STEPPER_PUL, LOW);
  // 50ms interrupt
  MsTimer2::set(0.001, __check_stepper_pulse);
  MsTimer2::start();
}

void __check_stepper_pulse(){
  static bool __pulse = LOW;
  if(step42.steps > 0) {
    __pulse = !__pulse;
   digitalWrite(STEPPER_PUL, __pulse);
   // a pulse finish
   if(__pulse == LOW){
    step42.steps--;
   }
  }else{
    __pulse = LOW;
    digitalWrite(STEPPER_PUL, __pulse);    
  }

}

void X_Move(uint16_t distance, bool direction){
  // set direction
  if(direction != step42.dir){
    digitalWrite(STEPPER_DIR, direction);
    step42.dir = direction;
  }
  // distance To steps
  step42.steps = distance * 400;
}

