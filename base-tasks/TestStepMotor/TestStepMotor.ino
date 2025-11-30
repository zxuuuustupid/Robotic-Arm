#define DIR_NORMAL 1
#define DIR_INV    -1

int PUL = 3; //定义脉冲引脚
int DIR = 2; //定义方向销
int ENA = 4; //定义启用引脚
void setup() {
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
  Serial.begin(9600); 
  step(3200,1);

  step(3200,-1);

}
void loop() {

}
void step(int steps,int dir){
  if (dir==1){
    digitalWrite(DIR,HIGH);
  }
  else{
    digitalWrite(DIR,LOW);
  } 
  delayMicroseconds(500);
  for (int i = 0; i < steps; i++) 
  {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL, LOW);
    delayMicroseconds(50);
  }
  digitalWrite(PUL, LOW);
  delayMicroseconds(500);
}