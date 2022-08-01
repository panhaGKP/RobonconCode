//motor right front side
#define R_IS_RIGHT_FRONT 22
#define L_IS_RIGHT_FRONT 23
#define R_EN_RIGHT_FRONT 24
#define L_EN_RIGHT_FRONT 25
#define R_PWM_RIGHT_FRONT 2
#define L_PWM_RIGHT_FRONT 3

//motor right back side
#define R_IS_RIGHT_BACK 30
#define L_IS_RIGHT_BACK 31
#define R_EN_RIGHT_BACK 32
#define L_EN_RIGHT_BACK 33
#define R_PWM_RIGHT_BACK 8
#define L_PWM_RIGHT_BACK 9

//motor left front side
#define R_IS_LEFT_FRONT 26
#define L_IS_LEFT_FRONT 27
#define R_EN_LEFT_FRONT 28
#define L_EN_LEFT_FRONT 29
#define R_PWM_LEFT_FRONT 4
#define L_PWM_LEFT_FRONT 5

//motor left back side
#define R_IS_LEFT_BACK 34
#define L_IS_LEFT_BACK 35
#define R_EN_LEFT_BACK 36
#define L_EN_LEFT_BACK 37
#define R_PWM_LEFT_BACK 6
#define L_PWM_LEFT_BACK 7



void setRunningMotor(int dir, int pwmVal, int R_PWM, int L_PWM) {
  if (dir == 1) {
    analogWrite(R_PWM, pwmVal);
    analogWrite(L_PWM, 0);
  } else if (dir == -1) {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, pwmVal);
  } else {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
  }
}

void initialRunningMotor(){
  //run motor right front side
  pinMode(R_IS_RIGHT_FRONT, OUTPUT);
  pinMode(R_EN_RIGHT_FRONT, OUTPUT);
  pinMode(L_IS_RIGHT_FRONT, OUTPUT);
  pinMode(L_EN_RIGHT_FRONT, OUTPUT);
  digitalWrite(R_IS_RIGHT_FRONT, LOW);
  digitalWrite(L_IS_RIGHT_FRONT, LOW);
  digitalWrite(R_EN_RIGHT_FRONT, HIGH);
  digitalWrite(L_EN_RIGHT_FRONT, HIGH);

  // run motor left front side
  pinMode(R_IS_LEFT_FRONT, OUTPUT);
  pinMode(R_EN_LEFT_FRONT, OUTPUT);
  pinMode(L_IS_LEFT_FRONT, OUTPUT);
  pinMode(L_EN_LEFT_FRONT, OUTPUT);
  digitalWrite(R_IS_LEFT_FRONT, LOW);
  digitalWrite(L_IS_LEFT_FRONT, LOW);
  digitalWrite(R_EN_LEFT_FRONT, HIGH);
  digitalWrite(L_EN_LEFT_FRONT, HIGH);

  //run motor right back side
  pinMode(R_IS_RIGHT_BACK, OUTPUT);
  pinMode(R_EN_RIGHT_BACK, OUTPUT);
  pinMode(L_IS_RIGHT_BACK, OUTPUT);
  pinMode(L_EN_RIGHT_BACK, OUTPUT);
  digitalWrite(R_IS_RIGHT_BACK, LOW);
  digitalWrite(L_IS_RIGHT_BACK, LOW);
  digitalWrite(R_EN_RIGHT_BACK, HIGH);
  digitalWrite(L_EN_RIGHT_BACK, HIGH);

  // run motor left back side
  pinMode(R_IS_LEFT_BACK, OUTPUT);
  pinMode(R_EN_LEFT_BACK, OUTPUT);
  pinMode(L_IS_LEFT_BACK, OUTPUT);
  pinMode(L_EN_LEFT_BACK, OUTPUT);
  digitalWrite(R_IS_LEFT_BACK, LOW);
  digitalWrite(L_IS_LEFT_BACK, LOW);
  digitalWrite(R_EN_LEFT_BACK, HIGH);
  digitalWrite(L_EN_LEFT_BACK, HIGH);
}

void setRunningMotorToward(int speedMotor){
  //right
  setRunningMotor(1, speedMotor, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(1, speedMotor, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(1, speedMotor, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(1, speedMotor, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}

void setRunningMotorBackward(int speedMotor){
  //right
  setRunningMotor(-1, speedMotor, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(-1, speedMotor, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(-1, speedMotor, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(-1, speedMotor, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}

void setRunningMotorLeftward(int speedMotor){
  //right
  setRunningMotor(1, speedMotor, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(-1, speedMotor, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(-1, speedMotor, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(1, speedMotor, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}

void setRunningMotorRightward(int speedMotor){
  //right
  setRunningMotor(-1, speedMotor, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(1, speedMotor, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(1, speedMotor, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(-1, speedMotor, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}

void setRunningMotorRotateLeft(int speedMotor){
  //right
  setRunningMotor(1, speedMotor, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(1, speedMotor, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(-1, speedMotor, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(-1, speedMotor, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}

void setRunningMotorRotateRight(int speedMotor){
  //right
  setRunningMotor(-1, speedMotor, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(-1, speedMotor, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(1, speedMotor, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(1, speedMotor, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}

void setRunningMotorDriftLeft(int speedMotor){
  //right
  setRunningMotor(0, speedMotor, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(0, speedMotor, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(1, speedMotor, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(1, speedMotor, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}

void setRunningMotorDriftRight(int speedMotor){
  //right
  setRunningMotor(1, speedMotor, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(1, speedMotor, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(0, speedMotor, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(0, speedMotor, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}

void setRunningMotorStop(){
  //right
  setRunningMotor(0, 0, R_PWM_RIGHT_FRONT, L_PWM_RIGHT_FRONT);
  setRunningMotor(0, 0, R_PWM_RIGHT_BACK, L_PWM_RIGHT_BACK);
  //left
  setRunningMotor(0, 0, R_PWM_LEFT_FRONT, L_PWM_LEFT_FRONT);
  setRunningMotor(0, 0, R_PWM_LEFT_BACK, L_PWM_LEFT_BACK);
}
