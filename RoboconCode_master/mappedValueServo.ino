#define MIN 0
#define MAX 180

uint8_t servonumR1 = 0;
uint8_t servonumM1 = 4;
uint8_t servonumL1 = 8;

uint8_t servonumR2 = 1;
uint8_t servonumM2 = 5;
uint8_t servonumL2 = 9;

uint8_t servonumR31 = 2;
uint8_t servonumM31 = 6;
uint8_t servonumL31 = 10;

uint8_t servonumR32 = 3;
uint8_t servonumM32 = 7;
uint8_t servonumL32 = 11;

int valueR1, valueM1, valueL1;
int valueR2, valueM2, valueL2;
int valueR31, valueM31, valueL31;
int valueR32, valueM32, valueL32;

int mappedValueR1UpDown(int value){
  int mappedValue = map(value, MIN, MAX, 260, 380);
  return mappedValue;
}

int mappedValueM1UpDown(int value){
  int mappedValue = map(value, MIN, MAX, 265, 390);
  return mappedValue;
}

int mappedValueL1UpDown(int value){
  int mappedValue = map(value, MIN, MAX, 95, 220);
  return mappedValue;
}

int mappedValueR2CCWCW(int value){
  int mappedValue = map(value, MIN, MAX,115, 555);
  return mappedValue;
}

int mappedValueM2CCWCW(int value){
  int mappedValue = map(value, MIN, MAX,120, 550);
  return mappedValue;
}

int mappedValueL2CCWCW(int value){
  int mappedValue = map(value, MIN, MAX,120, 555);
  return mappedValue;
}

int mappedValueR31InOut(int value){
  int mappedValue = map(value, MIN, MAX, 340, 370);
  return mappedValue;
}

int mappedValueR32InOut(int value){
  int mappedValue = map(value, MIN, MAX, 290, 270);
  return mappedValue;
}

int mappedValueM31InOut(int value){
  int mappedValue = map(value, MIN, MAX, 290, 270);
  return mappedValue;
}

int mappedValueM32InOut(int value){
  int mappedValue = map(value, MIN, MAX, 340, 370);
  return mappedValue;
}

int mappedValueL31InOut(int value){
  int mappedValue = map(value, MIN, MAX, 300, 270);
  return mappedValue;
}

int mappedValueL32InOut(int value){
  int mappedValue = map(value, MIN, MAX, 190, 220);
  return mappedValue;
}


void initialPosition(){
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  Serial.println("16 channel Servo begin!");
  yield();
  valueR1 = mappedValueR1UpDown(0);
  valueM1 = mappedValueM1UpDown(0);
  valueL1 = mappedValueL1UpDown(0);

  valueR2 = mappedValueR2CCWCW(90);
  valueM2 = mappedValueM2CCWCW(90);
  valueL2 = mappedValueL2CCWCW(90);

  valueR31 = mappedValueR31InOut(180);
  valueM31 = mappedValueM31InOut(180);
  valueL31 = mappedValueL31InOut(180);

  valueR32 = mappedValueR32InOut(180);
  valueM32 = mappedValueM32InOut(180);
  valueL32 = mappedValueL32InOut(180);

  pwm.setPWM(servonumR1, 0, valueR1);
  pwm.setPWM(servonumM1, 0, valueM1);
  pwm.setPWM(servonumL1, 0, valueL1);

  pwm.setPWM(servonumR2, 0, valueR2);
  pwm.setPWM(servonumM2, 0, valueM2);
  pwm.setPWM(servonumL2, 0, valueL2);

  pwm.setPWM(servonumR31, 0, valueR31);
  pwm.setPWM(servonumM31, 0, valueM31);
  pwm.setPWM(servonumL31, 0, valueL31);

  pwm.setPWM(servonumR32, 0, valueR32);
  pwm.setPWM(servonumM32, 0, valueM32);
  pwm.setPWM(servonumL32, 0, valueL32);
}

void control_R1(int value){
  int mappedValue = mappedValueR1UpDown(value);
  pwm.setPWM(servonumR1, 0, mappedValue);
}

void control_R2(int value){
  int mappedValue = mappedValueR2CCWCW(value);
  pwm.setPWM(servonumR2, 0, mappedValue);
}

void control_R31(int value){
  int mappedValue = mappedValueR31InOut(value);
  pwm.setPWM(servonumR31, 0, mappedValue);
}

void control_R32(int value){
  int mappedValue = mappedValueR32InOut(value);
  pwm.setPWM(servonumR32, 0, mappedValue);
}

void control_M1(int value){
  int mappedValue = mappedValueM1UpDown(value);
  pwm.setPWM(servonumM1, 0, mappedValue);
}

void control_M2(int value){
  int mappedValue = mappedValueM2CCWCW(value);
  pwm.setPWM(servonumM2, 0, mappedValue);
}

void control_M31(int value){
  int mappedValue = mappedValueM31InOut(value);
  pwm.setPWM(servonumM31, 0, mappedValue);
}

void control_M32(int value){
  int mappedValue = mappedValueM32InOut(value);
  pwm.setPWM(servonumM32, 0, mappedValue);
}

void control_L1(int value){
  int mappedValue = mappedValueL1UpDown(value);
  pwm.setPWM(servonumL1, 0, mappedValue);
}

void control_L2(int value){
  int mappedValue = mappedValueL2CCWCW(value);
  pwm.setPWM(servonumL2, 0, mappedValue);
}

void control_L31(int value){
  int mappedValue = mappedValueL31InOut(value);
  pwm.setPWM(servonumL31, 0, mappedValue);
}

void control_L32(int value){
  int mappedValue = mappedValueL32InOut(value);
  pwm.setPWM(servonumL32, 0, mappedValue);
}
