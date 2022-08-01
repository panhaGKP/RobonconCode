#define MIN 0
#define MAX 180


// UpDown

int mappedValueServoUpDown(int value){
  int mappedValue = map(value, MIN, MAX, 90, 370);
  return mappedValue;
}

void controlServoUpDown(int value, int servoNum){
  value = mappedValueServoUpDown(value);
  pwm.setPWM(servoNum, 0, value);
}

// CWCCW

int mappedValueServoCWCCW(int value){
  int mappedValue = map(value, MIN, MAX, 110, 530);
  return mappedValue;
}

void controlServoCWCCW(int value, int servoNum){
  value = mappedValueServoCWCCW(value);
  pwm.setPWM(servoNum, 0, value);
}

// InOut

int mappedValueServoInOut(int value){
  int mappedValue = map(value, MIN, MAX, 90, 370);
  return mappedValue;
}

void controlServoInOut(int value, int servoNum){
  value = mappedValueServoInOut(value);
  pwm.setPWM(servoNum, 0, value);
}

// InOutPair

void controlServoInOutPair(int value, int gripperPosition){
  if (gripperPosition == 1){
    controlServoInOut(value, servonumR32);
    controlServoInOut(180-value, servonumR31);
  } else if(gripperPosition == 2){
    controlServoInOut(value, servonumM31);
    controlServoInOut(180-value, servonumM32);
  } else if(gripperPosition == 3){
    controlServoInOut(value, servonumL31);
    controlServoInOut(180-value, servonumL32);
  }
}

void initialPosition(){
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  Serial.println("16 channel Servo begin!");
  yield();
  
  controlServoUpDown(165, servonumR1);
  controlServoUpDown(165, servonumM1);
  controlServoUpDown(165, servonumL1);

  controlServoCWCCW(180, servonumR2);
  controlServoCWCCW(180, servonumM2);
  controlServoCWCCW(180, servonumL2);

  controlServoInOutPair(70, 1);
  controlServoInOutPair(70, 2);
  controlServoInOutPair(70, 3);
}

//void setup(){
//  initialPosition();
//}
//
//void loop(){
////  controlServoInOutPair(0, 1);
////  controlServoInOutPair(0, 2);
////  controlServoInOutPair(0, 3);
////controlServoCWCCW(0, servonumM2);
//
////controlServoInOutPair(0, 1);
//
//controlServoInOutPair(80, 1);
//  controlServoInOutPair(80, 2);
//  controlServoInOutPair(80, 3);
//
//}
