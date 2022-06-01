#include <PS2X_lib.h>
//#include <ezButton.h>

#define rs485 Serial1


PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;
byte mreply[1000];
int liftDir = 0;
int stateDir = 0;
int mode = 1;

// motor catch
int rollDir = 0;
int right_state = 0;
int left_state = 0;

#define GO_UP -1000;
#define GO_DOWN 500;

#define GO_RIGHT_LEFT 400;
#define GO_LEFT_RIGHT -400;

// Motor back_right
#define ENCA_back_right 2
#define ENCB_back_right 3
#define PWM_back_right 5
#define IN2_back_right 6
#define IN1_back_right 7

//Motor front_right
#define ENCA_front_right 14
#define ENCB_front_right 15
#define PWM_front_right 8
#define IN2_front_right 17
#define IN1_front_right 16

//Motor back_left
#define ENCA_back_left 20
#define ENCB_back_left 21
#define PWM_back_left  9
#define IN2_back_left 23
#define IN1_back_left 22

//Motor front_left
#define ENCA_front_left 24
#define ENCB_front_left 25
#define PWM_front_left 4
#define IN2_front_left 27
#define IN1_front_left 26 

//Define direction state of running motor
#define GO_FORWARD 1
#define GO_BACKWARD -1
#define STOP_GO 0
#define DEFAULT_SPEED 120
#define FASTER 135
#define SLOWER -60

//initialize velocoty
int velo = 0;
int additional_speed = 0;

//initialize direct first state of running motor
int dir_front_right = 0;
int dir_back_right = 0;
int dir_front_left = 0;
int dir_back_left = 0;

//initialize first speed state of running motor
int speed_back_left = 0;
int speed_front_left = 0;
int speed_back_right = 0;
int speed_front_right = 0;

//define the state of joystick
int js_left_x = 128;
int js_left_y = 127;
int js_right_x = 128;
int js_right_y = 127;

void setup() {
  rs485.begin(115200);
  Serial.begin(9600);

  //Code for PS2 error to determind type of controller
  error = ps2x.config_gamepad(13, 11, 10, 12, true, true);
  if (error == 0) {
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }

  //Motor back_right
  pinMode(ENCA_back_right,INPUT);
  pinMode(ENCB_back_right,INPUT);
  pinMode(IN2_back_right, OUTPUT);
  pinMode(IN1_back_right, OUTPUT);
  pinMode(PWM_back_right, OUTPUT);

  //Motor front_right
  pinMode(ENCA_front_right,INPUT);
  pinMode(ENCB_front_right,INPUT);
  pinMode(IN2_front_right, OUTPUT);
  pinMode(IN1_front_right, OUTPUT);
  pinMode(PWM_front_right, OUTPUT);

  //Motor back_left
  pinMode(ENCA_back_left,INPUT);
  pinMode(ENCB_back_left,INPUT);
  pinMode(IN2_back_left, OUTPUT);
  pinMode(IN1_back_left, OUTPUT);
  pinMode(PWM_back_left, OUTPUT);

  //Motor front_left
  pinMode(ENCA_front_left,INPUT);
  pinMode(ENCB_front_left,INPUT);
  pinMode(IN2_front_left, OUTPUT);
  pinMode(IN1_front_left, OUTPUT);
  pinMode(PWM_front_left, OUTPUT);

  setMotor(0, 0, PWM_back_right, IN1_back_right, IN2_back_right);
  setMotor(0, 0, PWM_front_right, IN1_front_right, IN2_front_right);
  setMotor(0, 0, PWM_back_left, IN1_back_left, IN2_back_left);
  setMotor(0, 0, PWM_front_left, IN1_front_left, IN2_front_left);

  //run_speed(1, 0);
}

void loop() {
  if (error == 1)
    return;
  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
    if (ps2x.Button(PSB_START))                  //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if (ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");
    if (ps2x.Button(PSB_PAD_UP)) {        //will be TRUE as long as button is pressed
    //  Serial.print("Up held this hard: ");
    //  Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
      liftDir = ps2x.Analog(PSAB_PAD_UP);
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
      rollDir = ps2x.Analog(PSAB_PAD_RIGHT);
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
      rollDir = map(ps2x.Analog(PSAB_PAD_RIGHT), 0, 255, -255, 0);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
    //  Serial.print("DOWN held this hard: ");
    //  Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
      liftDir = map(ps2x.Analog(PSAB_PAD_UP),0, 255, -255, 0);
    }
    vibrate = ps2x.Analog(PSAB_BLUE);
    if (ps2x.NewButtonState())
    {
      if (ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if (ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if (ps2x.Button(PSB_L2)) {
        Serial.println("L2 pressed");
      }
      if (ps2x.Button(PSB_R2))
      
        Serial.println("R2 pressed");
      if (ps2x.Button(PSB_GREEN))
        Serial.println("Triangle pressed");
    }
    if (ps2x.ButtonPressed(PSB_RED))            //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if (ps2x.ButtonReleased(PSB_PINK))            //will be TRUE if button was JUST released
      Serial.println("Square just released");
    if (ps2x.NewButtonState(PSB_BLUE)){           //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    }

    //for running motor input read
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);
     // rollDir = ps2x.Analog(PSS_RX);
    js_left_x = ps2x.Analog(PSS_LX);
    js_left_y = ps2x.Analog(PSS_LY);
    js_right_x = ps2x.Analog(PSS_RX);
    js_right_y = ps2x.Analog(PSS_RY);
    //
    if(ps2x.Button(PSB_L1)){
      additional_speed = SLOWER;
    }
    if(ps2x.Button(PSB_R1)){
      additional_speed = FASTER;
    }
  }
  /*=================For streching motor======================*/
  
  if (rollDir == 0) {
    left_state = 0;
    right_state = 0;
  } else if (rollDir > 0) {
    left_state = GO_LEFT_RIGHT;
    right_state = GO_RIGHT_LEFT;
  } else if (rollDir < 0) {
    left_state = GO_RIGHT_LEFT;
    right_state = GO_LEFT_RIGHT;
  }


  /*=================For lift motor======================*/
  if (liftDir == 0) {
    stateDir = 0;
  } else if (liftDir > 0) {
    stateDir = GO_UP;
  } else if (liftDir < 0) {
    stateDir = GO_DOWN;
  }
  
 // run_speed(1, stateDir);

  //lefting motor
    
  //  run_speed(2, right_state);
  //  run_speed(3, left_state);

  //Giving direction codition to motor
  //GIve to Y axis of Left and right JOYStick
  
//  if(js_left_y <127)

  /*=================calculate the speed======================*/
  velo = DEFAULT_SPEED + additional_speed;


  
  /*=================For running motor======================*/
  if(js_left_y == 0)
  {
    dir_back_left = GO_FORWARD;
    dir_front_left = GO_FORWARD;
    speed_back_left = velo;
    speed_front_left = velo;
  }

//  if(js_right_y < 127)
  if(js_right_y == 0)
  {
    dir_back_right = GO_FORWARD;
    dir_front_right = GO_FORWARD;
    speed_back_right = velo;
    speed_front_right = velo;
  }
//  if(js_left_y > 127)
  if(js_left_y == 255)
  {
    dir_back_left = GO_BACKWARD;
    dir_front_left = GO_BACKWARD;
    speed_back_left = velo;
    speed_front_left = velo;
  }
//  if(js_right_y > 127)
  if(js_right_y == 255)
  {
    dir_back_right = GO_BACKWARD;
    dir_front_right = GO_BACKWARD;
    speed_back_right = velo;
    speed_front_right = velo;
  }
  //Give condition to X axis of Left and Right joyStick
  
 // if(js_left_x <128)
  if(js_left_x == 0)
  {
    dir_back_left = GO_FORWARD;
    dir_front_left = GO_BACKWARD;
    speed_back_left = velo;
    speed_front_left = velo;
  }

//  if(js_right_x < 128)
  if(js_right_x == 0)
  {
    dir_back_right = GO_BACKWARD;
    dir_front_right = GO_FORWARD;
    speed_back_right = velo;
    speed_front_right = velo;
  }
//  if(js_left_x > 128)
  if(js_left_x ==255)
  {
    dir_back_left = GO_BACKWARD;
    dir_front_left = GO_FORWARD;
    speed_back_left = velo;
    speed_front_left = velo;
  }
//  if(js_right_x > 128)
  if(js_right_x == 255)
  {
    dir_back_right = GO_FORWARD;
    dir_front_right = GO_BACKWARD;
    speed_back_right = velo;
    speed_front_right = velo;
  }
  /*=================Diagonal direction======================*
  /*=================Execute run motor======================*/
    setMotor(dir_back_right, speed_back_right, PWM_back_right, IN1_back_right, IN2_back_right);
    setMotor(dir_front_right, speed_front_right, PWM_front_right, IN1_front_right, IN2_front_right);
    setMotor(dir_back_left, speed_back_left, PWM_back_left, IN1_back_left, IN2_back_left);
    setMotor(dir_front_left, speed_front_left, PWM_front_left, IN1_front_left, IN2_front_left);


    
  /*=================Reset everthing part======================*/

  //resut the velo and additional speed
  additional_speed = 0;
  velo = 0;

  //reset direction of motor
  dir_back_right = 0;
  dir_front_right = 0;
  dir_back_left = 0;
  dir_front_left = 0;

  
  //reset speed motor
  speed_back_right = 0;
  speed_front_right = 0;
  speed_back_left = 0;
  speed_front_left = 0;

  //reset js(Joystick) state
  js_left_x = 128;
  js_left_y = 127;
  js_right_x = 128;
  js_right_y = 127;


  //resetPart
  liftDir = 0;// reset to the orginal state
  stateDir = 0;
  rollDir = 0;
  left_state = 0;
  right_state = 0;
  delay(40);
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else if(dir == -1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}
