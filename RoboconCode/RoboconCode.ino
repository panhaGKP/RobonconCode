#include <PS2X_lib.h>
//#include <ezButton.h>

#define rs485 Serial1


PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;
byte mreply[1000];
int liftDir = 127;
int stateDir = 0;

// motor catch
int mode = 1;
int rollDir = 128;
int right_state = 0;
int left_state = 0;

#define GO_UP -1000;
#define GO_DOWN 1000;

#define GO_RIGHT_LEFT 400;
#define GO_LEFT_RIGHT -400;

void setup() {
  rs485.begin(115200);
  Serial.begin(9600);
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
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
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
      //  mode++;
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
    if (ps2x.NewButtonState(PSB_BLUE))           //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if (ps2x.Button(PSB_L1)||ps2x.Button(PSB_R1)) // print stick values if either is TRUE
    {
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
      Serial.print(",");
      liftDir = ps2x.Analog(PSS_LY);
      Serial.print(ps2x.Analog(PSS_LX), DEC);
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC);
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC);
      rollDir = ps2x.Analog(PSS_RX);
    }
  }
  //   Serial.println(liftDir);
  if (rollDir == 128 || rollDir == 129 || rollDir == 127) {
    left_state = 0;
    right_state = 0;
  } else if (rollDir < 126) {
    left_state = GO_LEFT_RIGHT;
    right_state = GO_RIGHT_LEFT;
  } else if (rollDir > 129) {
    left_state = GO_RIGHT_LEFT;
    right_state = GO_LEFT_RIGHT;
  }
  if (liftDir == 127 || liftDir == 126 || liftDir == 128) {
    stateDir = 0;
  } else if (liftDir > 128) {
    stateDir = GO_DOWN;
  } else if (liftDir < 126) {
    stateDir = GO_UP;
  }
  
  run_speed(1, stateDir);
  run_speed(2, right_state);
  run_speed(3, left_state);
  
  rollDir = 128;

  liftDir = 127;// reset to the orginal state
  stateDir = 0;

  Serial.print(left_state);
  Serial.print(",");
  Serial.println(right_state);
  Serial.print("mode: ");
  Serial.println(mode);
  rollDir = 128;
  left_state = 0;
  right_state = 0;
  delay(40);
 
}
