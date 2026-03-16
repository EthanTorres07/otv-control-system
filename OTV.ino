#include <Enes100.h>

/*
 * ENES100 OTV Control System - Team Seedlings
 * Autonomous robot navigation with obstacle avoidance
 * Hardware: Arduino Mega, HC-SR04 ultrasonic sensor, ESP32-CAM,
 * BTS7960 Motor Driver, DC Motors, Mecanum Wheels 
 */


// Vision System info
const int MARKER_ID = 0;
const int RM_NUM = 1120;

// Ultrasonic pins
const int TRIG = 28;
const int ECHO = 29;

double distance;

// ESP32-CAM pins
const int RX = 0;
const int TX = 1;

// Motor enable pins 
const int FL_EN = 22;
const int FR_EN = 23;
const int BL_EN = 24;
const int BR_EN = 25;
const int RK_EN = 26;
const int PLT_EN = 27;

// Motor forward and reverse pins (PWM)
const int FL_REV = 2;
const int FL_FWD = 3;
const int FR_REV = 4;
const int FR_FWD = 5;
const int BL_REV = 6;
const int BL_FWD = 7;
const int BR_REV = 8;
const int BR_FWD = 9;
const int RK_REV = 10;
const int RK_FWD = 11;
const int PLT_REV = 12;
const int PLT_FWD = 13;

// Distance threshold
const int STOP_DIST = 5;
const int SLOW_DIST = 15;

// OTV states
enum STATES {MISSION, NAVIGATION};
int state = MISSION;

// Current demo being run (-1 is no demo)
const int DEMO_MODE = -1;

// Motor directions
enum DIRECTIONS {FWD, LEFT, RIGHT, REV};
int direction = FWD;

// Motor speeds
const int FULL_SPEED = 191;
const int SLOW_SPEED = 128;
const int CRAWL_SPEED = 80;

// Possible mission sites
enum SITES {UP, DOWN};

// ML identification options
enum ML_KEY {ROCKS, ORZO};
  
void setup() {
  
  // Set pin modes
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  // Loop through all pins to set them to output
  for (int i = FL_EN; i < PLT_EN + 1; i++){
    pinMode(i, OUTPUT);
  }

  for (int i = FL_REV; i < PLT_FWD + 1; i++) {
    pinMode(i, OUTPUT);
  }

  Serial.begin(9600);
  Serial.println("Ready to begin!");
  Enes100.begin("SEEDLINGS", SEED, MARKER_ID, RM_NUM, TX, RX);
  
}

void loop() {

  // Do demos
  if (DEMO_MODE == 1)
    demo_forward();
  else if (DEMO_MODE == 2)
    demo_turning();
  else if (DEMO_MODE == 3)
    demo_wireless_rx()
  else if (DEMO_MODE == 4)
    demo_wireless_rx();
  else {
    // Do main demonstration
    switch (state) {
      case MISSION:
        do_mission();
        state = NAVIGATION;
        break;
      case NAVIGATION:
        navigate();
        break;
  }

  }

}

void navigate() {
  while (Enes100.getX() < 3.7){

    // Enable all drive motors
    motor_on_off(HIGH);
    update_distance();

    // Motor logic
    if (distance <= STOP_DIST) {

      // Stop motors and get past obstacle
      motor_on_off(LOW);
      escape(); 
    }
    else if (distance <= SLOW_DIST){

      // Set all motors to 50% speed
      set_motor_speed(SLOW_SPEED);
    }
    else {

      // Set all motors to 75% speed
      set_motor_speed(FULL_SPEED);
    }

  }

  // Turn of motors and stop
  motor_on_off(LOW);
  Enes100.println("Goal!!");
}

void motor_on_off(int value) {
  digitalWrite(FL_EN, value);
  digitalWrite(FR_EN, value);
  digitalWrite(BL_EN, value);
  digitalWrite(BR_EN, value);
}

void set_motor_speed(int value) {
  // Set all motors to zero
  analogWrite(FL_FWD, 0);
  analogWrite(FR_FWD, 0);
  analogWrite(BL_FWD, 0);
  analogWrite(BR_FWD, 0);
  analogWrite(FL_REV, 0);
  analogWrite(FR_REV, 0);
  analogWrite(BL_REV, 0);
  analogWrite(BR_REV, 0);

  // Set direction
  switch (direction) {
    case FWD:
      analogWrite(FL_FWD, value);
      analogWrite(FR_FWD, value);
      analogWrite(BL_FWD, value);
      analogWrite(BR_FWD, value);
      break;
    case LEFT:
      analogWrite(FL_REV, value);
      analogWrite(FR_FWD, value);
      analogWrite(BL_FWD, value);
      analogWrite(BR_REV, value);
      break;
    case RIGHT:
      analogWrite(FL_FWD, value);
      analogWrite(FR_REV, value);
      analogWrite(BL_REV, value);
      analogWrite(BR_FWD, value);
      break;
    case REV:
      analogWrite(FL_REV, value);
      analogWrite(FR_REV, value);
      analogWrite(BL_REV, value);
      analogWrite(BR_REV, value);
      break;
  }
}

void escape() {
  double y0 = Enes100.getY();
  double distance_traveled = Enes100.getY() - y0;
  int clear_count = 0;
  
  // Travel left while still blocked and traveled less than 65cm
  // Turn motor on and move left
  motor_on_off(HIGH);
  direction = LEFT;
  set_motor_speed(FULL_SPEED);

  while (clear_count < 3 && distance_traveled < 0.65) {
    update_distance();

    // If its clear, update counter (3 readings in a row for safety)
    if (distance > 10 || distance == 0)
      clear_count++;
    else
      clear_count = 0;

    // Update distance_traveled
    distance_traveled = abs(Enes100.getY() - y0);
  }

  motor_on_off(LOW);

  // If obstacle not cleared, go right until the obstacle is cleared
  if (distance <= 5) {
    // Turn motor on and move right
    motor_on_off(HIGH);
    direction = RIGHT;
    set_motor_speed(FULL_SPEED);

    while (distance <= 5) {
      update_distance();
    }
  }

  motor_on_off(LOW);
  distance_traveled = 0;
  y0 = Enes100.getY();

  // Turn motor on and clear obstacle
  motor_on_off(HIGH);
  set_motor_speed(CRAWL_SPEED);
  while (distance_traveled < 0.10) {
    distance_traveled = abs(Enes100.getY() - y0);
  }
  
  direction = FWD;
}

void do_mission() {
  // Check which landing zone the OTV is and call  nav_to_mission with direction of mission
  if (Enes100.getY() < 1.0) 
    nav_to_mission(UP);
  else
    nav_to_mission(DOWN);

  // Plant seed and grab rocks
  // plant_seed();
  // grab_rocks();

  // Classify back pots and planting pattern
  classify_pots();

  // Move back 10 cm
  int y0 = Enes100.getY();
  int distance_traveled = 0;

  // Reverse at crawl speed
  direction = REV;
  motor_on_off(HIGH);
  set_motor_speed(CRAWL_SPEED);

  while (distance_traveled < 0.10)
    distance_traveled = abs(Enes100.getY() - y0);

  // Stop, turn to theta = 0 and set direction to FWD
  motor_on_off(LOW);
  turn_to_angle(0);
  direction = FWD;

}

void nav_to_mission(int orientation) {
  double tolerance = 0.05;
  // Turn towards mission depending on direction
  if (orientation == UP) 
    turn_to_angle(PI / 2);
  else
    turn_to_angle(-PI / 2);

  // Go to site
  motor_on_off(HIGH);
  set_motor_speed(SLOW_SPEED);

  while (distance > 5) {
    update_distance();
  }

  // Stop OTV
  motor_on_off(LOW);

  // Check if distance is within tolerance
  if (abs(distance - 2) > tolerance) {

    if (distance - 2 < 0) {
      direction = REV;
      motor_on_off(HIGH);
      set_motor_speed(CRAWL_SPEED);

      while (distance < 2 - tolerance)
        update_distance();
    }
    else {
      direction = FWD;
      motor_on_off(HIGH);
      set_motor_speed(CRAWL_SPEED);

      while (distance > 2 + tolerance)
        update_distance();
    }

    motor_on_off(LOW);
    direction = FWD;
  }
  motor_on_off(LOW);
  direction = FWD;
}

void turn_to_angle(double angle) {
  double tolerance = 0.05;
  double currentTheta = Enes100.getTheta();
  // Set all motors to zero
  analogWrite(FL_FWD, 0);
  analogWrite(FR_FWD, 0);
  analogWrite(BL_FWD, 0);
  analogWrite(BR_FWD, 0);
  analogWrite(FL_REV, 0);
  analogWrite(FR_REV, 0);
  analogWrite(BL_REV, 0);
  analogWrite(BR_REV, 0);

  motor_on_off(HIGH);

  // Turn until OTV has an angle of angle
  if (angle - currentTheta > 0) {
    analogWrite(FL_FWD, SLOW_SPEED);
    analogWrite(BL_FWD, SLOW_SPEED);
    analogWrite(FR_REV, SLOW_SPEED);
    analogWrite(BR_REV, SLOW_SPEED);
  }
  else {
    analogWrite(FL_REV, SLOW_SPEED);
    analogWrite(BL_REV, SLOW_SPEED);
    analogWrite(FR_FWD, SLOW_SPEED);
    analogWrite(BR_FWD, SLOW_SPEED);
  }
  while (abs(currentTheta - angle) > tolerance) {
    currentTheta = Enes100.getTheta();
  }

  // Turn off motors
  motor_on_off(LOW);

}

void update_distance() {
  int duration;
  // Set TRIG to LOW to reset
  digitalWrite(TRIG, LOW);
  delayMicroseconds(10); // 
  digitalWrite(TRIG, HIGH); //set the pin to HIGH to send the sound wave
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW); //set the pin to LOW to turn it off
  delayMicroseconds(10);

  duration = pulseIn(ECHO, HIGH); 

  distance = (duration*.0343)/2;
}

void classify_pots() {
  int distance_traveled = 0;
  int left_pot_dist = 0.25;
  int right_pot_dist = 0.50;
  int plot_layout;

  // Move to capture pot 1
  int x0 = Enes100.getX();

  motor_on_off(HIGH);
  set_motor_speed(CRAWL_SPEED);

  while (Enes100.getX() - x0 > left_pot_dist)
    distance_traveled = Enes100.getX() - x0;
  
  motor_on_off(LOW);  

  // Capture pot 1
  int left_classification = Enes100.MLGetPrediction(0);

  // Move to capture pot 2
  x0 = Enes100.getX();

  motor_on_off(HIGH);
  set_motor_speed(CRAWL_SPEED);

  while (Enes100.getX() - x0 > right_pot_dist)
    distance_traveled = Enes100.getX() - x0;

  // Capture pot 2
  int right_classification = Enes100.MLGetPrediction(0);

  // Determine pot arrangement (Make sure to match enum with model)
  if (left_classification == ROCKS && right_classification == ROCKS)
    plot_layout = NEITHER;
  else if (left_classification == ROCKS && right_classification == ORZO)
    plot_layout = DIAGONAL;
  else if (left_classification == ORZO && right_classification == ROCKS)
    plot_layout = ADJACENT;
  else
    plot_layout = BOTH;

  Enes100.mission(LOCATION, plot_layout);
}

// Demos OTVs ability to go forwards
void demo_forward() {
  int x;
  
  motor_on_off(HIGH);
  set_motor_speed(FULL_SPEED);
  while (x < 3.4)
    x = Enes100.getX();

  motor_on_off(LOW);
}

// Demos OTVs ability to make 3 consecutive 90 degree turns
void demo_turning() {
  turn_to_angle(Enes100.getTheta() + PI/2);
  turn_to_angle(Enes100.getTheta() + PI/2);
  turn_to_angle(Enes100.getTheta() + PI/2);
}

// Demos OTVs ability to receive current coordinates and heading information
void  demo_wireless_rx() {
  Enes100.getX();
  Enes100.getY();
  Enes100.getTheta();
}

// Demos OTVs ability to send mission messages via wireless communication
void dem_wireless_tx() {
  Enes100.mission(LOCATION, BOTH);
}









