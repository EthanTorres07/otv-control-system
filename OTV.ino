#include <Enes100.h>

// =========================
// ENES100 INFO
// =========================
const int MARKER_ID = 342;
const int RM_NUM = 1120;

const int ESP_RX = 50;
const int ESP_TX = 52;

// =========================
// ULTRASONIC SENSOR
// =========================
const int TRIG = 37;
const int ECHO = 36;

double distance;

// =========================
// DRIVE MOTOR PINS
// =========================
int FL_IN1 = 23;
int FL_IN2 = 22;
int BL_IN3 = 31;
int BL_IN4 = 30;
int FL_ENA = 2;
int BL_ENB = 3;

int BR_IN1 = 46;
int BR_IN2 = 47;
int FR_IN3 = 38;
int FR_IN4 = 39;
int BR_ENA = 5;
int FR_ENB = 4;

// =========================
// MISSION MOTOR PINS
// =========================
int PLT_IN1 = 34;
int PLT_IN2 = 35;
int RK_IN3 = 26;
int RK_IN4 = 27;
int PLT_ENA = 6;
int RK_ENB = 7;

// =========================
// SPEED SETTINGS
// =========================
const int STOP_DIST = 20;          // cm, used for normal obstacle avoidance
const int MISSION_STOP_DIST = 5;   // cm = 20 mm from mission site
const int MISSION_CLOSE_READINGS_NEEDED = 1; // stop after this many close readings
const unsigned long MISSION_MAX_DRIVE_MS = 8000; // safety stop so it cannot drive forever

const int DRIVE_SPEED = 100;
const int STRAFE_SPEED = 100;
const int MISSION_SPEED = 80;
const int TURN_SPEED = 75;

const int EXTRA_CLEAR_TIME = 850; // ms

const int NAV_SIDE_STEP_TIME = 250;   // 0.25 seconds
const int NAV_CHECK_DELAY = 350;      // short pause to update/check coordinates

// =========================
// ARENA / ROBOT SETTINGS
// =========================
// Y = 0 is RIGHT wall
// Y = 2 is LEFT wall
// X = 0 is back of arena
// X = 4 is front of arena
const double RIGHT_WALL_Y = 0.0;
const double LEFT_WALL_Y = 2.0;

const double ROBOT_HALF_WIDTH = 0.2; // robot is about 0.3m wide
const double SAFE_RIGHT_Y = RIGHT_WALL_Y + ROBOT_HALF_WIDTH;
const double SAFE_LEFT_Y = LEFT_WALL_Y - ROBOT_HALF_WIDTH;

// If robot is around y=1.5 or greater, it is too far left
const double TOO_FAR_LEFT_Y = 1.50;

const double ANGLE_TOLERANCE = 0.04; // radians
const double MISSION_BACK_TOLERANCE = 0.03; // meters
const double TURN_STEP_RAD = 0.01; // radians, turn only about this much before checking theta again
const unsigned long MAX_TURN_PULSE_MS = 80; // safety cap for each small turn pulse

// =========================
// MISSION X ALIGNMENT SETTINGS
// =========================
const double MISSION_X_POS_PI_OVER_2 = 0.221;  // when facing +PI/2
const double MISSION_X_NEG_PI_OVER_2 = 0.372;  // when facing -PI/2

// =========================
// CHECKPOINT SETTINGS
// =========================
const bool USE_CHECKPOINT = true;

const double CHECKPOINT_START_X = 2.85;   // when checkpoint begins
const double CHECKPOINT_TARGET_Y = 1.60;   // final Y position
const double CHECKPOINT_END_X = 3.58;     // final X position

const double Y_TOLERANCE = 0.04;

enum DIRECTIONS {FWD, LEFT, RIGHT, REV};
enum SITES {UP, DOWN};

bool missionComplete = false;

// =========================
// SETUP
// =========================
void setup() {
  enable_all_pins();
  stop_motors();
  stop_mission_motors();

  Enes100.begin("SEEDLINGS", SEED, MARKER_ID, RM_NUM, ESP_TX, ESP_RX);

  // Do not move or run the mission until the vision system is connected.
  while (!Enes100.isConnected()) {
    stop_motors();
    stop_mission_motors();
    delay(100);
  }

  delay(500);
}

// =========================
// MAIN LOOP
// =========================
void loop() {
  // Run the mission once immediately after startup/connection.
  if (!missionComplete) {
    do_mission();
    missionComplete = true;
  }

  update_distance();

  double x = Enes100.getX();
  double y = Enes100.getY();

  Enes100.print("X: ");
  Enes100.print(x);
  Enes100.print("  Y: ");
  Enes100.print(y);
  Enes100.print("  Theta: ");
  Enes100.print(Enes100.getTheta());
  Enes100.print("  Distance: ");
  Enes100.println(distance);

  // =========================
  // FINAL CHECKPOINT MISSION
  // =========================
  if (USE_CHECKPOINT && x >= CHECKPOINT_START_X) {
    stop_motors();
    go_to_y_then_stop_at_x(CHECKPOINT_TARGET_Y, CHECKPOINT_END_X);
  }

  // =========================
  // NORMAL OBSTACLE AVOIDANCE
  // =========================
  if (obstacle_close()) {
    stop_motors();
    delay(200);

    avoid_obstacle();

    stop_motors();
    delay(200);

    // After obstacle is avoided, re-face forward
    turn_to_angle_precise(0.0);

    stop_motors();
    delay(200);
  }
  else {
    set_motor_speed(DRIVE_SPEED, FWD);
  }

  delay(50);
}

// =========================
// STARTUP MISSION
// =========================
void do_mission() {
  double startY = Enes100.getY();

  if (startY < 1.0) {
    nav_to_mission(UP);
  }
  else {
    nav_to_mission(DOWN);
  }

  stop_motors();
  delay(300);

  plant_seeds();
  grab_rocks();
  delay(1000);
  int result = Enes100.MLGetPrediction(2);
  delay(1000);
  result = Enes100.MLGetPrediction(2);
  delay(1000);
  result = Enes100.MLGetPrediction(2);
  delay(8000);
  String labels[] = {"BOTH", "ADJACENT", "DIAGONAL", "NEITHER"};

  if (result >= 0 && result <= 3) {
      Enes100.println("Results are in! ML predicts: " + labels[result]);
  } else {
      Enes100.println("Results are in! ML returned an invalid value: " + String(result));
  }
  
  delay(1000);;

  // After the mission, do NOT try to return to the original X/Y coordinates.
  // Just back away from the mission site for 3.35 seconds.
  set_motor_speed(MISSION_SPEED, REV);
  delay(4000);

  digitalWrite(RK_IN3, LOW);
  digitalWrite(RK_IN4, HIGH);
  analogWrite(RK_ENB, 100);
  delay(2000);

  digitalWrite(RK_IN3, LOW);
  digitalWrite(RK_IN4, LOW);
  analogWrite(RK_ENB, 0);

  stop_motors();
  delay(300);

  // Face forward before running the normal navigation code.
  turn_to_angle_precise(0.0);
  stop_motors();
  delay(300);
}

void nav_to_mission(int orientation) {
  double targetAngle;

  if (orientation == UP) {
    // Starting near y = 0.5. Mission is toward larger y, so face left.
    targetAngle = PI / 2.0;
  }
  else {
    // Starting near y = 1.5. Mission is toward smaller y, so face right.
    targetAngle = -PI / 2.0;
  }

  // Turn toward the mission using small checked pulses so it does not overshoot.
  turn_to_angle_precise(targetAngle);
  align_to_x_mis(targetAngle);
  
  // Move forward for 1.5 seconds after X alignment
  set_motor_speed(MISSION_SPEED, FWD);
  delay(2500);
  stop_motors();
  delay(150);

  // Re-align to the mission angle again
  turn_to_angle_precise(targetAngle);

  update_distance();

  // Drive straight toward the mission. Do NOT keep turning during this part,
  // because small theta noise from the vision system can make the robot turn again for no reason.
  int closeReadings = 0;
  unsigned long missionDriveStart = millis();

  while (true) {
    update_distance();

    // Stop if the ultrasonic sensor confirms the mission wall/site is close.
    if (distance > 0 && distance <= MISSION_STOP_DIST) {
      closeReadings++;
    }
    else {
      closeReadings = 0;
    }

    if (closeReadings >= MISSION_CLOSE_READINGS_NEEDED) {
      break;
    }

    // Backup safety using the vision Y coordinate, in case the ultrasonic sensor misses
    // the mission site or returns 999 at very close range.
    if (orientation == UP && Enes100.getY() >= SAFE_LEFT_Y) {
      break;
    }

    if (orientation == DOWN && Enes100.getY() <= SAFE_RIGHT_Y) {
      break;
    }

    // Time safety so the robot cannot drive forever if both sensors fail.
    if (millis() - missionDriveStart > MISSION_MAX_DRIVE_MS) {
      break;
    }

    // Slow down as it approaches the mission site.
    if (distance > 6) {
      set_motor_speed(MISSION_SPEED, FWD);

    }
    else if (distance > 3) {
      set_motor_speed(85, FWD);
    }
    else {
      set_motor_speed(70, FWD);
    }

    delay(20);
  }

  stop_motors();
  delay(500);
}

void turn_to_angle_precise(double targetAngle) {
  // Pulse-turn instead of holding the motors on continuously.
  // Each pulse aims to change theta by only about TURN_STEP_RAD, then it stops and rechecks.
  while (abs(angle_error(targetAngle)) > ANGLE_TOLERANCE) {
    double error = angle_error(targetAngle);
    double startTheta = Enes100.getTheta();
    unsigned long pulseStart = millis();

    // If current theta is greater than target, rotate clockwise to reduce theta.
    // If current theta is less than target, rotate counterclockwise to increase theta.
    if (error > 0) {
      turn_clockwise(TURN_SPEED);
    }
    else {
      turn_counterclockwise(TURN_SPEED);
    }

    // Keep this turn pulse small: stop after about 0.05 rad of rotation,
    // or sooner if the robot reaches the target tolerance.
    while (abs(angle_difference(Enes100.getTheta(), startTheta)) < TURN_STEP_RAD &&
           abs(angle_error(targetAngle)) > ANGLE_TOLERANCE &&
           millis() - pulseStart < MAX_TURN_PULSE_MS) {
      delay(10);
    }

    stop_motors();
    delay(500);
  }

  stop_motors();
}

// Makes sure Robot is at x=0.5 before moving on to mission
void align_to_x_mis(double targetAngle) {
  double TOL;
  if (targetAngle > 0) {
    TOL = 0.002;          // facing +PI/2
  } else {
    TOL = 0.004;          // facing -PI/2 (0.002 higher)
  }

  double targetX;
  if (targetAngle > 0) {
    targetX = MISSION_X_POS_PI_OVER_2;
  } 
  else {
    targetX = MISSION_X_NEG_PI_OVER_2;
  }

  while (abs(Enes100.getX() - targetX) > TOL) {

    if (abs(angle_error(targetAngle)) > ANGLE_TOLERANCE) {
      stop_motors();
      delay(100);
      turn_to_angle_precise(targetAngle);
    }

    double currentX = Enes100.getX();

    int dir;

    if (targetAngle > 0) {
      dir = (currentX < targetX) ? RIGHT : LEFT;
    } 
    else {
      dir = (currentX < targetX) ? LEFT : RIGHT;
    }

    set_motor_speed(STRAFE_SPEED, dir);
    delay(100);
    stop_motors();
    delay(450);
  }

  delay(80);
  turn_to_angle_precise(targetAngle);

  stop_motors();
  delay(200);
}

// Returns current theta minus target theta, normalized to -PI through PI.
double angle_error(double targetAngle) {
  double error = Enes100.getTheta() - targetAngle;

  while (error > PI) {
    error -= 2.0 * PI;
  }

  while (error < -PI) {
    error += 2.0 * PI;
  }

  return error;
}

// Returns angle a minus angle b, normalized to -PI through PI.
double angle_difference(double a, double b) {
  double diff = a - b;

  while (diff > PI) {
    diff -= 2.0 * PI;
  }

  while (diff < -PI) {
    diff += 2.0 * PI;
  }

  return diff;
}

void turn_clockwise(int speed) {
  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, LOW);

  digitalWrite(BL_IN3, HIGH);
  digitalWrite(BL_IN4, LOW);

  digitalWrite(BR_IN1, LOW);
  digitalWrite(BR_IN2, HIGH);

  digitalWrite(FR_IN3, LOW);
  digitalWrite(FR_IN4, HIGH);

  analogWrite(FL_ENA, speed);
  analogWrite(BL_ENB, speed);
  analogWrite(BR_ENA, speed);
  analogWrite(FR_ENB, speed + 40);
}

void turn_counterclockwise(int speed) {
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, HIGH);

  digitalWrite(BL_IN3, LOW);
  digitalWrite(BL_IN4, HIGH);

  digitalWrite(BR_IN1, HIGH);
  digitalWrite(BR_IN2, LOW);

  digitalWrite(FR_IN3, HIGH);
  digitalWrite(FR_IN4, LOW);

  analogWrite(FL_ENA, speed);
  analogWrite(BL_ENB, speed);
  analogWrite(BR_ENA, speed);
  analogWrite(FR_ENB, speed + 40);
}

void plant_seeds() {
  digitalWrite(PLT_IN1, HIGH);
  digitalWrite(PLT_IN2, LOW);
  analogWrite(PLT_ENA, 100);
  delay(9500);

  digitalWrite(PLT_IN1, LOW);
  digitalWrite(PLT_IN2, LOW);
  analogWrite(PLT_ENA, 0);
}

void grab_rocks() {
  digitalWrite(RK_IN3, LOW);
  digitalWrite(RK_IN4, HIGH);
  analogWrite(RK_ENB, 100);
  delay(5500);

  digitalWrite(RK_IN3, HIGH);
  digitalWrite(RK_IN4, LOW);
  delay(4000);

  digitalWrite(RK_IN3, LOW);
  digitalWrite(RK_IN4, LOW);
  analogWrite(RK_ENB, 0);
}

void stop_mission_motors() {
  digitalWrite(PLT_IN1, LOW);
  digitalWrite(PLT_IN2, LOW);
  digitalWrite(RK_IN3, LOW);
  digitalWrite(RK_IN4, LOW);
  analogWrite(PLT_ENA, 0);
  analogWrite(RK_ENB, 0);
}

// =========================
// FINAL CHECKPOINT FUNCTION
// =========================
void go_to_y_then_stop_at_x(double targetY, double endX) {

  delay(300);
  // Step 1: Move sideways to target Y
  while (abs(Enes100.getY() - targetY) > Y_TOLERANCE) {
    double currentY = Enes100.getY();

    if (currentY < targetY) {
      // Need bigger Y, so move LEFT
      set_motor_speed(STRAFE_SPEED, LEFT);
      delay(NAV_SIDE_STEP_TIME);
      stop_motors();
      delay(NAV_CHECK_DELAY);
    }
    else {
      // Need smaller Y, so move RIGHT
      set_motor_speed(STRAFE_SPEED, RIGHT);
      delay(NAV_SIDE_STEP_TIME);
      stop_motors();
      delay(NAV_CHECK_DELAY);
    }

    delay(30);
  }

  stop_motors();
  delay(200);

  // Step 2: Move forward until final X
  turn_to_angle_precise(0.0);
  while (Enes100.getX() < endX) {
    set_motor_speed(DRIVE_SPEED, FWD);
    delay(30);
  }

  // Step 3: Mission complete — stop forever
  stop_motors();

  while (true) {
    // stay stopped forever
  }
}

// =========================
// OBSTACLE AVOIDANCE
// =========================
void avoid_obstacle() {
  double y = Enes100.getY();
  int avoidDirection;

  // If too far left, move RIGHT
  if (y >= TOO_FAR_LEFT_Y) {
    avoidDirection = RIGHT;
  }
  // If too close to right wall, move LEFT
  else if (y <= SAFE_RIGHT_Y) {
    avoidDirection = LEFT;
  }
  // Otherwise, default is move LEFT
  else {
    avoidDirection = LEFT;
  }

  set_motor_speed(STRAFE_SPEED, avoidDirection);
  delay(NAV_SIDE_STEP_TIME);
  stop_motors();
  delay(NAV_CHECK_DELAY);

  int clearCount = 0;

  // Move sideways until obstacle is gone for 3 readings
  while (clearCount < 2) {
    update_distance();

    if (!obstacle_close()) {
      clearCount++;
    }
    else {
      clearCount = 0;
    }

    // Wall protection
    if (avoidDirection == LEFT && Enes100.getY() >= SAFE_LEFT_Y) {
      avoidDirection = RIGHT;
      set_motor_speed(STRAFE_SPEED, RIGHT);
      clearCount = 0;
    }

    // if (avoidDirection == RIGHT && Enes100.getY() <= SAFE_RIGHT_Y) {
    //   avoidDirection = LEFT;
    //   set_motor_speed(STRAFE_SPEED, LEFT);
    //   clearCount = 0;
    // }

    set_motor_speed(STRAFE_SPEED, avoidDirection);
    delay(NAV_SIDE_STEP_TIME);
    stop_motors();
    delay(NAV_CHECK_DELAY);
  }

  // Extra clearance after obstacle disappears
  set_motor_speed(STRAFE_SPEED, avoidDirection);
  delay(EXTRA_CLEAR_TIME);

  stop_motors();
}

// =========================
// OBSTACLE CHECK
// =========================
bool obstacle_close() {
  return (distance > 0 && distance <= STOP_DIST);
}

// =========================
// MOTOR CONTROL
// =========================
void set_motor_speed(int value, int dir) {
  switch (dir) {
    case FWD:
      digitalWrite(FL_IN1, HIGH);
      digitalWrite(FL_IN2, LOW);

      digitalWrite(BL_IN3, HIGH);
      digitalWrite(BL_IN4, LOW);

      digitalWrite(BR_IN1, HIGH);
      digitalWrite(BR_IN2, LOW);

      digitalWrite(FR_IN3, HIGH);
      digitalWrite(FR_IN4, LOW);
      break;

    case LEFT:
      digitalWrite(FL_IN1, LOW);
      digitalWrite(FL_IN2, HIGH);

      digitalWrite(BL_IN3, HIGH);
      digitalWrite(BL_IN4, LOW);

      digitalWrite(BR_IN1, LOW);
      digitalWrite(BR_IN2, HIGH);

      digitalWrite(FR_IN3, HIGH);
      digitalWrite(FR_IN4, LOW);
      break;

    case RIGHT:
      digitalWrite(FL_IN1, HIGH);
      digitalWrite(FL_IN2, LOW);

      digitalWrite(BL_IN3, LOW);
      digitalWrite(BL_IN4, HIGH);

      digitalWrite(BR_IN1, HIGH);
      digitalWrite(BR_IN2, LOW);

      digitalWrite(FR_IN3, LOW);
      digitalWrite(FR_IN4, HIGH);
      break;

    case REV:
      digitalWrite(FL_IN1, LOW);
      digitalWrite(FL_IN2, HIGH);

      digitalWrite(BL_IN3, LOW);
      digitalWrite(BL_IN4, HIGH);

      digitalWrite(BR_IN1, LOW);
      digitalWrite(BR_IN2, HIGH);

      digitalWrite(FR_IN3, LOW);
      digitalWrite(FR_IN4, HIGH);
      break;
  }

  analogWrite(FL_ENA, value);
  analogWrite(BL_ENB, value);
  analogWrite(BR_ENA, value);
  analogWrite(FR_ENB, value+33);
}

// =========================
// STOP MOTORS
// =========================
void stop_motors() {
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);

  digitalWrite(BL_IN3, LOW);
  digitalWrite(BL_IN4, LOW);

  digitalWrite(BR_IN1, LOW);
  digitalWrite(BR_IN2, LOW);

  digitalWrite(FR_IN3, LOW);
  digitalWrite(FR_IN4, LOW);

  analogWrite(FL_ENA, 0);
  analogWrite(BL_ENB, 0);
  analogWrite(BR_ENA, 0);
  analogWrite(FR_ENB, 0);
}

// =========================
// ULTRASONIC FUNCTION
// =========================
void update_distance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(5);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);

  if (duration == 0) {
    distance = 999;
  }
  else {
    distance = (duration * 0.0343) / 2.0;
  }
}

// =========================
// HELPERS
// =========================
double distance_between(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2;
  double dy = y1 - y2;
  return sqrt(dx * dx + dy * dy);
}

// =========================
// PIN SETUP
// =========================
void enable_all_pins() {
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(BL_IN3, OUTPUT);
  pinMode(BL_IN4, OUTPUT);
  pinMode(FL_ENA, OUTPUT);
  pinMode(BL_ENB, OUTPUT);

  pinMode(BR_IN1, OUTPUT);
  pinMode(BR_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT);
  pinMode(FR_IN4, OUTPUT);
  pinMode(BR_ENA, OUTPUT);
  pinMode(FR_ENB, OUTPUT);

  pinMode(PLT_IN1, OUTPUT);
  pinMode(PLT_IN2, OUTPUT);
  pinMode(RK_IN3, OUTPUT);
  pinMode(RK_IN4, OUTPUT);
  pinMode(PLT_ENA, OUTPUT);
  pinMode(RK_ENB, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}
