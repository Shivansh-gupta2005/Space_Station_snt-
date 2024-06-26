#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 6
#define IN2 7

// Globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float eintegral = 0;
float prevError = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // Read the position in an atomic block to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1 / 420.0 * 60.0;
  //float v2 = velocity2 / 420.0 * 60.0;

  // Set a target
  float vt = 30;

  // Compute the control signal u
  float kp = 9;
  float ki = 5;
  float kd = 0.1; // Add derivative gain

  float error = vt - v1;
  eintegral = eintegral + error * deltaT;
  float derivative = (error - prevError) / deltaT;
  
  float u = kp * error + ki * eintegral + kd * derivative;
  prevError = error;

  // Set the motor speed and direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  int pwr = (int)fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  setMotor(dir, pwr, PWM, IN1, IN2);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1);
  Serial.println();
  delay(100);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  if (dir == -1) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == 1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Or don't turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
  } else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}