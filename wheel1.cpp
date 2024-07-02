#include <CytronMotorDriver.h>
#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5
#define DIR 6

CytronMD motor(PWM_DIR, PWM, DIR);  // PWM = 5, DIR = 6

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
  float v1 = velocity1 / 375.0 * 60.0;
  //float v2 = velocity2 / 420.0 * 60.0;

  // Set a target
  float vt = 90;

  // Compute the control signal u
  float kp = 0.075;
  float ki = 0.0;
  float kd = 0.0; // Add derivative gain
  
  float error = vt - v1;
  eintegral = eintegral + error * deltaT;
  float derivative = (error - prevError) / deltaT;
  
  
  float u = kp * error + ki * eintegral + kd * derivative ;
  prevError = error;

  // Set the motor speed and direction
  int pwr = (int)(vt/0.4 + u);
  if (pwr > 255) {
    pwr = 255;
  }




  
  motor.setSpeed(pwr);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.println();
  delay(100);
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