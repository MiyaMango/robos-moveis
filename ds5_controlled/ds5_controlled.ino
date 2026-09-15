#include <ps5Controller.h>

// Motor A (left track)
static const int motor1Pin1 = 26;
static const int motor1Pin2 = 27;
static const int enable1Pin = 25;

// Motor B (right track)
static const int motor2Pin1 = 18;
static const int motor2Pin2 = 19;
static const int enable2Pin = 21;

// PWM properties
static const int PwmFreq = 20000;
static const int PwmCh1  = 0;
static const int PwmCh2  = 1;
static const int PwmRes  = 8;   // 8-bit -> 0..255, matches ps5.l2/r2 range

// deadzone for controller trigger
static const int TriggerThreshold = 10;

// Trigger -> PWM shaping: the motors don't move below ~170 PWM, so anything
// under the threshold is 0, and the rest of the trigger's travel is
// remapped onto the motor's real 170..255 operating range.

static const int MotorMinPwm      = 170;  // PWM below this stalls the motor
static const int MotorMaxPwm      = 255;
static int triggerToPwm(uint8_t raw) {
  if (raw <= TriggerThreshold) return 0;
  return map(raw, TriggerThreshold, 255, MotorMinPwm, MotorMaxPwm);
}

static void driveMotors() {
  ledcWrite(enable1Pin, triggerToPwm(ps5.l2));
  ledcWrite(enable2Pin, triggerToPwm(ps5.r2));
}

void setup() {
  //setup motor pin modes
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  digitalWrite(motor1Pin1, HIGH);   // left track: forward
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);   // right track: forward
  digitalWrite(motor2Pin2, LOW);

  ledcAttachChannel(enable1Pin, PwmFreq, PwmRes, PwmCh1);
  ledcAttachChannel(enable2Pin, PwmFreq, PwmRes, PwmCh2);

  ps5.begin(30);   // try pairing to a ds5 controller for 30 seconds
}

void loop() {
  if (!ps5.isConnected()) return;
  driveMotors();
}
