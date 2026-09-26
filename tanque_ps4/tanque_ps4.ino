#include <Bluepad32.h>

#define LED_BUILTIN 2

// PWM properties
static const int PwmFreq = 20000;
static const int PwmCh1  = 0;
static const int PwmCh2  = 1;
static const int PwmRes  = 8;   // 8-bit -> 0..255

#define m1PWM 25
#define m1CW 26
#define m1CCW 27
#define m2PWM 21
#define m2CW 18
#define m2CCW 19

// deadzone do analogico
static const int AxisDeadzone = 64;

// velocidade maxima e minima do motor
static const int MotorMinPwm = 170;
static const int MotorMaxPwm = 255;

//aceleraçao pra velocidade de frente/ré
static const float MaxAccelPerSec = 3.0f;   // "speed units" (of the -1..1 range) per second
static float    currentSpeed     = 0.0f;
static uint32_t lastSpeedUpdate  = 0;

// Bluepad32 tracks up to 4 pads at once. We only drive from one
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
static ControllerPtr myController = nullptr;

// Rescales a signed stick axis (-511..512) to a 0..511 magnitude,
static int rescaleAxis(int v) {
    int a = (v >= 0) ? v : -v;
    if (a > 511) a = 511;
    return (a > AxisDeadzone) ? ((a - AxisDeadzone) * 511 / (511 - AxisDeadzone)) : 0;
}

// LEFT stick Y -> throttle, normalized -1..+1. Up (raw < 0) = forward (+).
static float axisThrottle(int rawY) {
    int mag = rescaleAxis(rawY);
    if (mag == 0) return 0.0f;
    float n = (float)mag / 511.0f;
    return (rawY < 0) ? n : -n;
}

// RIGHT stick X -> turn, normalized -1..+1.
static float axisTurn(int rawX) {
    int mag = rescaleAxis(rawX);
    if (mag == 0) return 0.0f;
    float n = (float)mag / 511.0f;
    return (rawX > 0) ? -n : n;
}

// Para os dirs: 1 é frente, 0 é trás
// Sets direction pins for one motor.
static void setMotorDirection(int cwPin, int ccwPin, bool forward) {
    if (forward) {
        digitalWrite(cwPin, LOW);
        digitalWrite(ccwPin, HIGH);
    } else {
        digitalWrite(cwPin, HIGH);
        digitalWrite(ccwPin, LOW);
    }
}

// Drives one wheel from a normalized -1..+1 command: sign picks direction,
// magnitude maps linearly to a PWM 
static void driveWheel(int pwmChannel, int cwPin, int ccwPin, float value) {
    bool forward = (value >= 0.0f);
    float mag = fabsf(value);
    int pwm = 0;
    if (mag > 0.0f) {
        pwm = (int)(MotorMinPwm + mag * (MotorMaxPwm - MotorMinPwm));
    }
    setMotorDirection(cwPin, ccwPin, forward);
    ledcWrite(pwmChannel, pwm);
}

// GTA-style tank drive: LEFT stick Y is an acceleration input that ramps a
// persistent currentSpeed, RIGHT stick X is a differential offset applied
// with opposite sign to each wheel.
static void driveMotors() {
    uint32_t now = millis();
    float dt = (now - lastSpeedUpdate) / 1000.0f;
    lastSpeedUpdate = now;

    // Target speed = what the stick is asking for; currentSpeed chases it
    // at a limited rate instead of jumping straight there.
    float targetSpeed = axisThrottle(myController->axisY());
    float maxStep      = MaxAccelPerSec * dt;
    if (currentSpeed < targetSpeed) {
        currentSpeed = fminf(currentSpeed + maxStep, targetSpeed);
    } else if (currentSpeed > targetSpeed) {
        currentSpeed = fmaxf(currentSpeed - maxStep, targetSpeed);
    }

    float turn = axisTurn(myController->axisRX());

    float left  = currentSpeed + turn;
    float right = currentSpeed - turn;

    // Preserve the turn ratio if the combination would exceed +-1 (e.g.
    // full speed + full turn), instead of hard-clipping one side.
    float maxMag = fmaxf(fabsf(left), fabsf(right));
    if (maxMag > 1.0f) {
        left  /= maxMag;
        right /= maxMag;
    }

    driveWheel(PwmCh1, m1CW, m1CCW, left);    // motor 1 = left wheel
    driveWheel(PwmCh2, m2CW, m2CCW, right);   // motor 2 = right wheel
}

// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("[BT  ] controller connected, index=%d, model=%s\n",
                          i, ctl->getModelName().c_str());
            myControllers[i] = ctl;
            foundEmptySlot = true;

            if (myController == nullptr) {
                myController   = ctl;
                currentSpeed   = 0.0f;      // start from a stop on (re)connect
                lastSpeedUpdate = millis(); // avoid a huge dt on the first driveMotors() call
            }

            // Lights the controller up green the moment it connects.
            ctl->setColorLED(0, 255, 0);
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("[BT  ] controller connected, but no empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool found = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("[BT  ] controller disconnected, index=%d\n", i);
            myControllers[i] = nullptr;
            found = true;
            break;
        }
    }
    if (myController == ctl) {
        myController = nullptr;
        currentSpeed = 0.0f;   // stop the robot if the driving pad drops
    }
    if (!found) {
        Serial.println("[BT  ] controller disconnected, but not found in list");
    }
}

/* setup */
void setup() {
    // LED
    pinMode(LED_BUILTIN, OUTPUT);

    // PWM
    pinMode(m1PWM, OUTPUT);
    pinMode(m2PWM, OUTPUT);
    pinMode(m1CW, OUTPUT);
    pinMode(m1CCW, OUTPUT);
    pinMode(m2CW, OUTPUT);
    pinMode(m2CCW, OUTPUT);

    // IMPORTANTE:
    // O robô começa parado, em modo FORWARD (direction pins default to
    // forward; PWM stays 0 until a stick is pushed)
    currentSpeed = 0.0f;
    lastSpeedUpdate = millis();
    setMotorDirection(m1CW, m1CCW, true);
    setMotorDirection(m2CW, m2CCW, true);

    // Serial
    Serial.begin(115200);
    Serial.println("Booting");
    Serial.printf("Bluepad32 firmware: %s\n", BP32.firmwareVersion());

    // PWM channels (legacy LEDC API: set up the channel, then attach a pin to it)
    ledcSetup(PwmCh1, PwmFreq, PwmRes);
    ledcAttachPin(m1PWM, PwmCh1);
    ledcSetup(PwmCh2, PwmFreq, PwmRes);
    ledcAttachPin(m2PWM, PwmCh2);

    // Bluepad32: register the connect/disconnect callbacks.
    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
    // Fetches all controllers' data; must be called every loop iteration.
    BP32.update();

    if (myController && myController->isConnected() && myController->hasData()) {
        driveMotors();
    }

    vTaskDelay(1);
}
