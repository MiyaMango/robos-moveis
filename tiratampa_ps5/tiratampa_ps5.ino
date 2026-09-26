/*definicoes*/
#include <ps5Controller.h>

#define LED_BUILTIN 2

// PWM properties
static const int PwmFreq = 20000;
static const int PwmCh1  = 0;
static const int PwmCh2  = 1;
static const int PwmRes  = 8;   // 8-bit -> 0..255

#define m1PWM 23
#define m1CW 21
#define m1CCW 18
#define m2PWM 22
#define m2CW 19
#define m2CCW 5

// deadzone do analogico
static const int AxisDeadzone = 16;

// velocidade maxima e minima do motor
static const int MotorMinPwm = 30;
static const int MotorMaxPwm = 120;

// exponencial pra curva logaritmica do controle
static const float MotorCurve = 3.0f;

//aceleraçao pra velocidade de frente/ré
static const float MaxAccelPerSec = 3.0f;   // "speed units" (of the -1..1 range) per second
static float    currentSpeed     = 0.0f;
static uint32_t lastSpeedUpdate  = 0;

// keep-alive: refresh lightbar periodically so the controller
// doesn't decide the link is idle and disconnect on its own
static const uint32_t LightbarIntervalMs = 1000;
static uint32_t lastLightbarUpdate = 0;

// Rescales a signed stick axis to a 0..127 magnitude, deadzone removed.
static int rescaleAxis(int8_t v) {
    int a = (v >= 0) ? v : -v;
    if (a > 127) a = 127;
    return (a > AxisDeadzone) ? ((a - AxisDeadzone) * 127 / (127 - AxisDeadzone)) : 0;
}

// LEFT stick Y -> throttle, normalized -1..+1. Up (raw < 0) = forward (+).
static float axisThrottle(int8_t rawY) {
    int mag = rescaleAxis(rawY);
    if (mag == 0) return 0.0f;
    float n = (float)mag / 127.0f;
    return (rawY < 0) ? n : -n;
}

// RIGHT stick X -> turn, normalized -1..+1. Right (raw > 0) = turn right (+).
static float axisTurn(int8_t rawX) {
    int mag = rescaleAxis(rawX);
    if (mag == 0) return 0.0f;
    float n = (float)mag / 127.0f;
    return (rawX > 0) ? n : -n;
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

// funcao de set light bar
static void setLightbar(uint8_t r, uint8_t g, uint8_t b) {
    ps5.output.r = r;
    ps5.output.g = g;
    ps5.output.b = b;
    ps5.send();
}

// dirige uma roda a partir de um valor entre -1 e 1
// sinal escolhe direção, magnitude passa pela função exponencial pra ficar + dirigivel
static void driveWheel(int pwmPin, int cwPin, int ccwPin, float value) {
    bool forward = (value >= 0.0f);
    float mag = fabsf(value);
    int pwm = 0;
    if (mag > 0.0f) {
        float curved = (exp(MotorCurve * mag) - 1.0f) /
                       (exp(MotorCurve) - 1.0f);
        pwm = (int)(MotorMinPwm + curved * (MotorMaxPwm - MotorMinPwm));
    }
    setMotorDirection(cwPin, ccwPin, forward);
    ledcWrite(pwmPin, pwm);
}

// controle de tanque estilo gta; analogico esquerdo faz velocidade frente/tras, 
//analogico direito aplica um offset pra girar
static void driveMotors() {
    uint32_t now = millis();
    float dt = (now - lastSpeedUpdate) / 1000.0f;
    lastSpeedUpdate = now;

    // Target speed = what the stick is asking for; currentSpeed chases it
    // at a limited rate instead of jumping straight there.
    float targetSpeed = axisThrottle(ps5.ly);
    float maxStep      = MaxAccelPerSec * dt;
    if (currentSpeed < targetSpeed) {
        currentSpeed = fminf(currentSpeed + maxStep, targetSpeed);
    } else if (currentSpeed > targetSpeed) {
        currentSpeed = fmaxf(currentSpeed - maxStep, targetSpeed);
    }

    float turn = axisTurn(ps5.rx);

    float left  = currentSpeed + turn;
    float right = currentSpeed - turn;

    // Preserve the turn ratio if the combination would exceed +-1 (e.g.
    // full speed + full turn), instead of hard-clipping one side.
    float maxMag = fmaxf(fabsf(left), fabsf(right));
    if (maxMag > 1.0f) {
        left  /= maxMag;
        right /= maxMag;
    }

    driveWheel(m1PWM, m1CW, m1CCW, left);    // motor 1 = left wheel
    driveWheel(m2PWM, m2CW, m2CCW, right);   // motor 2 = right wheel
}

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

    //coisas p/ velocidade frontal
    currentSpeed = 0.0f;
    lastSpeedUpdate = millis();

    //começar os 2 motores pra frente
    setMotorDirection(m1CW, m1CCW, true);
    setMotorDirection(m2CW, m2CCW, true);

    // Serial
    Serial.begin(115200);
    Serial.println("Booting");

    // PWM channels
    ledcAttachChannel(m1PWM, PwmFreq, PwmRes, PwmCh1);
    ledcAttachChannel(m2PWM, PwmFreq, PwmRes, PwmCh2);

    // PS5
    ps5.begin(30);
    setLightbar(0, 255, 0);
}

void loop() {

    if (ps5.isConnected()) {
        driveMotors();
        // resend the lightbar color once a second so the controller doesnt die
        uint32_t now = millis();
        if (now - lastLightbarUpdate >= LightbarIntervalMs) {
            lastLightbarUpdate = now;
            setLightbar(0, 255, 0);
        }
    }

    delay(10);
}
