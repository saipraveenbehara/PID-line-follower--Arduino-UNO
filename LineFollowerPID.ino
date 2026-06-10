/*
 * ============================================================
 *  LINE-FOLLOWING ROBOT — ARDUINO PID CONTROLLER
 * ============================================================
 *  Created by Sai Praveen Behara
 *
 *  Description:
 *    A complete Arduino sketch for a line-following robot using
 *    an IR sensor array and a PID (Proportional-Integral-Derivative)
 *    controller to steer two DC motors via an L298N motor driver.
 *    The robot calculates a weighted position error from the sensor
 *    array, feeds it through the PID algorithm, and applies a
 *    differential correction to the left and right motor speeds,
 *    keeping the robot centered on the line.
 *
 *  Hardware:
 *    - Arduino Uno (or any ATmega328P-based board)
 *    - 5-channel IR sensor array (e.g., TCRT5000 x5, or QTRX-5A)
 *    - L298N dual H-bridge motor driver module
 *    - Two DC gear motors
 *    - 7.4V LiPo battery (or 2x 18650 cells)
 *    - Optional: push-button for start/stop, LED status indicator
 *
 *  Wiring Summary:
 *    IR Sensors  : Digital pins D2–D6 (sensor 0 = leftmost, 4 = rightmost)
 *    Motor Driver:
 *      ENA (Left  PWM speed)  -> D9  (Timer1, PWM)
 *      IN1 (Left  forward)    -> D7
 *      IN2 (Left  backward)   -> D8
 *      ENB (Right PWM speed)  -> D10 (Timer1, PWM)
 *      IN3 (Right forward)    -> D11
 *      IN4 (Right backward)   -> D12
 *    Optional start button    -> D13
 *
 *  PID Tuning Guide:
 *    1. Set KI and KD to 0. Increase KP until the robot
 *       oscillates around the line with a consistent amplitude.
 *    2. Increase KD to dampen oscillation until the robot tracks
 *       smoothly without overshoot.
 *    3. Add a small KI only if you notice a persistent steady-state
 *       offset (robot consistently drifts to one side).
 *
 *  Known Limitations / TODO:
 *    - Sensor calibration is hardcoded; add a calibration routine
 *      that sweeps the robot over the line at startup for
 *      auto-threshold detection.
 *    - Speed ramp-up at start is not implemented; add acceleration
 *      to avoid wheel slip on first launch.
 *    - No intersection detection; all sensors fully ON is treated
 *      as straight-ahead.
 *
 *  License: MIT
 * ============================================================
 */


// ============================================================
//  CALIBRATION & TUNING PARAMETERS  (adjust here only)
// ============================================================

// --- PID gains ---
// Start with KP = 20, KI = 0, KD = 5 and tune from there.
const float KP = 20.0;   // Proportional gain
const float KI = 0.0;    // Integral gain (keep 0 until KP/KD are set)
const float KD = 5.0;    // Derivative gain

// --- Base motor speed (0–255) ---
// Lower this if the robot overshoots on tight curves.
const int BASE_SPEED = 160;

// --- Maximum individual motor speed (0–255) ---
// Caps speed after PID correction is applied.
const int MAX_SPEED = 220;

// --- Minimum motor speed (0–255) ---
// Prevents motors from stalling at very low duty cycles.
const int MIN_SPEED = 50;

// --- IR sensor threshold ---
// ADC readings BELOW this value are considered "on the line"
// when using analog sensors. For digital sensors this is ignored.
const int IR_THRESHOLD = 500;

// --- PID integral windup limit ---
// Clamps the accumulated integral to prevent runaway.
const float INTEGRAL_LIMIT = 500.0;

// --- Loop delay (milliseconds) ---
// Controls how often the PID loop runs. 0 = as fast as possible.
const int LOOP_DELAY_MS = 0;


// ============================================================
//  PIN DEFINITIONS
// ============================================================

// IR sensor input pins (left to right: sensor[0] ... sensor[4])
const int SENSOR_PINS[5] = {2, 3, 4, 5, 6};
const int NUM_SENSORS    = 5;

// Left motor (L298N channel A)
const int PIN_ENA = 9;   // PWM speed control
const int PIN_IN1 = 7;   // Direction bit 1
const int PIN_IN2 = 8;   // Direction bit 2

// Right motor (L298N channel B)
const int PIN_ENB = 10;  // PWM speed control
const int PIN_IN3 = 11;  // Direction bit 1
const int PIN_IN4 = 12;  // Direction bit 2

// Optional start button (active LOW, internal pull-up enabled)
const int PIN_START_BTN = 13;


// ============================================================
//  SENSOR POSITION WEIGHTS
// ============================================================
// Each sensor is assigned a weight representing its lateral
// position. The weighted average of active sensors gives a
// position value ranging from -2 (hard left) to +2 (hard right),
// with 0 meaning perfectly centered on the line.
//
// weights[i] corresponds to SENSOR_PINS[i].
const int SENSOR_WEIGHTS[5] = {-2, -1, 0, 1, 2};


// ============================================================
//  RUNTIME STATE  (do not edit)
// ============================================================

float pidIntegral    = 0.0;
float pidLastError   = 0.0;
int   sensorValues[5];
bool  robotRunning   = false;


// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================

void    readSensors();
float   computeError();
float   computePID(float error);
void    applyMotorOutput(float pidOutput);
void    setLeftMotor(int speed);
void    setRightMotor(int speed);
void    stopMotors();
void    waitForStartButton();
void    debugPrint(float error, float output, int leftSpeed, int rightSpeed);


// ============================================================
//  SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // Configure sensor pins as inputs
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
    }

    // Configure motor driver pins as outputs
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    // Optional start button with internal pull-up
    pinMode(PIN_START_BTN, INPUT_PULLUP);

    stopMotors();

    Serial.println(F("Line-Following Robot | PID Controller"));
    Serial.println(F("-------------------------------------"));
    Serial.print(F("KP: ")); Serial.print(KP);
    Serial.print(F("  KI: ")); Serial.print(KI);
    Serial.print(F("  KD: ")); Serial.println(KD);
    Serial.print(F("Base speed: ")); Serial.println(BASE_SPEED);
    Serial.println(F("Press start button (D13) to begin..."));

    // Halt here until the operator presses the start button.
    // Remove this call if no button is wired.
    waitForStartButton();

    robotRunning = true;
    Serial.println(F("Robot started."));
}


// ============================================================
//  MAIN LOOP
// ============================================================

void loop()
{
    if (!robotRunning) {
        stopMotors();
        return;
    }

    // 1. Read all IR sensors
    readSensors();

    // 2. Compute weighted position error
    float error = computeError();

    // 3. Run PID algorithm
    float pidOutput = computePID(error);

    // 4. Translate PID output to individual motor speeds
    applyMotorOutput(pidOutput);

    // 5. Optional serial debug output
    // Uncomment the line below to stream live PID data to Serial Monitor.
    // WARNING: printing every loop significantly slows the control rate.
    // debugPrint(error, pidOutput, 0, 0);  // placeholder speed args

    if (LOOP_DELAY_MS > 0) {
        delay(LOOP_DELAY_MS);
    }
}


// ============================================================
//  SENSOR READING
// ============================================================

/*
 * readSensors()
 * Reads all IR sensor pins and stores binary values in sensorValues[].
 * 1 = sensor detects the line (dark surface / LOW digital output from
 *     most TCRT5000 modules when over black tape).
 * 0 = sensor detects the background (light surface).
 *
 * If using analog IR sensors, replace digitalRead() with analogRead()
 * and apply IR_THRESHOLD to convert to binary:
 *   sensorValues[i] = (analogRead(SENSOR_PINS[i]) < IR_THRESHOLD) ? 1 : 0;
 */
void readSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        // For digital sensors: LOW = on line, HIGH = off line
        sensorValues[i] = (digitalRead(SENSOR_PINS[i]) == LOW) ? 1 : 0;
    }
}


// ============================================================
//  ERROR COMPUTATION
// ============================================================

/*
 * computeError()
 * Calculates a weighted average position of the line relative to
 * the sensor array center.
 *
 * Returns:
 *   A float in the range [-2.0, +2.0].
 *   Negative = line is to the LEFT  -> steer left  (reduce left motor)
 *   Zero     = line is centered     -> drive straight
 *   Positive = line is to the RIGHT -> steer right (reduce right motor)
 *
 * Edge cases:
 *   - All sensors OFF: line is lost. The last known error is returned
 *     so the robot continues curving in the direction the line was last
 *     seen, rather than stopping abruptly.
 *   - All sensors ON: intersection detected. Returns 0 (straight ahead).
 *     Add intersection-handling logic here if needed.
 */
float computeError()
{
    int   weightedSum = 0;
    int   activeCount = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i]) {
            weightedSum += SENSOR_WEIGHTS[i];
            activeCount++;
        }
    }

    // All sensors OFF — line is lost
    if (activeCount == 0) {
        // Return the last known error to maintain the last steering direction
        return pidLastError;
    }

    // All sensors ON — intersection or very wide line; treat as center
    if (activeCount == NUM_SENSORS) {
        return 0.0;
    }

    // Normal case: return fractional weighted average
    return (float)weightedSum / (float)activeCount;
}


// ============================================================
//  PID ALGORITHM
// ============================================================

/*
 * computePID(error)
 * Standard discrete PID controller.
 *
 * Proportional term: responds immediately to the current error.
 * Integral term:     accumulates past error to eliminate steady-state offset.
 * Derivative term:   predicts future error from the rate of change,
 *                    damping oscillation.
 *
 * Parameters:
 *   error  - current position error from computeError()
 *
 * Returns:
 *   pid_output - a signed correction value applied to motor speeds.
 *                Positive -> steer right; Negative -> steer left.
 */
float computePID(float error)
{
    // Proportional term
    float pTerm = KP * error;

    // Integral term with windup clamp
    pidIntegral += error;
    pidIntegral  = constrain(pidIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float iTerm  = KI * pidIntegral;

    // Derivative term (rate of error change between consecutive loops)
    float dTerm    = KD * (error - pidLastError);
    pidLastError   = error;

    return pTerm + iTerm + dTerm;
}


// ============================================================
//  MOTOR OUTPUT
// ============================================================

/*
 * applyMotorOutput(pidOutput)
 * Converts the PID correction signal into individual left/right
 * motor speeds and drives the motors accordingly.
 *
 * Positive pidOutput -> robot needs to turn right
 *   -> right motor slows down, left motor speeds up
 * Negative pidOutput -> robot needs to turn left
 *   -> left motor slows down, right motor speeds up
 */
void applyMotorOutput(float pidOutput)
{
    int leftSpeed  = BASE_SPEED + (int)pidOutput;
    int rightSpeed = BASE_SPEED - (int)pidOutput;

    // Clamp speeds to valid PWM range
    leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
}


// ============================================================
//  MOTOR DRIVER HELPERS
// ============================================================

/*
 * setLeftMotor(speed)
 * Drives the left motor forward at the given PWM duty cycle.
 * Pass a negative value if reverse is needed (not used in
 * normal line-following but useful for sharp turns or recovery).
 */
void setLeftMotor(int speed)
{
    if (speed >= 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, speed);
    } else {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        analogWrite(PIN_ENA, -speed);
    }
}

/*
 * setRightMotor(speed)
 * Drives the right motor forward at the given PWM duty cycle.
 */
void setRightMotor(int speed)
{
    if (speed >= 0) {
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, speed);
    } else {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENB, -speed);
    }
}

/*
 * stopMotors()
 * Cuts power to both motors (coast stop).
 * For a braking stop, set both IN pins HIGH simultaneously.
 */
void stopMotors()
{
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);

    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, 0);
}


// ============================================================
//  START BUTTON
// ============================================================

/*
 * waitForStartButton()
 * Blocks execution until the button on PIN_START_BTN is pressed.
 * Uses a simple 50 ms debounce. Remove this function call in
 * setup() if no button is wired.
 */
void waitForStartButton()
{
    while (digitalRead(PIN_START_BTN) == HIGH) {
        // Waiting for button press (active LOW)
    }
    delay(50);  // Debounce
}


// ============================================================
//  DEBUG OUTPUT
// ============================================================

/*
 * debugPrint(error, output, leftSpeed, rightSpeed)
 * Streams live PID data to the Serial Monitor at 115200 baud.
 * Outputs a CSV-friendly line that can be pasted into a
 * spreadsheet or the Arduino Serial Plotter for tuning.
 *
 * Format: error, output, leftSpeed, rightSpeed, sensor_bits
 *
 * To use: uncomment the debugPrint() call in loop().
 */
void debugPrint(float error, float output, int leftSpeed, int rightSpeed)
{
    // Sensor state as a 5-bit binary string
    char sensorStr[6];
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorStr[i] = sensorValues[i] ? '1' : '0';
    }
    sensorStr[NUM_SENSORS] = '\0';

    Serial.print(F("ERR:"));    Serial.print(error,  2);
    Serial.print(F(" OUT:"));   Serial.print(output, 2);
    Serial.print(F(" L:"));     Serial.print(leftSpeed);
    Serial.print(F(" R:"));     Serial.print(rightSpeed);
    Serial.print(F(" SNS:"));   Serial.println(sensorStr);
}
