#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define SDA_CONVEYOR_SENSOR     21
#define SCL_CONVEYOR_SENSOR     22

#define MOTOR_CONVEYOR_FORWARD  26
#define MOTOR_CONVEYOR_BACKWARD 27

// ============================================================
// SENSOR / FILTER CONFIGURATION
// ============================================================

const uint16_t MAX_VALID_DISTANCE = 1000;   // mm, discard readings above this
const uint8_t  MEDIAN_SIZE        = 5;
const uint8_t  AVERAGE_SIZE       = 5;

// Calibration offset applied to the filtered distance
const float CALIBRATION_OFFSET = 50.0f;

// ============================================================
// CONTROL LOOP THRESHOLDS (on calibrated distance)
// ============================================================
//   calibratedDistance >= DISTANCE_UPPER_LIMIT           -> STOP
//   DISTANCE_LOWER_LIMIT < calibratedDistance < UPPER     -> RUN BACKWARD
//   calibratedDistance <= DISTANCE_LOWER_LIMIT            -> STOP

const float DISTANCE_UPPER_LIMIT = 600.0f;  // mm
const float DISTANCE_LOWER_LIMIT = 60.0f;   // mm

// ============================================================
// TRANSITION LOCKOUT CONFIGURATION
// ============================================================
// When leaving ARRIVED (box picked up / removed), the sensor reading
// sweeps through the MOVING zone on its way to IDLE. This lockout
// window blocks the motor from reacting to that transient sweep.
const unsigned long LOCKOUT_DURATION_MS = 700; // 500-1000 ms recommended

// ============================================================
// GLOBALS
// ============================================================

VL53L0X sensor;

uint16_t medianBuffer[MEDIAN_SIZE];
uint16_t averageBuffer[AVERAGE_SIZE];

uint8_t medianIndex  = 0;
uint8_t averageIndex = 0;

bool medianFilled  = false;
bool averageFilled = false;

enum MotorState : uint8_t
{
    MOTOR_STOPPED,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
};

MotorState currentMotorState = MOTOR_STOPPED;

// System-level state machine (independent from MotorState, which only
// tracks the physical GPIO output). This is what implements the fix.
enum SystemState : uint8_t
{
    STATE_IDLE,     // distance >= DISTANCE_UPPER_LIMIT, motor stopped
    STATE_MOVING,   // LOWER < distance < UPPER, motor running backward
    STATE_ARRIVED,  // distance <= DISTANCE_LOWER_LIMIT, motor stopped
    STATE_LOCKOUT   // just left ARRIVED; ignoring transient MOVING readings
};

SystemState currentState = STATE_IDLE;
unsigned long lockoutStartTime = 0;

// ============================================================
// FILTERING HELPERS
// ============================================================

uint16_t median(uint16_t *array, uint8_t size)
{
    uint16_t temp[size];
    memcpy(temp, array, sizeof(uint16_t) * size);

    for (uint8_t i = 0; i < size - 1; i++)
    {
        for (uint8_t j = i + 1; j < size; j++)
        {
            if (temp[j] < temp[i])
            {
                uint16_t t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    return temp[size / 2];
}

float movingAverage(uint16_t *array, uint8_t size)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < size; i++)
        sum += array[i];

    return (float)sum / size;
}

// Feeds a raw reading through the median -> moving-average pipeline.
// Returns true and writes filteredDistance once both buffers are full.
bool addMeasurement(uint16_t value, float &filteredDistance)
{
    medianBuffer[medianIndex] = value;
    medianIndex++;

    if (medianIndex >= MEDIAN_SIZE)
    {
        medianIndex = 0;
        medianFilled = true;
    }

    if (!medianFilled)
        return false;

    uint16_t med = median(medianBuffer, MEDIAN_SIZE);

    averageBuffer[averageIndex] = med;
    averageIndex++;

    if (averageIndex >= AVERAGE_SIZE)
    {
        averageIndex = 0;
        averageFilled = true;
    }

    if (!averageFilled)
        return false;

    filteredDistance = movingAverage(averageBuffer, AVERAGE_SIZE);
    return true;
}

// ============================================================
// MOTOR CONTROL HELPERS
// ============================================================

void motorStop()
{
    if (currentMotorState != MOTOR_STOPPED)
    {
        digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
        digitalWrite(MOTOR_CONVEYOR_BACKWARD, LOW);
        currentMotorState = MOTOR_STOPPED;
        Serial.println("Motor: STOP");
    }
}

void motorForward()
{
    if (currentMotorState != MOTOR_FORWARD)
    {
        digitalWrite(MOTOR_CONVEYOR_FORWARD, HIGH);
        digitalWrite(MOTOR_CONVEYOR_BACKWARD, LOW);
        currentMotorState = MOTOR_FORWARD;
        Serial.println("Motor: FORWARD");
    }
}

void motorBackward()
{
    if (currentMotorState != MOTOR_BACKWARD)
    {
        digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
        digitalWrite(MOTOR_CONVEYOR_BACKWARD, HIGH);
        currentMotorState = MOTOR_BACKWARD;
        Serial.println("Motor: BACKWARD");
    }
}

// ============================================================
// STATE MACHINE
// ============================================================
//
// Normal transitions (IDLE <-> MOVING <-> ARRIVED) happen immediately.
// The only guarded transition is ARRIVED -> (MOVING/IDLE): leaving
// ARRIVED always routes through LOCKOUT first, which suppresses the
// MOVING zone for LOCKOUT_DURATION_MS. Reaching IDLE (>= upper limit)
// is inherently safe (motor stays stopped) so it's allowed to confirm
// immediately and exit the lockout early.
void updateStateMachine(float d, unsigned long now)
{
    switch (currentState)
    {
        case STATE_IDLE:
            if (d <= DISTANCE_LOWER_LIMIT)
                currentState = STATE_ARRIVED;
            else if (d < DISTANCE_UPPER_LIMIT)
                currentState = STATE_MOVING;
            break;

        case STATE_MOVING:
            if (d <= DISTANCE_LOWER_LIMIT)
                currentState = STATE_ARRIVED;
            else if (d >= DISTANCE_UPPER_LIMIT)
                currentState = STATE_IDLE;
            break;

        case STATE_ARRIVED:
            if (d > DISTANCE_LOWER_LIMIT)
            {
                // Box picked up / removed: don't trust the reading yet.
                currentState = STATE_LOCKOUT;
                lockoutStartTime = now;
            }
            break;

        case STATE_LOCKOUT:
            if (d <= DISTANCE_LOWER_LIMIT)
            {
                // False alarm / bounce - box is still there.
                currentState = STATE_ARRIVED;
            }
            else if (d >= DISTANCE_UPPER_LIMIT)
            {
                // Confirmed clear all the way to IDLE - safe to exit early.
                currentState = STATE_IDLE;
            }
            else if (now - lockoutStartTime >= LOCKOUT_DURATION_MS)
            {
                // Cooldown expired and reading is still in the MOVING
                // zone -> this is now trusted as a genuine new object.
                currentState = STATE_MOVING;
            }
            // else: still within cooldown and still in the MOVING zone
            // -> stay in LOCKOUT, motor remains stopped.
            break;
    }
}

// Human-readable state name for debug logging.
const char *stateName(SystemState state)
{
    switch (state)
    {
        case STATE_IDLE:    return "IDLE";
        case STATE_MOVING:  return "MOVING";
        case STATE_ARRIVED: return "ARRIVED";
        case STATE_LOCKOUT: return "LOCKOUT";
        default:            return "UNKNOWN";
    }
}

// Maps the current system state to the physical motor output.
void applyMotorForState()
{
    switch (currentState)
    {
        case STATE_MOVING:
            motorBackward();
            break;

        case STATE_IDLE:
        case STATE_ARRIVED:
        case STATE_LOCKOUT:
        default:
            motorStop();
            break;
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(50);

    // --- Motor pins ---
    pinMode(MOTOR_CONVEYOR_FORWARD, OUTPUT);
    pinMode(MOTOR_CONVEYOR_BACKWARD, OUTPUT);
    digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
    digitalWrite(MOTOR_CONVEYOR_BACKWARD, LOW);

    // --- Distance sensor ---
    Wire.begin(SDA_CONVEYOR_SENSOR, SCL_CONVEYOR_SENSOR);

    sensor.setTimeout(100);

    if (!sensor.init())
    {
        Serial.println("VL53L0X no encontrado");
        while (1)
        {
            // Halt: sensor is required for safe motor operation.
        }
    }

    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);

    Serial.println("Sensor listo. Sistema iniciado.");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    uint16_t distance = sensor.readRangeContinuousMillimeters();

    if (sensor.timeoutOccurred())
    {
        Serial.println("Timeout");
        motorStop(); // fail-safe: stop motor on sensor timeout
        return;
    }

    if (distance > MAX_VALID_DISTANCE)
    {
        return; // discard absurd reading, keep previous motor state
    }

    float filtered;
    if (!addMeasurement(distance, filtered))
    {
        return; // filter buffers not yet full
    }

    float calibratedDistance = filtered - CALIBRATION_OFFSET;
    unsigned long now = millis();

    SystemState previousState = currentState;
    updateStateMachine(calibratedDistance, now);
    applyMotorForState();

    Serial.print("Raw: ");
    Serial.print(distance);
    Serial.print(" mm   Filtered: ");
    Serial.print(filtered);
    Serial.print(" mm   Calibrated: ");
    Serial.print(calibratedDistance);
    Serial.print(" mm   State: ");
    Serial.print(stateName(currentState));

    if (currentState == STATE_LOCKOUT && previousState != STATE_LOCKOUT)
    {
        Serial.print(" (lockout started)");
    }

    Serial.println();
}