#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define SDA_CONVEYOR_SENSOR     11
#define SCL_CONVEYOR_SENSOR     12

#define MOTOR_CONVEYOR_FORWARD  9
#define MOTOR_CONVEYOR_BACKWARD 10

#define MOTOR_MECHANISM_IN1 18
#define MOTOR_MECHANISM_IN2 8  
#define LIMIT_SWITCH_MOTOR_MECHANISM 48

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

const float DISTANCE_UPPER_LIMIT = 500.0f;  // mm
const float DISTANCE_LOWER_LIMIT = 60.0f;   // mm

// ============================================================
// TIMERS & LOCKOUT CONFIGURATION
// ============================================================
// When leaving ARRIVED (box picked up / removed), the sensor reading
// sweeps through the MOVING zone on its way to IDLE. This lockout
// window blocks the motor from reacting to that transient sweep.
const unsigned long LOCKOUT_DURATION_MS = 700; // 500-1000 ms recommended

// Tiempo máximo de activación del mecanismo antes de obligar el retorno (configurable)
const unsigned long MAX_MECH_DISPENSE_TIME_MS = 5800; // 5000 ms = 5 segundos

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

// ============================================================
// STATES ENUMS
// ============================================================

enum MotorState : uint8_t
{
    MOTOR_STOPPED,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
};
MotorState currentMotorState = MOTOR_STOPPED;

// Máquina de estados de la banda transportadora
enum SystemState : uint8_t
{
    STATE_IDLE,     // distance >= DISTANCE_UPPER_LIMIT, motor stopped
    STATE_MOVING,   // LOWER < distance < UPPER, motor running backward
    STATE_ARRIVED,  // distance <= DISTANCE_LOWER_LIMIT, motor stopped
    STATE_LOCKOUT   // just left ARRIVED; ignoring transient MOVING readings
};
SystemState currentState = STATE_IDLE;
unsigned long lockoutStartTime = 0;

// Máquina de estados del mecanismo secundario
enum MechState : uint8_t
{
    MECH_IDLE,       // Esperando comando de activación
    MECH_DISPENSING, // Motor mecanismo avanzando hacia la banda
    MECH_RETURNING   // Motor mecanismo retrocediendo hasta final de carrera
};
MechState currentMechState = MECH_IDLE;
unsigned long mechDispenseStartTime = 0;


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

// --- Controles de la Banda ---
void motorStop()
{
    if (currentMotorState != MOTOR_STOPPED)
    {
        digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
        digitalWrite(MOTOR_CONVEYOR_BACKWARD, LOW);
        currentMotorState = MOTOR_STOPPED;
        Serial.println("Banda: STOP");
    }
}

void motorForward()
{
    if (currentMotorState != MOTOR_FORWARD)
    {
        digitalWrite(MOTOR_CONVEYOR_FORWARD, HIGH);
        digitalWrite(MOTOR_CONVEYOR_BACKWARD, LOW);
        currentMotorState = MOTOR_FORWARD;
        Serial.println("Banda: FORWARD");
    }
}

void motorBackward()
{
    if (currentMotorState != MOTOR_BACKWARD)
    {
        digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
        digitalWrite(MOTOR_CONVEYOR_BACKWARD, HIGH);
        currentMotorState = MOTOR_BACKWARD;
        Serial.println("Banda: BACKWARD");
    }
}

// --- Controles del Mecanismo Secundario ---
void mechStop()
{
    digitalWrite(MOTOR_MECHANISM_IN1, LOW);
    digitalWrite(MOTOR_MECHANISM_IN2, LOW);
}

void mechForward() // Dispensar (Avanzar)
{
    digitalWrite(MOTOR_MECHANISM_IN1, HIGH);
    digitalWrite(MOTOR_MECHANISM_IN2, LOW);
}

void mechBackward() // Retornar
{
    digitalWrite(MOTOR_MECHANISM_IN1, LOW);
    digitalWrite(MOTOR_MECHANISM_IN2, HIGH);
}


// ============================================================
// STATE MACHINES
// ============================================================

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
                currentState = STATE_LOCKOUT;
                lockoutStartTime = now;
            }
            break;

        case STATE_LOCKOUT:
            if (d <= DISTANCE_LOWER_LIMIT)
            {
                currentState = STATE_ARRIVED;
            }
            else if (d >= DISTANCE_UPPER_LIMIT)
            {
                currentState = STATE_IDLE;
            }
            else if (now - lockoutStartTime >= LOCKOUT_DURATION_MS)
            {
                currentState = STATE_MOVING;
            }
            break;
    }
}

// Lógica no bloqueante para el mecanismo secundario
void updateMechStateMachine(unsigned long now)
{
    switch (currentMechState)
    {
        case MECH_IDLE:
            // Se espera activación vía comando (Ver ciclo loop)
            break;

        case MECH_DISPENSING:
            // Condición 1: Caja detectada en la banda (STATE_ARRIVED)
            // Condición 2: Timeout excedido
            if ((currentState == STATE_ARRIVED) || 
                (now - mechDispenseStartTime >= MAX_MECH_DISPENSE_TIME_MS))
            {
                currentMechState = MECH_RETURNING;
                mechBackward();
                Serial.println("Mecanismo: RETORNANDO (Condición de parada alcanzada)");
            }
            break;

        case MECH_RETURNING:
            // Se detiene al activar el final de carrera (Señal HIGH por Pull-down a 3.3v)
            if (digitalRead(LIMIT_SWITCH_MOTOR_MECHANISM) == HIGH)
            {
                currentMechState = MECH_IDLE;
                mechStop();
                Serial.println("Mecanismo: CICLO COMPLETADO. Motor detenido (IDLE)");
            }
            break;
    }
}

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

    // --- Motor conveyor pins ---
    pinMode(MOTOR_CONVEYOR_FORWARD, OUTPUT);
    pinMode(MOTOR_CONVEYOR_BACKWARD, OUTPUT);
    digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
    digitalWrite(MOTOR_CONVEYOR_BACKWARD, LOW);

    // --- Motor mechanism pins ---
    pinMode(MOTOR_MECHANISM_IN1, OUTPUT);
    pinMode(MOTOR_MECHANISM_IN2, OUTPUT);
    digitalWrite(MOTOR_MECHANISM_IN1, LOW);
    digitalWrite(MOTOR_MECHANISM_IN2, LOW);

    // --- Limit switch pins ---
    // Usamos pulldown, por tanto estado inactivo es LOW, activo es HIGH (3.3v)
    pinMode(LIMIT_SWITCH_MOTOR_MECHANISM, INPUT_PULLDOWN);

    // ========================================================
    // HOMING DEL MECANISMO AL INICIAR
    // ========================================================
    Serial.println("Verificando posicion inicial del mecanismo...");
    
    // Si el final de carrera NO está presionado, forzamos el retroceso
    if (digitalRead(LIMIT_SWITCH_MOTOR_MECHANISM) == LOW) 
    {
        Serial.println("Mecanismo fuera de HOME. Retornando...");
        mechBackward();
        
        // Bloqueamos la ejecución aquí hasta que el switch se active
        while (digitalRead(LIMIT_SWITCH_MOTOR_MECHANISM) == LOW) 
        {
            delay(10); // Pequeño delay para no saturar el watchdog
        }
        
        mechStop();
        Serial.println("Mecanismo asegurado en posicion HOME.");
    } 
    else 
    {
        Serial.println("El mecanismo ya se encuentra en HOME.");
    }
    
    // Aseguramos que el estado inicie en IDLE después del homing
    currentMechState = MECH_IDLE; 
    // ========================================================

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
    Serial.println("Envíe 'A' por puerto serial para activar el mecanismo.");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    unsigned long now = millis();

    // 1. Manejo de comandos para el mecanismo
    if (Serial.available() > 0)
    {
        char cmd = Serial.read();
        // Disparar mecanismo si está inactivo
        if ((cmd == 'A' || cmd == 'a') && currentMechState == MECH_IDLE)
        {
            currentMechState = MECH_DISPENSING;
            mechDispenseStartTime = now;
            mechForward();
            Serial.println("Mecanismo: DISPENSANDO (Comando Recibido)");
        }
    }

    // 2. Lectura y procesamiento de la banda
    uint16_t distance = sensor.readRangeContinuousMillimeters();

    if (sensor.timeoutOccurred())
    {
        Serial.println("Timeout sensor");
        motorStop(); // fail-safe
        return;
    }

    if (distance <= MAX_VALID_DISTANCE)
    {
        float filtered;
        if (addMeasurement(distance, filtered))
        {
            float calibratedDistance = filtered - CALIBRATION_OFFSET;
            SystemState previousState = currentState;
            
            // Actualizar estado de la banda
            updateStateMachine(calibratedDistance, now);
            applyMotorForState();

            // Descomentar para debug de distancias continuas (opcional)
            /*
            Serial.print("Filtered: ");
            Serial.print(filtered);
            Serial.print(" mm   State: ");
            Serial.print(stateName(currentState));
            if (currentState == STATE_LOCKOUT && previousState != STATE_LOCKOUT)
            {
                Serial.print(" (lockout started)");
            }
            Serial.println();
            */
        }
    }

    // 3. Ejecutar lógica del mecanismo
    updateMechStateMachine(now);
}