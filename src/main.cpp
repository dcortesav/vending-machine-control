#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================================================
// PIN DEFINITIONS - CNC
// ============================================================

#define EN_PIN_Y   4
#define STEP_PIN_Y 5
#define DIR_PIN_Y  6

#define EN_PIN_X   7
#define STEP_PIN_X 15
#define DIR_PIN_X  16

#define LIMIT_SWITCH_CNC_X 47
#define LIMIT_SWITCH_CNC_Y 21

// ============================================================
// PIN DEFINITIONS - CONVEYOR / MECHANISM
// ============================================================

#define SDA_CONVEYOR_SENSOR     11
#define SCL_CONVEYOR_SENSOR     12

#define MOTOR_CONVEYOR_FORWARD  9
#define MOTOR_CONVEYOR_BACKWARD 10

#define MOTOR_MECHANISM_IN1 18
#define MOTOR_MECHANISM_IN2 8
#define LIMIT_SWITCH_MOTOR_MECHANISM 48

// ============================================================
// CONSTANTS - CNC
// ============================================================

const float PASOS_POR_MM = 20.0;
// NOTA: TIEMPO_EN_CELDA se conserva definido por compatibilidad, pero ya
// no se usa como delay fijo: ahora el tiempo de espera en la celda lo
// determina el ciclo completo del mecanismo de dispensacion (ver
// ejecutarCicloDeTrabajo()), tal como exige la secuencia de coordinacion.
const unsigned long TIEMPO_EN_CELDA = 2000;

AccelStepper motorX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper motorY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

struct Celda {
  float x_mm;
  float y_mm;
};

const int FILAS = 5;
const int COLUMNAS = 6;

Celda matrizCeldas[FILAS][COLUMNAS] = {
  // Fila 1
  {{22, 10}, {112, 10}, {202, 10}, {293, 10}, {384, 10}, {474, 10}},
  // Fila 2
  {{21, 150}, {112, 150}, {203, 150}, {293, 150}, {385, 150}, {477, 150}},
  // Fila 3
  {{20, 290}, {110, 290}, {201, 290}, {294, 290}, {386, 290}, {477, 290}},
  // Fila 4
  {{18, 430}, {108, 430}, {201, 430}, {293, 430}, {386, 430}, {477, 430}},
  // Fila 5
  {{16, 570}, {108, 570}, {201, 570}, {293, 570}, {385, 570}, {477, 570}},
};

// ============================================================
// SENSOR / FILTER CONFIGURATION - CONVEYOR
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
const unsigned long LOCKOUT_DURATION_MS = 700; // 500-1000 ms recommended

// Tiempo máximo de activación del mecanismo antes de obligar el retorno (configurable)
const unsigned long MAX_MECH_DISPENSE_TIME_MS = 5800; // 5000 ms = 5 segundos

// ============================================================
// GLOBALS - CONVEYOR
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
// STATE MACHINES (CONVEYOR / MECHANISM) - SIN CAMBIOS
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
            // Se espera activación (ahora disparada por ejecutarCicloDeTrabajo() del CNC)
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
// COORDINACIÓN: lógica original del loop() del conveyor (pasos
// "2. Lectura y procesamiento de la banda" y "3. Ejecutar lógica
// del mecanismo") extraída a una función para poder invocarla
// tanto desde loop() como desde la espera bloqueante del CNC.
// El contenido es EXACTAMENTE el mismo del loop() original.
// ============================================================

void procesarBandaYMecanismo()
{
    unsigned long now = millis();

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

            // Actualizar estado de la banda
            updateStateMachine(calibratedDistance, now);
            applyMotorForState();
        }
    }

    // Ejecutar lógica del mecanismo
    updateMechStateMachine(now);
}

// ==========================================
// FUNCIONES CNC (DE ARRIBA A ABAJO) - SIN CAMBIOS EN SU LÓGICA INTERNA
// ==========================================

void habilitarMotores() {
  digitalWrite(EN_PIN_X, LOW);
  digitalWrite(EN_PIN_Y, LOW);
  delay(50);
}

void deshabilitarMotores() {
  digitalWrite(EN_PIN_X, HIGH);
  digitalWrite(EN_PIN_Y, HIGH);
}

void esperarMovimiento() {
  while (motorX.distanceToGo() != 0 || motorY.distanceToGo() != 0) {
    motorX.run();
    motorY.run();
  }
}

void irAHome() {
  habilitarMotores();

  motorX.setMaxSpeed(1000);
  motorX.setAcceleration(10000);
  motorY.setMaxSpeed(1000);
  motorY.setAcceleration(10000);

  // === 1. HOMING EJE Y (Con filtro Antirrebote) ===
  motorY.move(-100000);

  int confianzaY = 0;
  bool finalCarreraY_Alcanzado = false;

  while (!finalCarreraY_Alcanzado) {
    motorY.run();

    if (digitalRead(LIMIT_SWITCH_CNC_Y) == HIGH) {
      confianzaY++;
    } else {
      confianzaY--;
    }

    // Mantener el contador en el límite inferior de cero
    if (confianzaY < 0) {
      confianzaY = 0;
    }

    // Umbral de confianza (Ajustable. 500 lecturas netas positivas garantizan un toque real)
    if (confianzaY > 500) {
      finalCarreraY_Alcanzado = true;
    }
  }

  motorY.stop();
  motorY.runToPosition();
  motorY.setCurrentPosition(0);

  motorY.moveTo(5 * PASOS_POR_MM);
  motorY.runToPosition();
  motorY.setCurrentPosition(0);

  // === 2. HOMING EJE X (Con filtro Antirrebote) ===
  motorX.move(-100000);

  unsigned long tiempoDebounceX = 0;
  bool finalCarreraX_Alcanzado = false;

  while (!finalCarreraX_Alcanzado) {
    motorX.run();

    if (digitalRead(LIMIT_SWITCH_CNC_X) == HIGH) {
      if (tiempoDebounceX == 0) {
        tiempoDebounceX = millis();
      }
      if (millis() - tiempoDebounceX > 30) {
        finalCarreraX_Alcanzado = true;
      }
    } else {
      tiempoDebounceX = 0;
    }
  }

  motorX.stop();
  motorX.runToPosition();
  motorX.setCurrentPosition(0);

  motorX.moveTo(5 * PASOS_POR_MM);
  motorX.runToPosition();
  motorX.setCurrentPosition(0);

  motorX.setMaxSpeed(4000);
  motorX.setAcceleration(10000);
  motorY.setMaxSpeed(4000);
  motorY.setAcceleration(10000);

  deshabilitarMotores();
  Serial.println("Home establecido. Ejes liberados y en 0,0 seguro.");
}

// ==========================================
// COORDINACIÓN CNC <-> MECANISMO
// ==========================================
//
// Cambios respecto al original:
//  - Al llegar a la posición, en vez de "delay(TIEMPO_EN_CELDA)" se dispara
//    el mecanismo de dispensación (mismo código que usaba el comando serial
//    'A' del programa original del conveyor) y se espera, sin bloquear la
//    banda, a que el mecanismo complete TODO su ciclo (Dispensando ->
//    Retornando -> Idle). Solo entonces se ejecuta irAHome().
//
void ejecutarCicloDeTrabajo(float targetX, float targetY) {
  habilitarMotores();

  long pasosX = targetX * PASOS_POR_MM;
  long pasosY = targetY * PASOS_POR_MM;

  motorX.moveTo(pasosX);
  motorY.moveTo(pasosY);

  esperarMovimiento();
  Serial.println("Posicion alcanzada. Iniciando dispensacion...");

  // Disparo del mecanismo de dispensación (equivalente al comando 'A' del
  // programa original del conveyor)
  if (currentMechState == MECH_IDLE) {
    currentMechState = MECH_DISPENSING;
    mechDispenseStartTime = millis();
    mechForward();
    Serial.println("Mecanismo: DISPENSANDO (Disparado por llegada del CNC)");
  }

  // Esperar a que el mecanismo complete su ciclo completo (hasta volver a
  // MECH_IDLE, es decir, hasta que regrese a HOME), manteniendo activa la
  // lógica de la banda transportadora durante toda la espera.
  while (currentMechState != MECH_IDLE) {
    procesarBandaYMecanismo();
  }

  // El mecanismo ya volvió a HOME, pero el CNC debe permanecer en la celda
  // hasta que la banda confirme la llegada de la caja a su destino final
  // (mismo estado STATE_ARRIVED de la máquina de estados existente).
  // Se sigue llamando a procesarBandaYMecanismo() para que el sensor
  // VL53L0X y la máquina de estados de la banda continúen actualizándose
  // normalmente durante esta espera.
  while (currentState != STATE_ARRIVED) {
    procesarBandaYMecanismo();
  }

  Serial.println("Retornando a Home...");
  irAHome();
}

void leerCeldaSerial() {
  if (!Serial.available()) return;

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();

  if (entrada.length() != 2) {
    Serial.println("Error: Formato invalido. Use 2 digitos (Ej: 42)");
    return;
  }

  int columna = entrada.charAt(0) - '0';
  int fila = entrada.charAt(1) - '0';

  if (fila < 1 || fila > FILAS || columna < 1 || columna > COLUMNAS) {
    Serial.println("Error: Celda fuera de rango (Filas: 1-5, Columnas: 1-6)");
    return;
  }

  // === Restricción de seguridad exigida ===
  // El CNC solo puede moverse si el mecanismo de dispensación está
  // físicamente en HOME (final de carrera activo, misma lógica ya
  // existente en el proyecto original, sin nuevos filtros ni validaciones).
  if (digitalRead(LIMIT_SWITCH_MOTOR_MECHANISM) != HIGH) {
    Serial.println("Error: Mecanismo de dispensacion no esta en HOME. Movimiento del CNC bloqueado.");
    return;
  }

  float targetX = matrizCeldas[fila - 1][columna - 1].x_mm;
  float targetY = matrizCeldas[fila - 1][columna - 1].y_mm;

  Serial.print("Comando recibido: "); Serial.print(entrada);
  Serial.print(" -> Moviendo a X: "); Serial.print(targetX);
  Serial.print("mm, Y: "); Serial.print(targetY); Serial.println("mm");

  ejecutarCicloDeTrabajo(targetX, targetY);
}

// ==========================================
// FUNCIONES PRINCIPALES
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(50);

  // ---- Pines de la banda transportadora ----
  pinMode(MOTOR_CONVEYOR_FORWARD, OUTPUT);
  pinMode(MOTOR_CONVEYOR_BACKWARD, OUTPUT);
  digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
  digitalWrite(MOTOR_CONVEYOR_BACKWARD, LOW);

  // ---- Pines del mecanismo secundario ----
  pinMode(MOTOR_MECHANISM_IN1, OUTPUT);
  pinMode(MOTOR_MECHANISM_IN2, OUTPUT);
  digitalWrite(MOTOR_MECHANISM_IN1, LOW);
  digitalWrite(MOTOR_MECHANISM_IN2, LOW);

  // ---- Final de carrera del mecanismo ----
  pinMode(LIMIT_SWITCH_MOTOR_MECHANISM, INPUT_PULLDOWN);

  // ========================================================
  // PASO 1a: HOMING DEL MECANISMO DE DISPENSACION (idéntico al original)
  // ========================================================
  Serial.println("Verificando posicion inicial del mecanismo...");

  if (digitalRead(LIMIT_SWITCH_MOTOR_MECHANISM) == LOW)
  {
    Serial.println("Mecanismo fuera de HOME. Retornando...");
    mechBackward();

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

  currentMechState = MECH_IDLE;

  // ---- Sensor de distancia VL53L0X (idéntico al original) ----
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
  Serial.println("Sensor listo.");

  // ========================================================
  // PASO 1b: HOMING DEL CNC (idéntico al original, se ejecuta
  // únicamente después de confirmar que el mecanismo está en HOME)
  // ========================================================
  pinMode(EN_PIN_X, OUTPUT);
  pinMode(EN_PIN_Y, OUTPUT);

  pinMode(LIMIT_SWITCH_CNC_X, INPUT_PULLDOWN);
  pinMode(LIMIT_SWITCH_CNC_Y, INPUT_PULLDOWN);

  motorX.setPinsInverted(true, false, false);
  motorY.setPinsInverted(true, false, false);

  motorX.setMaxSpeed(4000);
  motorX.setAcceleration(10000);
  motorY.setMaxSpeed(4000);
  motorY.setAcceleration(10000);

  deshabilitarMotores();

  Serial.println("Iniciando sistema. Buscando Home del CNC...");
  irAHome();

  // ========================================================
  // PASO 1c: SISTEMA LISTO
  // ========================================================
  Serial.println("Sistema listo. Mecanismo y CNC en HOME.");
  Serial.println("Ingrese la celda de 2 digitos (ColumnaFila).");
}

void loop() {
  // La banda transportadora y el mecanismo siguen funcionando
  // exactamente igual que en el programa original, en todo momento.
  procesarBandaYMecanismo();

  // Espera del mismo comando serial de celda que en el CNC original.
  leerCeldaSerial();
}