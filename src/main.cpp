#include <Arduino.h>
#include <AccelStepper.h>

#define EN_PIN_Y   4
#define STEP_PIN_Y 5
#define DIR_PIN_Y  6

#define EN_PIN_X   7
#define STEP_PIN_X 15
#define DIR_PIN_X  16

#define LIMIT_SWITCH_CNC_X 47
#define LIMIT_SWITCH_CNC_Y 21

const float PASOS_POR_MM = 20.0;
const unsigned long TIEMPO_EN_CELDA = 2000; // 3 segundos

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

// ==========================================
// DEFINICIÓN DE FUNCIONES (DE ARRIBA A ABAJO)
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

  // Reducir la velocidad temporalmente para el Homing
  motorX.setMaxSpeed(1000); 
  motorX.setAcceleration(10000);
  motorY.setMaxSpeed(1000); 
  motorY.setAcceleration(10000);

  // === 1. HOMING EJE Y (Se ejecuta primero) ===
  motorY.move(-100000); 

  // Lógica Pull-Down: Reposo es LOW. Sale cuando detecta HIGH.
  while (digitalRead(LIMIT_SWITCH_CNC_Y) == LOW) {
    motorY.run();
  }
  
  motorY.stop();
  motorY.runToPosition(); 
  motorY.setCurrentPosition(0); // Cero temporal en el switch
  
  // Retroceder 5 mm para liberar el sensor
  motorY.moveTo(5 * PASOS_POR_MM);
  motorY.runToPosition(); // Función bloqueante hasta alcanzar los 5mm
  motorY.setCurrentPosition(0); // Este es ahora nuestro cero absoluto seguro en Y

  // === 2. HOMING EJE X (Se ejecuta después de Y) ===
  motorX.move(-100000); 
  
  while (digitalRead(LIMIT_SWITCH_CNC_X) == LOW) {
    motorX.run();
  }
  
  motorX.stop(); 
  motorX.runToPosition(); 
  motorX.setCurrentPosition(0); // Cero temporal en el switch
  
  // Retroceder 5 mm para liberar el sensor
  motorX.moveTo(5 * PASOS_POR_MM);
  motorX.runToPosition(); // Función bloqueante hasta alcanzar los 5mm
  motorX.setCurrentPosition(0); // Este es ahora nuestro cero absoluto seguro en X

  // Restaurar velocidades de trabajo
  motorX.setMaxSpeed(4000);
  motorX.setAcceleration(10000);
  motorY.setMaxSpeed(4000);
  motorY.setAcceleration(10000);

  deshabilitarMotores();
  Serial.println("Home establecido. Ejes liberados y en 0,0 seguro.");
}

void ejecutarCicloDeTrabajo(float targetX, float targetY) {
  habilitarMotores();

  long pasosX = targetX * PASOS_POR_MM;
  long pasosY = targetY * PASOS_POR_MM;

  motorX.moveTo(pasosX);
  motorY.moveTo(pasosY);

  esperarMovimiento();
  Serial.println("Posicion alcanzada. Esperando...");

  delay(TIEMPO_EN_CELDA);

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
    Serial.println("Error: Celda fuera de rango (Filas: 1-6, Columnas: 1-5)");
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

  pinMode(EN_PIN_X, OUTPUT);
  pinMode(EN_PIN_Y, OUTPUT);

  // Configuración de finales de carrera como INPUT normal (Pull-Down físico)
  pinMode(LIMIT_SWITCH_CNC_X, INPUT_PULLDOWN);
  pinMode(LIMIT_SWITCH_CNC_Y, INPUT_PULLDOWN);

  // Invertir la dirección de los motores por software
  motorX.setPinsInverted(true, false, false);
  motorY.setPinsInverted(true, false, false);

  motorX.setMaxSpeed(4000);
  motorX.setAcceleration(10000);
  motorY.setMaxSpeed(4000);
  motorY.setAcceleration(10000);

  deshabilitarMotores();

  Serial.println("Iniciando sistema. Buscando Home...");
  irAHome();

  Serial.println("Sistema listo.");
  Serial.println("Ingrese la celda de 2 digitos (FilaColumna). Ejemplo: 11, 42");
}

void loop() {
  leerCeldaSerial();
}