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
    Serial.println("Error: Celda fuera de rango (Filas: 1-5, Columnas: 1-6)");
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

  pinMode(LIMIT_SWITCH_CNC_X, INPUT_PULLDOWN);
  pinMode(LIMIT_SWITCH_CNC_Y, INPUT_PULLDOWN);

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
  Serial.println("Ingrese la celda de 2 digitos (ColumnaFila).");
}

void loop() {
  leerCeldaSerial();
}