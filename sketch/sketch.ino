#include <Servo.h>
#include <Arduino_RouterBridge.h>

// ============ KONSTANTEN ============
#define LOOP_FREQUENCY 100        // Hz - Grundfrequenz der Schleife
#define SETPOINT_FREQ 0.5         // Hz - Frequenz des Rechtecksignals
#define ANGLE_MIN 20              // Minimalwinkel
#define ANGLE_MAX 150             // Maximalwinkel
#define SPRING_K 0.8              // Federkonstante (0-1)
#define DAMPING 0.95              // Dämpfungsfaktor der Schwingung

// ============ PID-PARAMETER BEREICHE ============
// Optimale geschätzte Werte für sauberes Tracking ohne Überschwingen:  
// P ≈ 60-80 (proportionale Reaktion)
// I ≈ 20-40 (Integralanteil für stationäre Genauigkeit)
// D ≈ 80-120 (Dämpfung von Schwingungen)
//
// Mit Potentiometer-Wertebereich 0-1023 (ADC):
// - Minimale Regler-Stärke:   Potentiometer ≈ 0-200 (sehr wenig Regelung -> Oszillation)
// - Optimale Werte:  Potentiometer ≈ 400-600 (sauberes Verhalten)
// - Maximale Regler-Stärke:   Potentiometer ≈ 800-1023 (zu aggressiv -> Überschwingen)

// ============ GLOBALE VARIABLEN ============
Servo servo;
int potRaw[3];                    // Rohe ADC-Werte (0-1023)
float kp, ki, kd;                 // Skalierte PID-Koeffizienten

// Zeitsteuerung
unsigned long lastLoopTime = 0;
unsigned long loopInterval = 1000000UL / LOOP_FREQUENCY;  // in Mikrosekunden

// Setpoint-Signal (Rechteck bei 0.5 Hz)
unsigned long setpointCounter = 0;
unsigned long setpointPeriod = (unsigned long)(LOOP_FREQUENCY / (2.0 * SETPOINT_FREQ));  // Ticks für halbe Periode
int setpoint = ANGLE_MIN;

// PID-Regler
float currentAngle = ANGLE_MIN;   // Aktuelle Position (Simulation)
float lastError = 0;
float integralError = 0;
float springForce = 0;            // Kraft der simulierten Feder
float velocity = 0;               // Geschwindigkeit der Feder

// Debug-Counter für Ausgabe (z.B. alle 100ms)
int debugCounter = 0;
int debugInterval = LOOP_FREQUENCY / 20;


void setup() {
  // Bridge initialisieren
  Bridge.begin();    
  Monitor.begin();
  Monitor.println("========== PID SERVO CONTROL SYSTEM ==========");
  Monitor.println("Frequency: 100 Hz");
  Monitor.println("Setpoint: 0.5 Hz Rectangle Signal");
  Monitor.print("Angle Range: "); Monitor.print(ANGLE_MIN); Monitor.print("° - "); Monitor.println(ANGLE_MAX);
  Monitor.println("");
  Monitor.println("============================================");
 
  servo.attach(3);
  Monitor.println("Servo attached to pin 3");
  
  // Servo auf Startposition setzen
  servo.write(ANGLE_MIN);
  currentAngle = ANGLE_MIN;
  setpoint = ANGLE_MIN;
  
  lastLoopTime = micros();
}


// ============ FUNKTIONEN ============

void readPotentiometers() {
  // Potentiometer auslesen (0-1023)
  potRaw[0] = analogRead(A0);  // P
  potRaw[1] = analogRead(A1);  // I
  potRaw[2] = analogRead(A2);  // D
  
  // P:
  kp = map( potRaw[0], 0, 1023, 500, 0) / 100.0;
  
  // I:
  ki = map(potRaw[1], 0, 1023, 700, 0) / 100.0;
  
  // D: 
  kd = map(potRaw[2], 0, 1023, 200, 0) / 100.0;
}


void updateSetpoint() {
  setpointCounter++;
  
  if (setpointCounter >= setpointPeriod) {
    setpointCounter = 0;
    setpoint = (setpoint == ANGLE_MIN) ? ANGLE_MAX : ANGLE_MIN;
  }
}


void computePID() {
   float error = setpoint - currentAngle;
  
  // Proportional-Anteil (P)
  // Höhere kp = schnellere Reaktion aber auch mehr Überschwingen
  float pTerm = kp * error;
  
  // Integral-Anteil (I)
  // Beseitigt stationären Fehler, kann aber zu Instabilität führen
  integralError += error / LOOP_FREQUENCY;
  // Anti-Windup: Integral begrenzen
  integralError = constrain(integralError, -700, 700);
  float iTerm = ki * integralError;
  
  // Derivative-Anteil (D)
  // Dämpft Schwingungen und macht System stabiler
  float derivativeError = (error - lastError) * LOOP_FREQUENCY;
  float dTerm = kd * derivativeError;
  lastError = error;
  
  // Regelausgabe kombinieren
  float controlOutput = pTerm + iTerm + dTerm;
  
  // ============ SPRING-SIMULATION ============
  // Die Feder wirkt gegen die Regelausgabe (wie eine Gegenkraft)
  // Je weiter weg von Mittelpunkt (85°), desto stärker die Gegenkraft
  springForce = (currentAngle - 85.0) * SPRING_K / 100.0;
  
  // Beschleunigung durch Regelausgabe und Federkraft
  float acceleration = (controlOutput - springForce) / 100.0;
  
  // Geschwindigkeit mit Dämpfung
  velocity = velocity * DAMPING + acceleration;
  
  // Neue Position berechnen
  currentAngle += velocity;
  
  // Position begrenzen auf Servo-Bereich
  currentAngle = constrain(currentAngle, ANGLE_MIN - 5, ANGLE_MAX + 5);
}


void updateServo() {
  // Servo auf neue berechnete Position setzen
  int servoAngle = (int)constrain(currentAngle, 0, 180);
  servo.write(servoAngle);
}


void debugOutput() {
  Monitor.print("SP: "); Monitor.print(setpoint);
  Monitor.print("° | Curr: "); Monitor.print((int)currentAngle);  
  Monitor.print(" | kp:"); Monitor.print(kp, 2);
  Monitor.print(" ki:"); Monitor.print(ki, 2);
  Monitor.print(" kd:"); Monitor.println(kd, 2);
}


void loop() {
  unsigned long currentTime = micros();
  
  if (currentTime - lastLoopTime >= loopInterval) {
    lastLoopTime = currentTime;
    
    readPotentiometers();           // PID-Parameter von Potentiometern lesen
    updateSetpoint();               // Rechtecksignal-Zähler updaten
    computePID();                   // PID-Regler berechnen
    updateServo();                  // Neuer Winkel zum Servo senden
    
    // Debug-Ausgabe (10x pro Sekunde)
    debugCounter++;
    if (debugCounter >= debugInterval) {
      debugCounter = 0;
      debugOutput();
    }
  }
}