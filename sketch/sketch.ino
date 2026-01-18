#include <Servo.h>
#include <Arduino_RouterBridge.h>

Servo servo;

int potentiometer[3];

void setup() {
  // Initialize the Bridge
  Bridge.begin();    
  Monitor.begin();
  Monitor.println("Monitor ready");
 
  servo.attach(3);
  Monitor.println("Servo attached");
}

void readPid(int pot[]){
  pot[0] = map(analogRead(A0), 1023, 0, 0, 180);
  pot[1] = map(analogRead(A1), 1023, 0, 0, 180);
  pot[2] = map(analogRead(A2), 1023, 0, 0, 180);
}

void printPid(int pot[]) {
  Monitor.print("P="); Monitor.print(pot[0]); Monitor.print(", ");
  Monitor.print("I="); Monitor.print(pot[1]); Monitor.print(", ");
  Monitor.print("D="); Monitor.println(pot[2]);
}


void loop() {  
  Monitor.print("Sensor reading: ");
  readPid (potentiometer);
  printPid(potentiometer);    

  // servo.write(map(analogRead(A0), 1024, 0, 0, 180));
  delay(1000); // Eine Sekunde warten
}
