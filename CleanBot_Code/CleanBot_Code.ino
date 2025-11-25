#include <Ultrasonic.h>
#include <ESP32Servo.h>

#define trigger 5
#define echo 18
#define servoPin 4

#define IN1 25
#define IN2 26
#define ENA 32

#define IN3 27
#define IN4 14
#define ENB 33

#define DistMin 20

#define ultrasonicInterval 1000

Ultrasonic ultrassom(trigger, echo);
Servo servo;

int pos;
String flag = "";
bool started = 1;
String previousDirection = "Left";
long distR, distL, distF;
unsigned long lastMoveTime = 0;
float valuesCM[3];

void forward(int velocidade = 200);
void turnLeft(int velocidade = 200);
void turnRight(int velocidade = 200);
void stopMotors();
float* moveScanner();
long readUltrasonic();

void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  servo.attach(servoPin);
  servo.write(90);

  stopMotors();
}

void loop() {
  float* direction = moveScanner();

  float maxDir1 = max(direction[0], direction[1]);
  float maxDir = max(maxDir1, direction[2]);

  if (started == 1 || 
     (previousDirection == "Left" && direction[0] < DistMin) || 
     (previousDirection == "Right" && direction[2] < DistMin)) { 

    if (maxDir == direction[1]) flag = "Forward";
    else if (maxDir == direction[0]) flag = "Right";
    else flag = "Left";

    started = 0;
  } else {
    flag = previousDirection;
  }

  if (millis() - lastMoveTime > 2000) {
    if (flag == "Left") turnLeft();
    else if (flag == "Right") turnRight();
    else forward();
    lastMoveTime = millis();
  }

  stopMotors();
  delay(100);
}

long readUltrasonic() {
  long cm = ultrassom.read(CM);
  Serial.print("Distancia (cm): ");
  Serial.println(cm);
  return cm;
}

float* moveScanner() {
  // Direita
  servo.write(0);
  delay(500);
  distR = readUltrasonic();

  // Frente
  servo.write(90);
  delay(500);
  distF = readUltrasonic();

  // Esquerda
  servo.write(180);
  delay(500);
  distL = readUltrasonic();

  // Volta ao centro
  servo.write(90);
  delay(200);

  valuesCM[0] = distR;
  valuesCM[1] = distF;
  valuesCM[2] = distL;

  return valuesCM;
}

void forward(int velocidade) {
  Serial.println("Seguindo em frente");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velocidade);
  analogWrite(ENB, velocidade);
}

void turnLeft(int velocidade) {
  Serial.println("Virando à esquerda");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velocidade);
  analogWrite(ENB, velocidade);
}

void turnRight(int velocidade) {
  Serial.println("Virando à direita");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, velocidade);
  analogWrite(ENB, velocidade);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
