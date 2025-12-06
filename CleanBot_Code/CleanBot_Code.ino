#include <Ultrasonic.h>
#include <ESP32Servo.h>

// ---- DIREITA TRASEIRA (A)
#define IN1_A 27
#define IN2_A 14
#define EN_A  13
bool INV_A = true;

// ---- ESQUERDA TRASEIRA (B)
#define IN1_B 32
#define IN2_B 33
#define EN_B  12
bool INV_B = true;

// ---- ESQUERDA DIANTEIRA (C)
#define IN1_C 18
#define IN2_C 19
#define EN_C  23
bool INV_C = true;

// ---- DIREITA DIANTEIRA (D)
#define IN1_D 4
#define IN2_D 5
#define EN_D  15
bool INV_D = false;

// ---- ULTRASSÔNICO
#define trigger 5
#define echo 18

// ---- SERVO MOTOR
#define servoPin 4

// ---- CONSTANTES
#define DistMin 20
#define ultrasonicInterval 1000

// ---- OBJETOS
Ultrasonic ultrassom(trigger, echo);
Servo servo;

// ---- VARIÁVEIS
int pos;
bool started = 1;
float valuesCM[3];
long distR, distL, distF;
unsigned long lastMoveTime = 0;
String flag = "";
String previousDirection = "Left";

// ---- DEFINIÇÃO DE FUNÇÕES
void turnRight(int s = 255);
void turnLeft(int s = 255);
void forward(int s = 255);
void backward(int s = 255);
void stopAll();

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando serial...");

  pinMode(IN1_A, OUTPUT); pinMode(IN2_A, OUTPUT); pinMode(EN_A, OUTPUT);
  pinMode(IN1_B, OUTPUT); pinMode(IN2_B, OUTPUT); pinMode(EN_B, OUTPUT);
  pinMode(IN1_C, OUTPUT); pinMode(IN2_C, OUTPUT); pinMode(EN_C, OUTPUT);
  pinMode(IN1_D, OUTPUT); pinMode(IN2_D, OUTPUT); pinMode(EN_D, OUTPUT);

  servo.attach(servoPin);
  servo.write(90);

  stopAll();
  Serial.println("Robô pronto!");
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

  stopAll();
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

void driveMotor(int in1, int in2, int en, int speed, bool invert)
{
  if (invert) speed = -speed;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, speed);
  }
  else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -speed);
  }
  else { 
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  }
}

void stopAll() {
  driveMotor(IN1_A, IN2_A, EN_A, 0, INV_A);
  driveMotor(IN1_B, IN2_B, EN_B, 0, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C, 0, INV_C);
  driveMotor(IN1_D, IN2_D, EN_D, 0, INV_D);
}

void forward(int s) {
  Serial.println("Frente");
  driveMotor(IN1_A, IN2_A, EN_A,  s, INV_A);
  driveMotor(IN1_B, IN2_B, EN_B,  s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C,  s, INV_C);
  driveMotor(IN1_D, IN2_D, EN_D,  s, INV_D);
}

void backward(int s) {
  driveMotor(IN1_A, IN2_A, EN_A, -s, INV_A);
  driveMotor(IN1_B, IN2_B, EN_B, -s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C, -s, INV_C);
  driveMotor(IN1_D, IN2_D, EN_D, -s, INV_D);
}

void turnLeft(int s) {
  Serial.println("Esquerda");
  // direita pra frente, esquerda pra trás
  driveMotor(IN1_A, IN2_A, EN_A,  s, INV_A);
  driveMotor(IN1_D, IN2_D, EN_D,  s, INV_D);
  driveMotor(IN1_B, IN2_B, EN_B, -s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C, -s, INV_C);
}

void turnRight(int s) {
  Serial.println("Direita");
  // esquerda pra frente, direita pra trás
  driveMotor(IN1_A, IN2_A, EN_A, -s, INV_A);
  driveMotor(IN1_D, IN2_D, EN_D, -s, INV_D);
  driveMotor(IN1_B, IN2_B, EN_B,  s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C,  s, INV_C);
}

