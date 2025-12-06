#include <Ultrasonic.h>
#include <ESP32Servo.h>
// ==================================================
// MOTORES
// ==================================================
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

// ==================================================
// ENCODERS
// ==================================================
#define LEFT_ENCODER_PIN 25
#define RIGHT_ENCODER_PIN 26

// ==================================================
// SENSOR ULTRASSÔNICO
// ==================================================
#define TRIG 16
#define ECHO 17

// ==================================================
// MOTOR SERVO
// ==================================================
#define SERVO 2

// ==================================================
// CONSTANTES
// ==================================================
//RODA
const int CPR = 20;                     // furos do encoder
const float wheel_diameter = 0.065;     // 6.5cm
float wheel_circ = wheel_diameter * 3.14159265359;
//ENCODER
const float PULSOS_90_ESQ = 25.0;
const float PULSOS_90_DIR = 24.0;
const float PULSO_POR_GRAU_ESQ = PULSOS_90_ESQ / 90.0;
const float PULSO_POR_GRAU_DIR = PULSOS_90_DIR / 90.0;

// ==================================================
// VARIÁVEIS
// ==================================================
volatile long left_count = 0;
volatile long right_count = 0;

void IRAM_ATTR leftISR()  { left_count++; }
void IRAM_ATTR rightISR() { right_count++; }

void resetEncoders() {
  left_count = 0;
  right_count = 0;
}

void driveMotor(int in1, int in2, int en, int speed, bool invert);
void setMotorLeft(int s);
void setMotorRight(int s);
void forward(int s = 255);
void backward(int s = 255);
void stopAll();
void turnRightAngle(float graus, int speed = 200);
void turnLeftAngle(float graus, int speed = 200); 
void andarDistancia(float cm, int speed = 200);

// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);

  pinMode(IN1_A, OUTPUT); pinMode(IN2_A, OUTPUT); pinMode(EN_A, OUTPUT);
  pinMode(IN1_B, OUTPUT); pinMode(IN2_B, OUTPUT); pinMode(EN_B, OUTPUT);
  pinMode(IN1_C, OUTPUT); pinMode(IN2_C, OUTPUT); pinMode(EN_C, OUTPUT);
  pinMode(IN1_D, OUTPUT); pinMode(IN2_D, OUTPUT); pinMode(EN_D, OUTPUT);

  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightISR, RISING);

  stopAll();

  Serial.println("Setup realizado");
}

// ==================================================
// LOOP
// ==================================================

void loop() {
  //fazendo testes
  delay(2000);

  andarDistancia(30);
  turnLeftAngle(90);

  andarDistancia(30);
  turnLeftAngle(180);
  while (1);
}

// ==================================================
// FUNÇÃO PARA DIRIGIR MOTOR
// ==================================================

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

// ==================================================
// CONTROLE DOS 4 MOTORES COMO UM PAR ESQUERDA/DIREITA
// ==================================================

void setMotorLeft(int s) {
  driveMotor(IN1_B, IN2_B, EN_B, s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C, s, INV_C);
}

void setMotorRight(int s) {
  driveMotor(IN1_A, IN2_A, EN_A, s, INV_A);
  driveMotor(IN1_D, IN2_D, EN_D, s, INV_D);
}

void forward(int s) {
  setMotorLeft(s);
  setMotorRight(s);
}

void backward(int s) {
  setMotorLeft(-s);
  setMotorRight(-s);
}

void stopAll() {
  setMotorLeft(0);
  setMotorRight(0);
}

// ==================================================
// ANDAR X DISTÂNCIA
// ==================================================

void andarDistancia(float cm, int speed) {

  float metros = cm / 100.0;
  float rot = metros / wheel_circ;
  long alvo = rot * CPR;

  resetEncoders();
  forward(speed);

  while ((left_count + right_count) / 2 < alvo) {}

  stopAll();
}

void turnLeftAngle(float graus, int speed) {

  resetEncoders();
  float alvo = graus * PULSO_POR_GRAU_ESQ;

  // esquerda para trás, direita para frente
  setMotorLeft(-speed);
  setMotorRight(speed);

  while ((abs(left_count) + abs(right_count)) / 2.0 < alvo) {}

  stopAll();
}

void turnRightAngle(float graus, int speed) {

  resetEncoders();
  float alvo = graus * PULSO_POR_GRAU_DIR;

  // esquerda para frente, direita para trás
  setMotorLeft(speed);
  setMotorRight(-speed);

  while ((abs(left_count) + abs(right_count)) / 2.0 < alvo) {}

  stopAll();
}

/*void turnLeft(int s = 255) {
  driveMotor(IN1_A, IN2_A, EN_A,  s, INV_A);
  driveMotor(IN1_D, IN2_D, EN_D,  s, INV_D);
  driveMotor(IN1_B, IN2_B, EN_B, -s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C, -s, INV_C);
}

void turnRight(int s = 255) {
  driveMotor(IN1_A, IN2_A, EN_A, -s, INV_A);
  driveMotor(IN1_D, IN2_D, EN_D, -s, INV_D);
  driveMotor(IN1_B, IN2_B, EN_B,  s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C,  s, INV_C);
}

void debugPulsosGiro(int tempo_ms = 1000, int speed = 150) {

    Serial.println("\n=== DEBUG DE GIRO ===");

    // -------- GIRO PARA A ESQUERDA --------
    left_count = 0;
    right_count = 0;

    Serial.println("Girando para a ESQUERDA...");
    turnLeft(speed);
    delay(tempo_ms);
    stopAll();

    Serial.print("Pulsos esquerda (left_count): ");
    Serial.println(left_count);
    Serial.print("Pulsos direita (right_count): ");
    Serial.println(right_count);

    long mediaLeft = abs(left_count);
    long mediaRight = abs(right_count);
    long mediaEsq = (mediaLeft + mediaRight) / 2;

    Serial.print("Média de pulsos (esquerda): ");
    Serial.println(mediaEsq);

    Serial.println("PAUSA");
    delay(2000);

    // -------- GIRO PARA A DIREITA --------
    left_count = 0;
    right_count = 0;

    Serial.println("\nGirando para a DIREITA...");
    turnRight(speed);
    delay(tempo_ms);
    stopAll();

    Serial.print("Pulsos esquerda (left_count): ");
    Serial.println(left_count);
    Serial.print("Pulsos direita (right_count): ");
    Serial.println(right_count);

    mediaLeft = abs(left_count);
    mediaRight = abs(right_count);
    long mediaDir = (mediaLeft + mediaRight) / 2;

    Serial.print("Média de pulsos (direita): ");
    Serial.println(mediaDir);

    Serial.println("\nFIM DO DEBUG\n");
}*/

