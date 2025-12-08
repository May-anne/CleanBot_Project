#include <ESP32Servo.h>

// ==================================================
// MOTORES
// ==================================================
#define IN1_A 27
#define IN2_A 14
#define EN_A  21
bool INV_A = true;

#define IN1_B 32
#define IN2_B 33
#define EN_B  22
bool INV_B = true;

#define IN1_C 18
#define IN2_C 19
#define EN_C  23
bool INV_C = true;

#define IN1_D 4
#define IN2_D 5
#define EN_D  13
bool INV_D = false;

// ==================================================
// SENSORES
// ==================================================
#define LEFT_ENCODER_PIN 35
#define RIGHT_ENCODER_PIN 26
#define TRIG 16
#define ECHO 17
#define SERVO 25

// ==================================================
// CONSTANTES
// ==================================================
const int CPR = 20;
const float wheel_diameter = 0.061;
const float wheel_circ = wheel_diameter * 3.14159265359;
const float PULSOS_90_ESQ = 25.0;
const float PULSOS_90_DIR = 24.0;
const float PULSO_POR_GRAU_ESQ = PULSOS_90_ESQ / 90.0;
const float PULSO_POR_GRAU_DIR = PULSOS_90_DIR / 90.0;

// Parâmetros de navegação
const int DIST_OBSTACULO = 25;      // cm - detectar obstáculo frontal
const int DIST_WALL_TARGET = 20;    // cm - distância alvo da parede
const int DIST_WALL_MIN = 12;       // cm - muito perto da parede
const int DIST_WALL_MAX = 35;       // cm - muito longe da parede
const int DIST_WALL_LOST = 80;      // cm - perdeu a parede
const int ANGLE_WALL = 60;          // graus - ângulo do sensor para parede

// ==================================================
// ESTADOS
// ==================================================
enum State {
  SPIRAL,
  WALL_FOLLOW,
  RANDOM_WALK,
  ESCAPE
};

State currentState = SPIRAL;

// ==================================================
// VARIÁVEIS GLOBAIS
// ==================================================
volatile long left_count = 0;
volatile long right_count = 0;
bool motorsEnabled = false;

// Espiral
int spiralRadius = 5;               // cm - raio inicial
int spiralIncrement = 2;            // cm - incremento por volta

// Wall follow
bool followingLeftWall = true;      // true = parede à esquerda
unsigned long wallLostTime = 0;
const unsigned long WALL_LOST_TIMEOUT = 3000; // ms

// Escape
unsigned long lastProgressTime = 0;
float lastPositionX = 0;
float lastPositionY = 0;

Servo servo;

// ==================================================
// ISRs e funções básicas
// ==================================================
void IRAM_ATTR leftISR()  { left_count++; }
void IRAM_ATTR rightISR() { right_count++; }

void resetEncoders() {
  left_count = 0;
  right_count = 0;
}

// Declarações
void enableMotors();
void disableMotors();
void driveMotor(int in1, int in2, int en, int speed, bool invert);
void setMotorLeft(int s);
void setMotorRight(int s);
void stopAll();
void walkDistance(float cm, int speed = 255);
void turnLeftAngle(float graus, int speed = 255);
void turnRightAngle(float graus, int speed = 255);
int readUltrasonic(int angle);

void spiralMode();
void wallFollowMode();
void randomWalkMode();
void escapeMode();

// ==================================================
// SETUP
// ==================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  servo.setPeriodHertz(50);
  servo.attach(SERVO, 500, 2400);
  servo.write(90);
  delay(500);

  pinMode(IN1_A, OUTPUT); pinMode(IN2_A, OUTPUT); pinMode(EN_A, OUTPUT);
  pinMode(IN1_B, OUTPUT); pinMode(IN2_B, OUTPUT); pinMode(EN_B, OUTPUT);
  pinMode(IN1_C, OUTPUT); pinMode(IN2_C, OUTPUT); pinMode(EN_C, OUTPUT);
  pinMode(IN1_D, OUTPUT); pinMode(IN2_D, OUTPUT); pinMode(EN_D, OUTPUT);

  digitalWrite(IN1_A, LOW); digitalWrite(IN2_A, LOW); digitalWrite(EN_A, LOW);
  digitalWrite(IN1_B, LOW); digitalWrite(IN2_B, LOW); digitalWrite(EN_B, LOW);
  digitalWrite(IN1_C, LOW); digitalWrite(IN2_C, LOW); digitalWrite(EN_C, LOW);
  digitalWrite(IN1_D, LOW); digitalWrite(IN2_D, LOW); digitalWrite(EN_D, LOW);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightISR, RISING);

  Serial.println("\n========================================");
  Serial.println("   ALGORITMO AVANÇADO ATIVADO");
  Serial.println("   Estado inicial: SPIRAL");
  Serial.println("========================================\n");
  
  delay(2000);
}

// ==================================================
// LOOP PRINCIPAL
// ==================================================
void loop() {
  switch(currentState) {
    case SPIRAL:
      spiralMode();
      break;
    case WALL_FOLLOW:
      wallFollowMode();
      break;
    case RANDOM_WALK:
      randomWalkMode();
      break;
    case ESCAPE:
      escapeMode();
      break;
  }
  delay(50);
}

// ==================================================
// MODO SPIRAL
// ==================================================
void spiralMode() {
  Serial.println(">>> MODO SPIRAL <<<");
  
  disableMotors();
  servo.write(90);
  delay(200);

  int distFront = readUltrasonic(90);
  Serial.print("distFront: ");
  Serial.println(distFront);

  if (distFront > 0 && distFront < DIST_OBSTACULO) {
    Serial.println("Obstáculo detectado! Transição para WALL_FOLLOW");
    
    // Backup
    enableMotors();
    setMotorLeft(-200);
    setMotorRight(-200);
    delay(300);
    stopAll();
    disableMotors();
    
    // Escolher lado baseado em scan
    servo.write(90 + ANGLE_WALL);
    delay(300);
    int distLeft = readUltrasonic(90 + ANGLE_WALL);
    
    servo.write(90 - ANGLE_WALL);
    delay(300);
    int distRight = readUltrasonic(90 - ANGLE_WALL);
    
    followingLeftWall = (distLeft < distRight);
    Serial.print("Seguindo parede à ");
    Serial.println(followingLeftWall ? "ESQUERDA" : "DIREITA");
    
    currentState = WALL_FOLLOW;
    wallLostTime = millis();
    return;
  }
  
  // Movimento em espiral
  enableMotors();
  
  // Andar em arco
  int speedOuter = 255;
  int speedInner = 200 - (spiralRadius * 3); // Diferencial aumenta com raio
  if (speedInner < 150) speedInner = 150;
  
  setMotorLeft(speedOuter);
  setMotorRight(speedInner);
  delay(400);
  
  stopAll();
  disableMotors();
  
  spiralRadius += spiralIncrement;
  if (spiralRadius > 30) spiralRadius = 5; // Reset
}

// ==================================================
// MODO WALL_FOLLOW
// ==================================================
void wallFollowMode() {
  Serial.println(">>> MODO WALL_FOLLOW <<<");
  
  disableMotors();
  
  // Verificar frente
  servo.write(90);
  delay(200);
  int distFront = readUltrasonic(90);
  
  if (distFront > 0 && distFront < DIST_OBSTACULO) {
    Serial.println("Obstáculo frontal! Escape...");
    
    enableMotors();
    // Backup
    setMotorLeft(-200);
    setMotorRight(-200);
    delay(400);
    stopAll();
    
    // Giro para longe da parede
    if (followingLeftWall) {
      turnRightAngle(random(60, 100), 255);
    } else {
      turnLeftAngle(random(60, 100), 255);
    }
    disableMotors();
    return;
  }
  
  // Medir distância lateral
  int sensorAngle = followingLeftWall ? (90 + ANGLE_WALL) : (90 - ANGLE_WALL);
  servo.write(sensorAngle);
  delay(250);
  int distWall = readUltrasonic(sensorAngle);
  
  Serial.print("Distância da parede: ");
  Serial.println(distWall);
  
  // Verificar se perdeu a parede
  if (distWall == 0 || distWall > DIST_WALL_LOST) {
    if (wallLostTime == 0) {
      wallLostTime = millis();
    } else if (millis() - wallLostTime > WALL_LOST_TIMEOUT) {
      Serial.println("Parede perdida! Transição para RANDOM_WALK");
      currentState = RANDOM_WALK;
      return;
    }
  } else {
    wallLostTime = 0;
  }
  
  enableMotors();
  
  // Controle proporcional da distância
  int erro = distWall - DIST_WALL_TARGET;
  int speedBase = 220;
  int correcao = erro * 3;
  
  // Escape se muito perto
  if (distWall > 0 && distWall < DIST_WALL_MIN) {
    Serial.println("MUITO PERTO! Escape rápido");
    setMotorLeft(-200);
    setMotorRight(-200);
    delay(300);
    stopAll();
    
    if (followingLeftWall) {
      turnRightAngle(30, 255);
    } else {
      turnLeftAngle(30, 255);
    }
    disableMotors();
    return;
  }
  
  // Ajustar velocidades
  int speedLeft, speedRight;
  if (followingLeftWall) {
    speedLeft = speedBase - correcao;
    speedRight = speedBase + correcao;
  } else {
    speedLeft = speedBase + correcao;
    speedRight = speedBase - correcao;
  }
  
  speedLeft = constrain(speedLeft, 150, 255);
  speedRight = constrain(speedRight, 150, 255);
  
  setMotorLeft(speedLeft);
  setMotorRight(speedRight);
  delay(300);
  
  stopAll();
  disableMotors();
}

// ==================================================
// MODO RANDOM_WALK
// ==================================================
void randomWalkMode() {
  Serial.println(">>> MODO RANDOM_WALK <<<");
  
  disableMotors();
  servo.write(90);
  delay(200);
  int distFront = readUltrasonic(90);

  Serial.print("distFront = ");
  Serial.println(distFront);
  
  if (distFront > 0 && distFront < DIST_OBSTACULO) {
    Serial.println("Obstáculo! Backup e giro aleatório");
    
    enableMotors();
    
    // Backup proporcional
    float backupDist = random(8, 15);
    walkDistance(backupDist, -200);
    
    // Giro aleatório
    int turnAngle = random(60, 150);
    if (random(0, 2) == 0) {
      turnLeftAngle(turnAngle, 255);
    } else {
      turnRightAngle(turnAngle, 255);
    }
    
    disableMotors();
    return;
  }
  
  // Movimento reto
  enableMotors();
  
  float distToWalk;
  if (distFront == 0 || distFront > 80) {
    distToWalk = 20;
  } else {
    distToWalk = min(15.0f, (float)(distFront - 10));
  }
  
  walkDistance(distToWalk, 255);
  
  stopAll();
  disableMotors();
  
  // Checar se deve entrar em ESCAPE
  // (implementação simplificada - pode melhorar com odometria real)
  if (random(0, 20) == 0) {
    Serial.println("Verificação aleatória → ESCAPE");
    currentState = ESCAPE;
  }
}

// ==================================================
// MODO ESCAPE
// ==================================================
void escapeMode() {
  Serial.println(">>> MODO ESCAPE <<<");
  
  enableMotors();
  
  // Ré longa
  Serial.println("Ré longa...");
  setMotorLeft(-220);
  setMotorRight(-220);
  delay(800);
  stopAll();
  disableMotors();
  
  // Varredura angular
  Serial.println("Varredura...");
  int bestAngle = 90;
  int maxDist = 0;
  
  for (int angle = 0; angle <= 180; angle += 30) {
    servo.write(angle);
    delay(250);
    int dist = readUltrasonic(angle);
    Serial.print("Ângulo ");
    Serial.print(angle);
    Serial.print("°: ");
    Serial.println(dist);
    
    if (dist > maxDist) {
      maxDist = dist;
      bestAngle = angle;
    }
  }
  
  Serial.print("Melhor direção: ");
  Serial.println(bestAngle);
  
  // Girar para direção mais livre
  enableMotors();
  int turnNeeded = bestAngle - 90;
  if (turnNeeded > 0) {
    turnLeftAngle(abs(turnNeeded), 255);
  } else if (turnNeeded < 0) {
    turnRightAngle(abs(turnNeeded), 255);
  }
  disableMotors();
  
  Serial.println("Retornando para RANDOM_WALK");
  currentState = RANDOM_WALK;
}

// ==================================================
// FUNÇÕES DE CONTROLE
// ==================================================
void enableMotors() {
  if (motorsEnabled) return;
  servo.detach();
  delay(50);
  ledcAttach(EN_A, 5000, 8);
  ledcAttach(EN_B, 5000, 8);
  ledcAttach(EN_C, 5000, 8);
  ledcAttach(EN_D, 5000, 8);
  motorsEnabled = true;
}

void disableMotors() {
  if (!motorsEnabled) return;
  stopAll();
  delay(50);
  ledcDetach(EN_A);
  ledcDetach(EN_B);
  ledcDetach(EN_C);
  ledcDetach(EN_D);
  pinMode(EN_A, OUTPUT); digitalWrite(EN_A, LOW);
  pinMode(EN_B, OUTPUT); digitalWrite(EN_B, LOW);
  pinMode(EN_C, OUTPUT); digitalWrite(EN_C, LOW);
  pinMode(EN_D, OUTPUT); digitalWrite(EN_D, LOW);
  motorsEnabled = false;
  delay(50);
  servo.attach(SERVO, 500, 2400);
  delay(100);
}

void driveMotor(int in1, int in2, int en, int speed, bool invert) {
  if (!motorsEnabled) return;
  if (invert) speed = -speed;
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(en, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(en, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(en, 0);
  }
}

void setMotorLeft(int s) {
  driveMotor(IN1_B, IN2_B, EN_B, s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C, s, INV_C);
}

void setMotorRight(int s) {
  driveMotor(IN1_A, IN2_A, EN_A, s, INV_A);
  driveMotor(IN1_D, IN2_D, EN_D, s, INV_D);
}

void stopAll() {
  if (!motorsEnabled) return;
  digitalWrite(IN1_A, LOW); digitalWrite(IN2_A, LOW); ledcWrite(EN_A, 0);
  digitalWrite(IN1_B, LOW); digitalWrite(IN2_B, LOW); ledcWrite(EN_B, 0);
  digitalWrite(IN1_C, LOW); digitalWrite(IN2_C, LOW); ledcWrite(EN_C, 0);
  digitalWrite(IN1_D, LOW); digitalWrite(IN2_D, LOW); ledcWrite(EN_D, 0);
}

void walkDistance(float cm, int speed) {
  if (!motorsEnabled) return;
  float metros = abs(cm) / 100.0;
  float rot = metros / wheel_circ;
  long alvo = rot * CPR;
  
  resetEncoders();
  
  if (speed >= 0) {
    setMotorLeft(speed);
    setMotorRight(speed);
  } else {
    setMotorLeft(speed);
    setMotorRight(speed);
  }
  
  while ((left_count + right_count) / 2 < alvo) {
    delay(1);
  }
  stopAll();
}

void turnLeftAngle(float graus, int speed) {
  if (!motorsEnabled) return;
  resetEncoders();
  float alvo = graus * PULSO_POR_GRAU_ESQ;
  setMotorLeft(-speed);
  setMotorRight(speed);
  while (abs(right_count) < alvo) {
    delay(1);
  }
  stopAll();
}

void turnRightAngle(float graus, int speed) {
  if (!motorsEnabled) return;
  resetEncoders();
  float alvo = graus * PULSO_POR_GRAU_DIR;
  setMotorLeft(speed);
  setMotorRight(-speed);
  while (abs(left_count) < alvo) {
    delay(1);
  }
  stopAll();
}

int readUltrasonic(int angle) {
  servo.write(angle);
  delay(150);
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  return distance;
}