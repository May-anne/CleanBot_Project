#include <ESP32Servo.h>

// ==================================================
// MOTORES
// ==================================================
#define IN1_A 27
#define IN2_A 14
bool INV_A = true;

#define IN1_B 32
#define IN2_B 33
bool INV_B = true;

#define IN1_C 18
#define IN2_C 19
bool INV_C = true;

#define IN1_D 4
#define IN2_D 5
bool INV_D = false;

#define FAN_IN1 13
#define FAN_IN2 22    

// ==================================================
// SENSORES
// ==================================================
#define LEFT_ENCODER_PIN 35
#define RIGHT_ENCODER_PIN 26
#define TRIG 16
#define ECHO 17
#define SERVO 21
#define BUZZER_PIN 23

// ==================================================
// CONSTANTES
// ==================================================
const int CPR = 20;
const float wheel_diameter = 0.065;
const float wheel_circ = wheel_diameter * 3.14159265359;
const float PULSOS_90_ESQ = 25.0;
const float PULSOS_90_DIR = 24.0;
const float PULSO_POR_GRAU_ESQ = PULSOS_90_ESQ / 90.0;
const float PULSO_POR_GRAU_DIR = PULSOS_90_DIR / 90.0;

const int DIST_OBSTACULO = 25;
const int DIST_WALL_TARGET = 20;
const int DIST_WALL_MIN = 12;
const int DIST_WALL_MAX = 35;
const int DIST_WALL_LOST = 80;
const int ANGLE_WALL = 60;

const int FAN_FREQ = 20000;  
const int FAN_RES = 8;  

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

int spiralRadius = 5;
int spiralIncrement = 2;

bool followingLeftWall = true;
unsigned long wallLostTime = 0;
const unsigned long WALL_LOST_TIMEOUT = 3000;

bool fanEnabled = false;


// Detecção de travamento
struct StuckDetector {
  long totalPulses;
  unsigned long lastCheckTime;
  unsigned long stuckStartTime;
  bool isStuck;
  int stuckCount;
  const long MIN_PULSES_PER_SECOND = 5;
  const unsigned long CHECK_INTERVAL = 1000;
  const unsigned long STUCK_TIMEOUT = 3000;
  const int MAX_STUCK_BEFORE_ALARM = 2;
} stuckDetector = {0, 0, 0, false, 0};

// Memória espacial
struct Location {
  int signature;
  unsigned long lastVisit;
  int visitCount;
};

#define MAX_LOCATIONS 20
struct SpatialMemory {
  Location locations[MAX_LOCATIONS];
  int count;
  unsigned long lastSignature;
  int repeatedSignatures;
  const unsigned long MEMORY_TIMEOUT = 30000;
  const int MAX_VISITS = 3;
  const int SIGNATURE_REPEAT_THRESHOLD = 4;
} spatialMemory = {{}, 0, 0, 0};

Servo servo;

// ==================================================
// ISRs
// ==================================================
void IRAM_ATTR leftISR()  { left_count++; }
void IRAM_ATTR rightISR() { right_count++; }

void resetEncoders() {
  left_count = 0;
  right_count = 0;
}

// ==================================================
// FUNÇÕES AUXILIARES
// ==================================================
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

void enableMotors() {
  if (motorsEnabled) return;
  servo.detach();
  delay(50);
  motorsEnabled = true;
}

void disableMotors() {
  if (!motorsEnabled) return;
  digitalWrite(IN1_A, LOW); 
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, LOW); 
  digitalWrite(IN2_B, LOW);
  digitalWrite(IN1_C, LOW); 
  digitalWrite(IN2_C, LOW);
  digitalWrite(IN1_D, LOW); 
  digitalWrite(IN2_D, LOW);
  motorsEnabled = false;
  delay(50);
  servo.attach(SERVO, 500, 2400);
  delay(100);
}

void driveMotor(int in1, int in2, int speed, bool invert) {
  if (!motorsEnabled) return;
  if (invert) speed = -speed;
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setMotorLeft(int s) {
  driveMotor(IN1_B, IN2_B, s, INV_B);
  driveMotor(IN1_C, IN2_C, s, INV_C);
}

void setMotorRight(int s) {
  driveMotor(IN1_A, IN2_A, s, INV_A);
  driveMotor(IN1_D, IN2_D, s, INV_D);
}

void stopAll() {
  if (!motorsEnabled) return;
  digitalWrite(IN1_A, LOW); 
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, LOW); 
  digitalWrite(IN2_B, LOW);
  digitalWrite(IN1_C, LOW); 
  digitalWrite(IN2_C, LOW);
  digitalWrite(IN1_D, LOW); 
  digitalWrite(IN2_D, LOW);
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

// ==================================================
// SISTEMA DE DETECÇÃO
// ==================================================
void checkIfStuck() {
  unsigned long currentTime = millis();
  if (currentTime - stuckDetector.lastCheckTime >= stuckDetector.CHECK_INTERVAL) {
    long currentPulses = abs(left_count) + abs(right_count);
    long pulsesDelta = currentPulses - stuckDetector.totalPulses;
    
    if (pulsesDelta < stuckDetector.MIN_PULSES_PER_SECOND) {
      if (stuckDetector.stuckStartTime == 0) {
        stuckDetector.stuckStartTime = currentTime;
      } else if (currentTime - stuckDetector.stuckStartTime >= stuckDetector.STUCK_TIMEOUT) {
        if (!stuckDetector.isStuck) {
          stuckDetector.isStuck = true;
          stuckDetector.stuckCount++;
          Serial.println("🚨 ROBÔ TRAVADO!");
          if (stuckDetector.stuckCount >= stuckDetector.MAX_STUCK_BEFORE_ALARM) {
            // Alarme
            for (int i = 0; i < 5; i++) {
              digitalWrite(BUZZER_PIN, HIGH);
              delay(200);
              digitalWrite(BUZZER_PIN, LOW);
              delay(200);
            }
            stuckDetector.stuckCount = 0;
          }
          stopAll();
          disableMotors();
          currentState = ESCAPE;
        }
      }
    } else {
      stuckDetector.stuckStartTime = 0;
      stuckDetector.isStuck = false;
    }
    stuckDetector.totalPulses = currentPulses;
    stuckDetector.lastCheckTime = currentTime;
  }
}

int generateLocationSignature() {
  disableMotors();
  servo.write(0);
  delay(200);
  int d0 = readUltrasonic(0);
  servo.write(90);
  delay(200);
  int d90 = readUltrasonic(90);
  servo.write(180);
  delay(200);
  int d180 = readUltrasonic(180);
  servo.write(45);
  delay(200);
  int d45 = readUltrasonic(45);
  int sig = ((d0/10) * 1000) + ((d90/10) * 100) + ((d180/10) * 10) + (d45/10);
  return sig;
}

bool isRepeatingLocation() {
  int currentSig = generateLocationSignature();
  if (currentSig == spatialMemory.lastSignature) {
    spatialMemory.repeatedSignatures++;
    if (spatialMemory.repeatedSignatures >= spatialMemory.SIGNATURE_REPEAT_THRESHOLD) {
      return true;
    }
  } else {
    spatialMemory.repeatedSignatures = 0;
  }
  spatialMemory.lastSignature = currentSig;
  for (int i = 0; i < spatialMemory.count; i++) {
    if (spatialMemory.locations[i].signature == currentSig) {
      spatialMemory.locations[i].visitCount++;
      spatialMemory.locations[i].lastVisit = millis();
      if (spatialMemory.locations[i].visitCount >= spatialMemory.MAX_VISITS) {
        return true;
      }
    }
  }
  return false;
}

void recordLocation() {
  static unsigned long lastRecord = 0;
  if (millis() - lastRecord < 5000) return;
  lastRecord = millis();
  int sig = spatialMemory.lastSignature;
  bool found = false;
  for (int i = 0; i < spatialMemory.count; i++) {
    if (spatialMemory.locations[i].signature == sig) {
      found = true;
      break;
    }
  }
  if (!found && spatialMemory.count < MAX_LOCATIONS) {
    spatialMemory.locations[spatialMemory.count].signature = sig;
    spatialMemory.locations[spatialMemory.count].lastVisit = millis();
    spatialMemory.locations[spatialMemory.count].visitCount = 1;
    spatialMemory.count++;
  }
}

void forgetOldLocations() {
  unsigned long now = millis();
  for (int i = 0; i < spatialMemory.count; i++) {
    if (now - spatialMemory.locations[i].lastVisit > spatialMemory.MEMORY_TIMEOUT) {
      for (int j = i; j < spatialMemory.count - 1; j++) {
        spatialMemory.locations[j] = spatialMemory.locations[j + 1];
      }
      spatialMemory.count--;
      i--;
    }
  }
}

void forceRandomExploration() {
  Serial.println("🎲 EXPLORAÇÃO FORÇADA");
  enableMotors();
  setMotorLeft(-255);
  setMotorRight(-255);
  delay(1000);
  stopAll();
  int bigTurn = random(120, 180);
  if (random(0, 2) == 0) {
    turnLeftAngle(bigTurn, 255);
  } else {
    turnRightAngle(bigTurn, 255);
  }
  walkDistance(random(20, 40), 255);
  stopAll();
  disableMotors();
  spatialMemory.repeatedSignatures = 0;
  spatialMemory.lastSignature = 0;
  int newState = random(0, 3);
  currentState = (State)newState;
}

// ==================================================
// MODOS
// ==================================================
void spiralMode() {
  Serial.println("===== SPIRAL MODE =====");
  disableMotors();
  servo.write(90);
  delay(200);
  int distFront = readUltrasonic(90);
  Serial.print("DistFront = ");
  Serial.println(distFront);
  if (distFront > 0 && distFront < DIST_OBSTACULO) {
    enableMotors();
    setMotorLeft(-200);
    setMotorRight(-200);
    delay(300);
    stopAll();
    disableMotors();
    servo.write(90 + ANGLE_WALL);
    delay(300);
    int distLeft = readUltrasonic(90 + ANGLE_WALL);
    servo.write(90 - ANGLE_WALL);
    delay(300);
    int distRight = readUltrasonic(90 - ANGLE_WALL);
    followingLeftWall = (distLeft < distRight);
    currentState = WALL_FOLLOW;
    wallLostTime = millis();
    return;
  }
  enableMotors();
  setMotorLeft(255);
  setMotorRight(200 - (spiralRadius * 3) < 150 ? 150 : 200 - (spiralRadius * 3));
  delay(400);
  stopAll();
  disableMotors();
  spiralRadius += spiralIncrement;
  if (spiralRadius > 30) spiralRadius = 5;
}

void wallFollowMode() {
  Serial.println("===== WALL FOLLOW MODE =====");
  disableMotors();
  servo.write(90);
  delay(200);
  int distFront = readUltrasonic(90);
  Serial.print("DistFront = ");
  Serial.println(distFront);
  if (distFront > 0 && distFront < DIST_OBSTACULO) {
    enableMotors();
    setMotorLeft(-200);
    setMotorRight(-200);
    delay(400);
    stopAll();
    if (followingLeftWall) {
      turnRightAngle(random(60, 100), 255);
    } else {
      turnLeftAngle(random(60, 100), 255);
    }
    disableMotors();
    return;
  }
  int sensorAngle = followingLeftWall ? (90 + ANGLE_WALL) : (90 - ANGLE_WALL);
  servo.write(sensorAngle);
  delay(250);
  int distWall = readUltrasonic(sensorAngle);
  if (distWall == 0 || distWall > DIST_WALL_LOST) {
    if (wallLostTime == 0) {
      wallLostTime = millis();
    } else if (millis() - wallLostTime > WALL_LOST_TIMEOUT) {
      currentState = RANDOM_WALK;
      return;
    }
  } else {
    wallLostTime = 0;
  }
  enableMotors();
  int erro = distWall - DIST_WALL_TARGET;
  int speedBase = 220;
  int correcao = erro * 3;
  if (distWall > 0 && distWall < DIST_WALL_MIN) {
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

void randomWalkMode() {
  Serial.println("===== RANDOM WALK =====");
  disableMotors();
  servo.write(90);
  delay(200);
  int distFront = readUltrasonic(90);
  Serial.print("DistFront = ");
  Serial.println(distFront);
  if (distFront > 0 && distFront < DIST_OBSTACULO) {
    enableMotors();
    walkDistance(random(8, 15), -200);
    int turnAngle = random(60, 150);
    if (random(0, 2) == 0) {
      turnLeftAngle(turnAngle, 255);
    } else {
      turnRightAngle(turnAngle, 255);
    }
    disableMotors();
    return;
  }
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
  if (random(0, 20) == 0) {
    currentState = ESCAPE;
  }
}

void escapeMode() {
  Serial.println("===== ESCAPE MODE =====");
  enableMotors();
  setMotorLeft(-220);
  setMotorRight(-220);
  delay(800);
  stopAll();
  disableMotors();
  int bestAngle = 90;
  int maxDist = 0;
  for (int angle = 0; angle <= 180; angle += 30) {
    servo.write(angle);
    delay(250);
    int dist = readUltrasonic(angle);
    Serial.print("Dist = ");
    Serial.println(dist);
    if (dist > maxDist) {
      maxDist = dist;
      bestAngle = angle;
    }
  }
  enableMotors();
  int turnNeeded = bestAngle - 90;
  if (turnNeeded > 0) {
    turnLeftAngle(abs(turnNeeded), 255);
  } else if (turnNeeded < 0) {
    turnRightAngle(abs(turnNeeded), 255);
  }
  disableMotors();
  currentState = RANDOM_WALK;
}

// ==================================================
// CONTROLE DO VENTILADOR
// ==================================================

void fanOn(int speed) { // speed de 0 a 255
  digitalWrite(FAN_IN2, LOW);  // Mantém LOW fixo
  ledcWrite(FAN_IN1, speed);   // PWM no IN1
  fanEnabled = true;
}

void fanOff() {
  ledcWrite(FAN_IN1, 0);
  digitalWrite(FAN_IN2, LOW);
  fanEnabled = false;
}

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
  pinMode(IN1_A, OUTPUT); 
  pinMode(IN2_A, OUTPUT);
  pinMode(IN1_B, OUTPUT); 
  pinMode(IN2_B, OUTPUT);
  pinMode(IN1_C, OUTPUT); 
  pinMode(IN2_C, OUTPUT);
  pinMode(IN1_D, OUTPUT); 
  pinMode(IN2_D, OUTPUT);
  digitalWrite(IN1_A, LOW); 
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, LOW); 
  digitalWrite(IN2_B, LOW);
  digitalWrite(IN1_C, LOW); 
  digitalWrite(IN2_C, LOW);
  digitalWrite(IN1_D, LOW); 
  digitalWrite(IN2_D, LOW);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_IN1, OUTPUT);
  pinMode(FAN_IN2, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  ledcAttach(FAN_IN1, FAN_FREQ, FAN_RES);
  digitalWrite(FAN_IN2, LOW);
  ledcWrite(FAN_IN1, 0); 

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightISR, RISING);
  Serial.println("\n=== SISTEMA ATIVO ===");
  stuckDetector.lastCheckTime = millis();
  delay(2000);
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  if (!fanEnabled) {
    fanOn(255);
  }
  
  if (motorsEnabled) {
    checkIfStuck();
  }
  if (isRepeatingLocation()) {
    forceRandomExploration();
  }
  recordLocation();
  forgetOldLocations();
  switch(currentState) {
    case SPIRAL: spiralMode(); break;
    case WALL_FOLLOW: wallFollowMode(); break;
    case RANDOM_WALK: randomWalkMode(); break;
    case ESCAPE: escapeMode(); break;
  }
  delay(50);
}
