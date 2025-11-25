// ==================================================
// DEFINIÇÃO DOS MOTORES (com flags de inversão)
// ==================================================

// ---- DIREITA TRASEIRA (A)
#define IN1_A 27
#define IN2_A 14
#define EN_A  13
bool INV_A = true;

// ---- ESQUERDA TRASERA (B)
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

void forward(int s = 255) {
  Serial.println("Frente");
  driveMotor(IN1_A, IN2_A, EN_A,  s, INV_A);
  driveMotor(IN1_B, IN2_B, EN_B,  s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C,  s, INV_C);
  driveMotor(IN1_D, IN2_D, EN_D,  s, INV_D);
}

void backward(int s = 255) {
  driveMotor(IN1_A, IN2_A, EN_A, -s, INV_A);
  driveMotor(IN1_B, IN2_B, EN_B, -s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C, -s, INV_C);
  driveMotor(IN1_D, IN2_D, EN_D, -s, INV_D);
}

void turnLeft(int s = 255) {
  Serial.println("Esquerda");
  // direita pra frente, esquerda pra trás
  driveMotor(IN1_A, IN2_A, EN_A,  s, INV_A);
  driveMotor(IN1_D, IN2_D, EN_D,  s, INV_D);
  driveMotor(IN1_B, IN2_B, EN_B, -s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C, -s, INV_C);
}

void turnRight(int s = 255) {
  Serial.println("Direita");
  // esquerda pra frente, direita pra trás
  driveMotor(IN1_A, IN2_A, EN_A, -s, INV_A);
  driveMotor(IN1_D, IN2_D, EN_D, -s, INV_D);
  driveMotor(IN1_B, IN2_B, EN_B,  s, INV_B);
  driveMotor(IN1_C, IN2_C, EN_C,  s, INV_C);
}


void setup() {
  Serial.begin(115200);

  pinMode(IN1_A, OUTPUT); pinMode(IN2_A, OUTPUT); pinMode(EN_A, OUTPUT);
  pinMode(IN1_B, OUTPUT); pinMode(IN2_B, OUTPUT); pinMode(EN_B, OUTPUT);
  pinMode(IN1_C, OUTPUT); pinMode(IN2_C, OUTPUT); pinMode(EN_C, OUTPUT);
  pinMode(IN1_D, OUTPUT); pinMode(IN2_D, OUTPUT); pinMode(EN_D, OUTPUT);

  stopAll();
  Serial.println("Iniciando...");
}

void loop() {
  forward();
  delay(1500);

  stopAll();
  delay(1500);

  turnRight();
  delay(1500);

  stopAll();
  delay(1500);

  turnLeft();
  delay(1500);

  stopAll();
  delay(1500);
}
