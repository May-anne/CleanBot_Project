#include <Ultrasonic.h>
#include <Servo.h>

#define trigger 4
#define echo 5

#define IN1_A 9
#define IN2_A 10
#define EN_A 11

#define IN1_B 3
#define IN2_B 12
#define ultrasonicInterval 1000

Ultrasonic ultrassom(trigger, echo);
Servo servo;

int pos;
int pwmA = 0;
String flag = "";
//bool init = false;
int init = 1;
String previousDirection = "Left";
int velMotorA = 0;
long distR, distL, distF;
unsigned long currentMillis = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando a serial...");

  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(EN_A, OUTPUT);
  pinMode(IN1_A , OUTPUT);
  pinMode(IN2_A, OUTPUT);

  servo.attach(6);
  servo.write(0);

  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);

  analogWrite(EN_A, velMotorA);

}

void loop() {
  float* direction =  moveScanner();
  
  float maxDirection1 = max(direction[0], direction[1]);
  float maxDirection = max(maxDirection1, direction[2]);


  if(init == 1 || (previousDirection == "Left" && direction[0] < DistMin) || (previousDirection == "Right" && direction[2] < DistMin)){ 
    if(maxDirection == maxDirection1){
      flag = "Forward";
    }
    else if(direction[2] == maxDirection){
      flag = "Left";
    } else if(direction[0] == maxDirection1){
      flag = "Right";
    }
    init = 0;
  } else {
    flag = previousDirection;
  }

  switch(flag){
    unsigned long currentTime = 0;
    case "Right": //Right
      if(currentTime - millis() > 2000){
        turnRight();
        currentTime = millis();
      }
      previousDirection = "Right";
      break;
    case "Left": //Left
      turnLeft();
      previousDirection = "Left";
      break;
  }

  stop();
  delay(100);
  unsigned long currentTimeFront = 0;
  if(currentTimeFront - millis() > 2000){
    forward();
    currentTimeFront = millis();
  }

  stop();
  delay(100);

  //scan
}

long readUltrasonic(){
  long cmMsec;
  cmMsec = ultrassom.read(CM);

  Serial.print("Distancia em cm: ");
  Serial.println(cmMsec);

  return cmMsec;
}

float* moveScanner(){
  //o scanner deve medir a distância à esquerda, direita e frente do robô para tomar a decisão de mover (range de 180 graus)
  //move 90º para a esquerda
  //o centro possui 0 graus
  //move 90º para a direita

  Serial.println("0º");
  distR = readUltrasonic();

  for (pos = 0; pos < 90; pos++) {
    servo.write(pos);
    delay(15); 
  }

  Serial.println("90º");
  distF = readUltrasonic();
  delay(2000);

  for (pos = 90; pos < 180; pos++) {
    servo.write(pos);
    delay(15); 
  }

  Serial.println("180º");
  distL = readUltrasonic();

  for (pos = 180; pos >= 0; pos--) {
    servo.write(pos);
    delay(15);
  }

  delay(2000);

  float valuesCM[] = {distR, distF, distL};
  return valuesCM;

}

void turnLeft(){

}

void turnRight(){

}

void forward(){

}

void backward(){

}

void moveMotor(){
  analogWrite(EN_A, 255); //speed 0 - 255
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN1_B, HIGH);
}

void stop(){
  analogWrite(EN_A, 0);
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN1_B, LOW);
}


