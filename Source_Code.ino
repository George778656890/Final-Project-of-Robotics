//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;  //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;  //control pin 2 on the motor driver for the right motor
const int PWMA = 11;  //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;  //speed control pin on the motor driver for the left motor
const int BIN2 = 9;   //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;   //control pin 1 on the motor driver for the left motor

int switchPin = 7;  //switch to turn the robot on and off

int photoresistor = 0;  //this variable will hold a value based on the brightness of the ambient light
int BrightnessThreshold = 970;

int BaseSpeed = 250;
int LeftSpeed = BaseSpeed;
int RightSpeed = BaseSpeed +3 ;
double Proportionality_Constant=1.025;  //Used for Proportional Control(P) to keep the robot in the middle of the road


double RightDistance = 0.0;  //variable to store the distance measured by the Front distance sensor
double LeftDistance = 0.0;   //variable to store the distance measured by the Front distance sensor


//The ultrasound sensor variables
const int FrontTrigPin = 2;   
const int FrontEchoPin = 1;   //The serial communication must be disabled to activate this pin !!!

const int RightTrigPin = 4;
const int RightEchoPin = 3;

const int LeftTrigPin = 6;
const int LeftEchoPin = 5;




void setup() {
  //Serial.begin(9600);                //start a serial connection with the computer
  pinMode(switchPin, INPUT_PULLUP);  //set this as a pullup to sense whether the switch is flipped

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  //set the ultrasound sensor's input and output
  pinMode(FrontTrigPin, OUTPUT);
  pinMode(FrontEchoPin, INPUT);

  pinMode(RightTrigPin, OUTPUT);
  pinMode(RightEchoPin, INPUT);

  pinMode(LeftTrigPin, OUTPUT);
  pinMode(LeftEchoPin, INPUT);
}

void loop() {
  photoresistor = analogRead(A0);  //set photoresistor to a number between 0 and 1023 based on how bright the ambient light is
  /*Serial.print("The brightness is :");
  Serial.println(photoresistor);*/


  LeftDistance = getDistance(LeftTrigPin, LeftEchoPin);
  //Serial.print("The Left distance is :");
  //Serial.println(LeftDistance);

  RightDistance = getDistance(RightTrigPin, RightEchoPin);
  //Serial.print("The Right distance is :");
  //Serial.println(RightDistance);


// Behaviour switch
 if (digitalRead(switchPin) == LOW){

  if((getDistance(LeftTrigPin, LeftEchoPin))<10.0){          //Detect obstacle on the left side,move right
      MoveRight();
  }else if((getDistance(RightTrigPin, RightEchoPin))<10.0){  //Detect obstace on the right side,move left
      MoveLeft();
  }else if(photoresistor>BrightnessThreshold){               // Detecting the high brightness,stop there
      TurnOffMotors();
  }else if((getDistance(FrontTrigPin, FrontEchoPin) - 2.32)<10){//Detect obstacle in front,turn to the side which has more space
    if(getDistance(LeftTrigPin, LeftEchoPin)<getDistance(RightTrigPin, RightEchoPin)){
      ReverseAndTurnRight();
    }else{
      ReverseAndTurnLeft();
    }
    
  }else{
    MoveForward();//By default,the robot will move forward
  }

}else{
  TurnOffMotors();
}


}

/********************************************************************************/
double getDistance(int trigPin, int echoPin) {
  double echoTime;            //variable to store the time it takes for a ping to bounce off an object
  double calculatedDistance;  //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);  //use the pulsein command to see how long it takes for the
                                      //pulse to bounce back to the sensor

  calculatedDistance = ((echoTime / 1000000) * 343 / 2) * 100;  //The unit of length is "centimeter" now,by Zhihong Liu(student id:3704783)

  return calculatedDistance;  //send back the distance that was calculated
}




void MoveForward(){

  SetForwardMode();
  analogWrite(PWMA, RightSpeed);
  analogWrite(PWMB, LeftSpeed);

}

void ReverseAndTurnLeft(){

  TurnOffMotors();

  SetReverseMode();
  analogWrite(PWMA, RightSpeed);
  analogWrite(PWMB, LeftSpeed);
  delay(1000);

  SetLeftTurnMode();
  analogWrite(PWMA, RightSpeed);
  analogWrite(PWMB, LeftSpeed);
  delay(600);

  TurnOffMotors();

}

void ReverseAndTurnRight(){

  TurnOffMotors();

  SetReverseMode();
  analogWrite(PWMA, RightSpeed);
  analogWrite(PWMB, LeftSpeed);
  delay(1000);

  SetRightTurnMode();
  analogWrite(PWMA, RightSpeed);
  analogWrite(PWMB, LeftSpeed);
  delay(600);

  TurnOffMotors();

}

void MoveRight(){

  SetForwardMode();
  analogWrite(PWMA, RightSpeed);
  analogWrite(PWMB, round(LeftSpeed*Proportionality_Constant));  //Increase the left wheel's speed to move right
  delay(500);
  TurnOffMotors();

}

void MoveLeft(){

  SetForwardMode();
  analogWrite(PWMA, round(RightSpeed*Proportionality_Constant)); //Increase the right wheel's speed to move left
  analogWrite(PWMB,LeftSpeed);
  delay(500);
  TurnOffMotors();

}

void SetForwardMode() {

  digitalWrite(AIN1, HIGH);  //set pin 1 to high
  digitalWrite(AIN2, LOW);   //set pin 2 to low
  digitalWrite(BIN1, HIGH);  //set pin 1 to high
  digitalWrite(BIN2, LOW);   //set pin 2 to low
}

void SetReverseMode() {

  digitalWrite(AIN1, LOW);   //set pin 1 to high
  digitalWrite(AIN2, HIGH);  //set pin 2 to low
  digitalWrite(BIN1, LOW);   //set pin 1 to high
  digitalWrite(BIN2, HIGH);  //set pin 2 to low
}

void SetLeftTurnMode() {
  digitalWrite(AIN1, HIGH);  //set pin 1 to high
  digitalWrite(AIN2, LOW);   //set pin 2 to low
  digitalWrite(BIN1, LOW);   //set pin 1 to high
  digitalWrite(BIN2, HIGH);  //set pin 2 to low
}

void SetRightTurnMode() {

  digitalWrite(AIN1, LOW);   //set pin 1 to low
  digitalWrite(AIN2, HIGH);  //set pin 2 to high
  digitalWrite(BIN1, HIGH);  //set pin 1 to high
  digitalWrite(BIN2, LOW);   //set pin 2 to low
}

void TurnOffMotors() {

  digitalWrite(AIN1, LOW);  //set pin 1 to low
  digitalWrite(AIN2, LOW);  //set pin 2 to low
  digitalWrite(BIN1, LOW);  //set pin 1 to low
  digitalWrite(BIN2, LOW);  //set pin 2 to low
}
