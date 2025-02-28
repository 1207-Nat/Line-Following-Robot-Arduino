#include <LiquidCrystal.h>
#include<Wire.h>

int rightSensorValue;
int leftSensorValue; 

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, 4, 5, 6, 7); // Initialize the LCD with your pin configuration

// MPU-6050
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;

// Ultrasonic sensor
const int trigPin = 2;
const int echoPin = 10;
long duration;
int distance;

// Motor control pins
int RIGHT_SENSOR = 12;
int LEFT_SENSOR = 13;

int ENA = 11;
int IN1 = A0; // RIGHT BACKWARD
int IN2 = A1; // RIGHT FORWARD

int ENB = 3;
int IN3 = A2; // LEFT BACKWARD
int IN4 = A3; // LEFT FORWARD
/*
// Encoder sensor pins
int encoderA = 10;
int encoderB = 2;
int stateA;
int lastStateA;
int stateB;
int lastStateB;
volatile unsigned long encoderCountA = 0;
volatile unsigned long encoderCountB = 0;
float circumference = 21.2; //cm
//Distance measurement
float distanceForward = 0;*/

// Accelerate at the beginning
unsigned long startTime;
unsigned long accelerationDuration = 500; // 1000 milliseconds = 1 second

void setup()
{
  /*//MPU
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);*/
  Serial.begin(9600);

  startTime = millis();

  //Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  //Motor
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  /*//Encoder
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  lastStateA=digitalRead(encoderA);
  lastStateB=digitalRead(encoderB);*/

  lcd.begin(16, 2); // Initialize a 16x2 LCD
}

void moveForward() 
{ 
  analogWrite(ENA, 65);
  analogWrite(ENB, 65);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void speedUp() 
{ 
  analogWrite(ENA, 170);
  analogWrite(ENB, 150);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void slowDown() 
{ 
  analogWrite(ENA, 60);
  analogWrite(ENB, 60);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveUpRamp(){
  unsigned long hillStart = millis();
  while (millis() - hillStart <= 3500) {
    speedUp();
  }
}

void turnRight() 
{
  analogWrite(ENA, 100);
  analogWrite(ENB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() 
{
  analogWrite(ENA, 200);
  analogWrite(ENB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMoving() 
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void rotate()
{
  unsigned long rotateStart = millis();
  unsigned long rotateCurrent;
  unsigned long rotateTime = 0;
  
  while (rotateTime<4000)
  {
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    rotateCurrent = millis();
    rotateTime = rotateCurrent - rotateStart;

    /*if ((rightSensorValue == LOW) && (leftSensorValue == HIGH))
    {
      count++;
    }

    else if ((rightSensorValue == HIGH) && (leftSensorValue == LOW))
    {
      count++;
    }*/
  }
  delay(2000);
}

/*double readMPU()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90); 
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);  
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  Serial.print("AngleX= ");
  Serial.println(x);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Angle x: ");
  lcd.setCursor(0, 1);
  lcd.print(x);
  return (x);
}*/

double readUltrasonic()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.0343 / 2;

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HRSr04: ");
  lcd.print(distance);

  return(distance);
}
void stop4s()
{
  unsigned long stopStart = millis();
  while (millis() - stopStart <= 4000) 
  {
    stopMoving();
  }

}
void loop()
{
  int b=1;
  unsigned long currentTime = millis();
  unsigned long accelerationElapsedTime = currentTime - startTime;

  //  Check if the acceleration phase is still ongoing
  if (accelerationElapsedTime < accelerationDuration)
  {
    // Accelerate the robot
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  else
  {
    while (b==1)
    {
      rightSensorValue = digitalRead(RIGHT_SENSOR);
      leftSensorValue = digitalRead(LEFT_SENSOR);   
      
      //double x1=readMPU();

      
      
      if (rightSensorValue == LOW && leftSensorValue == LOW) 
      {
        distance=readUltrasonic();
        if ((distance>=30.0) && (distance<=40.0))
        {
          moveUpRamp();
          stop4s(); //stop for 4s
          rotate();
        }

        else if ((distance>=450.0) && (distance<=550.0))
        {
          unsigned long hillStart = millis();
          unsigned long hillCurrent = 0;
          unsigned long hillElapsedTime = 0;
          while (hillElapsedTime<=2000)
          {
            slowDown();
            hillCurrent = millis();
            hillElapsedTime = hillCurrent - hillStart;
          }
        }
        
        else
        {
          moveForward();
        }
        //double x2=readMPU();

        /*if ((x1-x2)>30) //reaches the top of ramp
        {
          slowDown();
          delay(4000); //stop for 4s
          rotate();
        }*/

        /*else if (x1>x2) //reaches the end of the top of the ramp
        {
          slowDown(); //slowing down when going down the ramp
        }*/        
      } 
      
      else if (rightSensorValue == LOW && leftSensorValue == HIGH) 
      {
        turnLeft();
      } 
     
      else if (rightSensorValue == HIGH && leftSensorValue == LOW)           
      {
        turnRight();
      } 
      
      else 
      {
        stopMoving();
        b=0;
      }
    }
  }

  /*while(b==0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.setCursor(0, 1);
    lcd.print("cm");   
  } */
}