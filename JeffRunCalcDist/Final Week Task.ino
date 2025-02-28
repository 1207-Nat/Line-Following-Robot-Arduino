#include <LiquidCrystal.h>
#include<Wire.h>

unsigned long startTime;

int rightSensorValue;
int leftSensorValue; 

int b = 1;
int once = 1;
int end = 1;

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

/*
// Ultrasonic sensor
const int trigPin = 2;
const int echoPin = 10;
long duration;
int distance;
*/

// Motor control pins
int RIGHT_SENSOR = 12;
int LEFT_SENSOR = 13;

int ENA = 11;
int IN1 = A0; // RIGHT BACKWARD
int IN2 = A1; // RIGHT FORWARD

int ENB = 3;
int IN3 = A2; // LEFT BACKWARD
int IN4 = A3; // LEFT FORWARD

// Encoder sensor pins
//int encoderA = 10;
//int stateA;
//int lastStateA;
//volatile unsigned long encoderCountA = 0;

int encoderB = 2;
int stateB;
int lastStateB;
volatile unsigned long encoderCountB1 = 0;
volatile unsigned long encoderCountB2 = 0;
volatile unsigned long encoderCountB3 = 0;
volatile unsigned long encoderCountB4 = 0;
volatile unsigned long encoderCountB5 = 0;

float circumference = 21.2; //cm
float a=0.0;
float* distanceForward = &a;
float* distance60 = &a;
float distanceMoveUpRamp = 0.0;
float distanceAccelerate = 0.0;
float distanceAfterRamp = 0.0;
float totalDistance = 0.0;

void setup()
{
  //MPU
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  //Accelerate at the beginning
  startTime = millis();

  /*
  //Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  */

  //Motor
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Encoder
  //pinMode(encoderA, INPUT);
  //pinMode(encoderA, INPUT_PULLUP);
  //lastStateA=digitalRead(encoderA);

  pinMode(encoderB, INPUT);
  pinMode(encoderB, INPUT_PULLUP);
  lastStateB=digitalRead(encoderB);

  //LCD
  lcd.begin(16, 2); // Initialize a 16x2 LCD
}

void moveForward(int once, float* distanceForward, float* distance60) 
{ 
  analogWrite(ENA, 65);
  analogWrite(ENB, 65);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if (once==0) //Already pass the ramp
  {
    //To calculate the distance after the ramp
    stateB=digitalRead(encoderB);
    if (stateB!=lastStateB)
    {
      encoderCountB1++;
    }
    stateB=lastStateB;
    *distance60=circumference*(float)(encoderCountB1)/30.0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance1: ");
    lcd.print(*distance60);

    if (((*distance60)>59.9)&&((*distance60)<60.1))
    {
      unsigned long startTask2 = millis();
      while (millis() - startTask2 <= 5000) 
      {
        stopMoving();
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("Distance2:");
        lcd.print(*distance60);
      }

      unsigned long Start = millis();
      while (millis()-Start<=500)
      {
        analogWrite(ENA, 100);
        analogWrite(ENB, 100);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
    }
  }
  
  else  //Calculate the distance before the ramp
  {
    stateB=digitalRead(encoderB);
    if (stateB!=lastStateB)
    {
      encoderCountB2++;
    }
    stateB=lastStateB;
    *distanceForward=circumference*(float)(encoderCountB2)/30.0;  
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance3:");
    lcd.print(*distanceForward); 
  }
}

float moveUpRamp()
{
  unsigned long hillStart = millis();
  while (millis() - hillStart <= 1200) 
  {
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    double angleX=readMPU();
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Ramp angle: ");
    lcd.print(angleX);

    stateB=digitalRead(encoderB);
    if (stateB!=lastStateB)
    {
      encoderCountB3++;
    }
    stateB=lastStateB;
    distanceMoveUpRamp=circumference*(float)(encoderCountB3)/30.0;
  }
  return (distanceMoveUpRamp);
}

void turnRight() 
{
  analogWrite(ENA, 100);
  analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() 
{
  analogWrite(ENA, 255);
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
  while (millis()-rotateStart<=3100)
  {
    analogWrite(ENA, 200);
    analogWrite(ENB, 50);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  //To stabilize the car
  unsigned long rotateStop = millis();
  while (millis() - rotateStop <= 500) 
  {
    stopMoving();
  }
}

double readMPU()
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
  lcd.setCursor(0, 1);
  lcd.print("Angle x: ");
  lcd.print(x);

  return (x);
}

/*
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
*/

void stop4s()
{
  unsigned long stopStart = millis();
  while (millis()-stopStart<=4000)
  {
    stopMoving();
  }

}

void loop()
{
  // Check if the acceleration phase is still ongoing
  if (millis()-startTime < 500)
  {
    // Accelerate the robot
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    stateB=digitalRead(encoderB);
    if (stateB!=lastStateB)
    {
      encoderCountB4++;
    }
    stateB=lastStateB;
    distanceAccelerate=circumference*(float)(encoderCountB4)/30.0;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance: ");
    lcd.print(distanceAccelerate);
  }
  
  else
  {
    while (b==1)
    {    
      /*totalDistance = distanceForward + distanceMoveUpRamp + distanceAccelerate + distanceAfterRamp;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Distance: ");
      lcd.print(totalDistance);*/
      
      rightSensorValue = digitalRead(RIGHT_SENSOR);
      leftSensorValue = digitalRead(LEFT_SENSOR); 
      
      if (rightSensorValue == LOW && leftSensorValue == LOW) 
      {
        double x1=readMPU();
        if ((x1>13.0)&&(x1<25.0)&&(once==1))
        {
          double x1=readMPU();
          distanceMoveUpRamp=moveUpRamp();
          stop4s(); 
          rotate(); 
          once=0;
        }
        
        else
        {
          moveForward(once, distanceForward, distance60);
        }           
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
        if (end==1) //reaches the end of the top of the ramp
        {
          unsigned long start=millis();
          while (millis()-start<500)
          {
            analogWrite(ENA, 100);
            analogWrite(ENB, 100);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);

            stateB=digitalRead(encoderB);
            if (stateB!=lastStateB)
            {
              encoderCountB5++;
            }
            stateB=lastStateB;
            distanceAfterRamp=circumference*(float)(encoderCountB5)/30.0;
            end=0;
          }
          moveForward(once, distanceForward, distance60); //slowing down when going down the ramp
        }

        else
        {
          stopMoving();
          b=0;
        }
      }
    }
  }

  while(b==0)
  {
    totalDistance = *distanceForward +  *distance60;
    // + distanceMoveUpRamp + distanceAccelerate + distanceAfterRamp;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance: ");
    lcd.print(totalDistance);
    lcd.print("cm");   
  } 
}
