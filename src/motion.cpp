#include <var.h>
#include <Arduino.h>


//HEADING PID
float targetHeading = 0;

float kpHeading = 1.5;
float kiHeading = 0.01;
float kdHeading = 0.2;

float headingIntegral = 0;
float previousError = 0;
int lastDriveCommand = 0;

void motion(int _data) {

  if(_data == 0) { 

    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
    
  }  
  
  else if (_data == 1) 
  {
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    if(lastDriveCommand != _data)
    {
      targetHeading = heading;
      headingIntegral = 0;
      previousError = 0;
    }

    float error = targetHeading - heading;

    if(error > 180) error -= 360;
    if(error < -180) error += 360;

    headingIntegral += error;
    headingIntegral = constrain(headingIntegral, -100, 100);

    float derivative = error - previousError;

    float correction =
          kpHeading * error
        + kiHeading * headingIntegral
        + kdHeading * derivative;

    previousError = error;

    int pwmL = (rpmAlter == 0 ? 205 : 240) - correction;
    int pwmR = (rpmAlter == 0 ? 215 : 250) + correction;


    pwmL = constrain(pwmL, 0, 255);
    pwmR = constrain(pwmR, 0, 255);

    analogWrite(pwmPin_L, pwmL);
    analogWrite(pwmPin_R, pwmR);

    lastDriveCommand = _data;
  } 
  
  else if(_data == 2) 
  {
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    if(lastDriveCommand != _data)
    {
      targetHeading = heading;
      headingIntegral = 0;
      previousError = 0;
    }

    float error = targetHeading - heading;

    if(error > 180) error -= 360;
    if(error < -180) error += 360;

    headingIntegral += error;
    headingIntegral = constrain(headingIntegral, -100, 100);

    float derivative = error - previousError;

    float correction =
          kpHeading * error
        + kiHeading * headingIntegral
        + kdHeading * derivative;

    previousError = error;

    int pwmL = (rpmAlter == 0 ? 205 : 240) + correction;
    int pwmR = (rpmAlter == 0 ? 215 : 250) - correction;
   
    pwmL = constrain(pwmL, 0, 255);
    pwmR = constrain(pwmR, 0, 255);

    analogWrite(pwmPin_L, pwmL);
    analogWrite(pwmPin_R, pwmR);


    lastDriveCommand = _data;

  } 
  
  else if (_data >= 11 && _data <= 20) {

    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 82 : 148);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80: 146);
  } 
  
  else if (_data >= 21 && _data <= 30) 
  {
    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 83 :150);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80 :147);
  }  
  
  else if(_data >= 111 && _data <= 120) {
  
    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 204:245);
    analogWrite(pwmPin_R, 150);  
  } 
  
  else if(_data >= 121 && _data <= 130) {

    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 215 :250);
  } 
  
  else if(_data >= 211 && _data <= 220) {
    
    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;
    
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 209 : 251);
    analogWrite(pwmPin_R, 150); 
  } 
  
  else if(_data >= 221 && _data <= 230) {
    
    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 202 :245);
  } 
  
  else {} 
}

