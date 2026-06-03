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

    int pwmL = 230 - correction;
    int pwmR = 230 + correction;


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
    
    int pwmL = 230 + correction;
    int pwmR = 230 - correction;


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

    int pwm;

    if(_data <14) pwm = 80;
    else if(_data < 17) pwm = 130;
    else pwm = 180; 

    analogWrite(pwmPin_L, pwm);
    analogWrite(pwmPin_R, pwm);
  } 
  
  else if (_data >= 21 && _data <= 30) 
  {
    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);

    int pwm;
    if(_data <24) pwm = 80;
    else if(_data < 27) pwm = 130;
    else pwm = 180;

    analogWrite(pwmPin_L, pwm);
    analogWrite(pwmPin_R, pwm);
  }  
  
  else if(_data >= 111 && _data <= 120) {
  
    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    int outerpwm;
    int innerpwm;

    if(_data <114) 
    {
      outerpwm = 180;
      innerpwm = 140;
    } 
    else if(_data <117) 
    {
      outerpwm = 210;
      innerpwm = 130;
    } 
    else 
    {
      outerpwm = 240;
      innerpwm = 120;
    }

    analogWrite(pwmPin_L, outerpwm);
    analogWrite(pwmPin_R, innerpwm);  
  } 
  
  else if(_data >= 121 && _data <= 130) {

    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    int outerpwm;
    int innerpwm;

    if(_data <124) 
    {
      outerpwm = 180;
      innerpwm = 140;
    } 
    else if(_data <127) 
    {
      outerpwm = 210;
      innerpwm = 130;
    } 
    else 
    {
      outerpwm = 240;
      innerpwm = 120;
    }

    analogWrite(pwmPin_L, innerpwm);
    analogWrite(pwmPin_R, outerpwm);
  } 
  
  else if(_data >= 211 && _data <= 220) {
    
    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;
    
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    
    int outerpwm;
    int innerpwm;

    if(_data <214) 
    {
      outerpwm = 180;
      innerpwm = 140;
    } 
    else if(_data <217) 
    {
      outerpwm = 210;
      innerpwm = 130;
    } 
    else 
    {
      outerpwm = 240;
      innerpwm = 120;
    }

    analogWrite(pwmPin_L, outerpwm);
    analogWrite(pwmPin_R, innerpwm); 
  } 
  
  else if(_data >= 221 && _data <= 230) {
    
    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    int outerpwm;
    int innerpwm;

    if(_data <224) 
    {
      outerpwm = 180;
      innerpwm = 140;
    } 
    else if(_data <227) 
    {
      outerpwm = 210;
      innerpwm = 130;
    } 
    else 
    {
      outerpwm = 240;
      innerpwm = 120;
    }

    analogWrite(pwmPin_L, innerpwm);
    analogWrite(pwmPin_R, outerpwm);
  } 
  
  else {} 
}

