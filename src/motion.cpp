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

// RPM PID
float kpRPM = 1.0;
float kiRPM = 0.02;
float kdRPM = 0.1;

float rpmIntegral_L = 0;
float rpmIntegral_R = 0;

float previousRPMError_L = 0;
float previousRPMError_R = 0;

float pwmOutput_L = 50;
float pwmOutput_R = 50;

// S-curve ramp-up state: RPM target ramps from 0 to baseRPM
float rampedBaseRPM = 0;

// S-curve ramp-up on RPM target (mirrors rampDown's 5-zone philosophy)
// Zone 1:  0 - 14 RPM  -> step 0.5  (slow start)
// Zone 2: 14 - 25 RPM  -> step 0.5  (gradual)
// Zone 3: 25 - 50 RPM  -> step 2.0  (fast mid)
// Zone 4: 50 - 60 RPM  -> step 0.5  (decelerate)
// Zone 5: 60 - target  -> step 0.5  (settle)
float applyRPMRampUp(float current, float target)
{
    float step;
    if(current < 14)       step = 0.5;
    else if(current < 25)  step = 0.5;
    else if(current < 50)  step = 2.0;
    else if(current < 60)  step = 0.5;
    else                   step = 0.5;

    current += step;
    if(current > target) current = target;
    return current;
}

int applyRamp(int current, int target, int step)
{
    if(current > target)
    {
        current -= step;

        if(current < target)
            current = target;
    }

    return current;
}

void rampDown()
{
    if (pwmOutput_L > 180)
        pwmOutput_L = applyRamp(pwmOutput_L, 0, 8);
    else if (pwmOutput_L > 150)
        pwmOutput_L = applyRamp(pwmOutput_L, 0, 2);
    else if (pwmOutput_L > 90)
        pwmOutput_L = applyRamp(pwmOutput_L, 0, 8);
    else if (pwmOutput_L > 60)
        pwmOutput_L = applyRamp(pwmOutput_L, 0, 2);
    else
        pwmOutput_L = applyRamp(pwmOutput_L, 0, 2);

    if (pwmOutput_R > 180)
        pwmOutput_R = applyRamp(pwmOutput_R, 0, 8);
    else if (pwmOutput_R > 150)
        pwmOutput_R = applyRamp(pwmOutput_R, 0, 2);
    else if (pwmOutput_R > 90)
        pwmOutput_R = applyRamp(pwmOutput_R, 0, 8);
    else if (pwmOutput_R > 60)
        pwmOutput_R = applyRamp(pwmOutput_R, 0, 2);
    else
        pwmOutput_R = applyRamp(pwmOutput_R, 0, 2);

    analogWrite(pwmPin_L, pwmOutput_L);
    analogWrite(pwmPin_R, pwmOutput_R);
}

void motion(int _data) {

  if(_data == 0) { 

    if(pwmOutput_L > 0 || pwmOutput_R > 0)
    {
      rampDown();
      return;
    }

    lastDriveCommand = 0;
    headingIntegral = 0;
    previousError = 0;

    rpmIntegral_L = 0;
    rpmIntegral_R = 0;

    previousRPMError_L = 0;
    previousRPMError_R = 0;

    pwmOutput_L = 0;
    pwmOutput_R = 0;

    rampedBaseRPM = 0;

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


      rpmIntegral_L = 0;
      rpmIntegral_R = 0;

      previousRPMError_L = 0;
      previousRPMError_R = 0;

      pwmOutput_L = 0;
      pwmOutput_R = 0;
      rampedBaseRPM = 0;
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

    // int pwmL = (rpmAlter == 0 ? 205 : 240) - correction;
    // int pwmR = (rpmAlter == 0 ? 215 : 250) + correction;

    // pwmL = constrain(pwmL, 0, 255);
    // pwmR = constrain(pwmR, 0, 255);

    // analogWrite(pwmPin_L, pwmL);
    // analogWrite(pwmPin_R, pwmR);

    // S-curve ramp the RPM target — PID runs the whole time
    rampedBaseRPM = applyRPMRampUp(rampedBaseRPM, 70);
    float baseRPM = rampedBaseRPM;

    float targetRPM_L = baseRPM - correction;
    float targetRPM_R = baseRPM + correction;

    targetRPM_L = constrain(targetRPM_L, 0, 100);
    targetRPM_R = constrain(targetRPM_R, 0, 100);

    if(rpmUpdated)
    {
      rpmUpdated = false;

      float errorRPM_L = targetRPM_L - avgRPM_L;
      float errorRPM_R = targetRPM_R - avgRPM_R;
  
      rpmIntegral_L += errorRPM_L;
      rpmIntegral_R += errorRPM_R;
      
      rpmIntegral_L = constrain(rpmIntegral_L, -500, 500);
      rpmIntegral_R = constrain(rpmIntegral_R, -500, 500);
  
      float derivativeRPM_L = errorRPM_L - previousRPMError_L;
      float derivativeRPM_R = errorRPM_R - previousRPMError_R;
  
      float correctionRPM_L =
            kpRPM * errorRPM_L
          + kiRPM * rpmIntegral_L
          + kdRPM * derivativeRPM_L;
  
      float correctionRPM_R =
            kpRPM * errorRPM_R
          + kiRPM * rpmIntegral_R
          + kdRPM * derivativeRPM_R;
  
      previousRPMError_L = errorRPM_L;
      previousRPMError_R = errorRPM_R;
  
      pwmOutput_L += correctionRPM_L;
      pwmOutput_R += correctionRPM_R;
  
      pwmOutput_L = constrain(pwmOutput_L, 0, 255);
      pwmOutput_R = constrain(pwmOutput_R, 0, 255);
      analogWrite(pwmPin_L, pwmOutput_L);
      analogWrite(pwmPin_R, pwmOutput_R);
    }
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


      rpmIntegral_L = 0;
      rpmIntegral_R = 0;

      previousRPMError_L = 0;
      previousRPMError_R = 0;

      pwmOutput_L = 0;
      pwmOutput_R = 0;
      rampedBaseRPM = 0;
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

    // S-curve ramp the RPM target — PID runs the whole time
    rampedBaseRPM = applyRPMRampUp(rampedBaseRPM, 70);
    float baseRPM = rampedBaseRPM;

    float targetRPM_L = baseRPM + correction;
    float targetRPM_R = baseRPM - correction;

    targetRPM_L = constrain(targetRPM_L, 0, 100);
    targetRPM_R = constrain(targetRPM_R, 0, 100);

    if(rpmUpdated)
    {
      rpmUpdated = false;

      float errorRPM_L = targetRPM_L - avgRPM_L;
      float errorRPM_R = targetRPM_R - avgRPM_R;
    
      rpmIntegral_L += errorRPM_L;
      rpmIntegral_R += errorRPM_R;
    
      rpmIntegral_L = constrain(rpmIntegral_L, -500, 500);
      rpmIntegral_R = constrain(rpmIntegral_R, -500, 500);
    
      float derivativeRPM_L = errorRPM_L - previousRPMError_L;
      float derivativeRPM_R = errorRPM_R - previousRPMError_R;
    
      float correctionRPM_L =
            kpRPM * errorRPM_L
          + kiRPM * rpmIntegral_L
          + kdRPM * derivativeRPM_L;
    
      float correctionRPM_R =
            kpRPM * errorRPM_R
          + kiRPM * rpmIntegral_R
          + kdRPM * derivativeRPM_R;
    
      previousRPMError_L = errorRPM_L;
      previousRPMError_R = errorRPM_R;
    
      pwmOutput_L += correctionRPM_L;
      pwmOutput_R += correctionRPM_R;
    
      pwmOutput_L = constrain(pwmOutput_L, 0, 255);
      pwmOutput_R = constrain(pwmOutput_R, 0, 255);
    
      analogWrite(pwmPin_L, pwmOutput_L);
      analogWrite(pwmPin_R, pwmOutput_R);
    }
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

    pwmOutput_L = pwm;
    pwmOutput_R = pwm;
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

    pwmOutput_L = pwm;
    pwmOutput_R = pwm;
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

    pwmOutput_L = outerpwm;
    pwmOutput_R = innerpwm;
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

    pwmOutput_L = innerpwm;
    pwmOutput_R = outerpwm;
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

    pwmOutput_L = outerpwm;
    pwmOutput_R = innerpwm;
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

    pwmOutput_L = innerpwm;
    pwmOutput_R = outerpwm;
  } 
  
  else {} 
}

