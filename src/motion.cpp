#include <var.h>
#include <Arduino.h>

void motion(int _data) {

  if(_data == 0) { 
    rpmAlter = false;
    rpmAlter_T = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
  }  
  
  else if (_data == 1) {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, LOW);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 215: 250);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 205: 240);
  } 
  
  else if(_data == 2) {
    rpmAlter_T = false;
    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 202 : 240);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 211 : 249); 
  } 
  
  else if (_data >= 11 && _data <= 20) {
    rpmAlter = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 82 : 148);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80: 146);
  } 
  
  else if (_data >= 21 && _data <= 30) {
    rpmAlter = false;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 83 :150);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80 :147);
  }  
  
  else if(_data >= 111 && _data <= 120) {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 204:245);
    analogWrite(pwmPin_R, 150);  
  } 
  
  else if(_data >= 121 && _data <= 130) {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 215 :250);
  } 
  
  else if(_data >= 211 && _data <= 220) {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 209 : 251);
    analogWrite(pwmPin_R, 150); 
  } 
  
  else if(_data >= 221 && _data <= 230) {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 202 :245);
  } 
  
  else {} 
}

