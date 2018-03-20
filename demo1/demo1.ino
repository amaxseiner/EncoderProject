#define irSensor 0
#define motorLeft A6
#define motorRight A1
#define potPin A0
#define motorLeft_Dir_cntl_1 23
#define motorLeft_Dir_cntl_2 21
#define motorRight_Dir_cntl_1 16
#define motorRight_Dir_cntl_2 15
#define pot_max_value 1023
#define pot_min_value 0
#define pot_mid 512
#define kp 1.0
#define ki 0.01
#define kd 0.2
#define max_rads 22
#define min_rads 0
//errors
#define e_ok 0
#define e_dir -1
#define e_mot -2
#define e_pot -3
const int led = LED_BUILTIN;
const float mot_max_rads = 20.8;
const float mot_min_rads = 0.0;
const int mot_max_val = 255;
const int mot_min_val = 0;
const int clockwise = 1;
const int counterClockWise = 0;
const float piii = .523;
int rr= -1;
int j = 0;
int dj = 10;
float T = .001;
int r = 0;
float dt = 0;
int counter =0;
float e = 0.0;
//float ei = 0;
float lastE = 0.0;

float theta = 0.0;
float thetaDis = 0.0;
float vel = 0.0;

void setup() {
  pinMode(irSensor, INPUT);
  pinMode(motorLeft_Dir_cntl_1,OUTPUT);
  pinMode(motorLeft_Dir_cntl_2,OUTPUT);
  pinMode(motorLeft,OUTPUT);
  // Only enabling the left motor
  // this desing only has 1 motor
  pinMode(led,OUTPUT);
  pinMode(potPin,INPUT);
  //digitalWrite(pwm_pin, HIGH);
  //digitalWrite(pwm_pin2, LOW);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  // digital write FIRST
  double ei= 1.0;
  while(true){
    counter ++;
    thetaDis = (float)(max_rads - min_rads) * getPot(potPin);
    theta = getVelocity();  
    e = (float)(thetaDis - theta);
    if( ei > 0.0)
      ei = (float)e + (float)ei;
    else
      ei = e;
    
    vel = (float)((float)(ki * ei) + (float)(e * kp) + (float)(kd*(e - lastE)));
    lastE = e;
    //vel = doControlE(thetaDis,theta);
    powerMotor((float)abs(vel)/max_rads, motorLeft);
    if((counter % 300) == 0){
      //Serial.println("theta DIS " + String(thetaDis) + " rad/s");
      //Serial.println("recorded theta " + String(theta) + " rad/s");
      //Serial.println("ei " + String(ei));
      //Serial.println("Vel " + String(vel));
      //Serial.println("E: " + String(e));
      Serial.println(String(thetaDis) + " " + String(theta));
    }
    delay(1);
  }  
}

float getVelocity(){
  //Serial.println(getPot(potPin));
  //powerMotor(0.5,motorLeft);
  r = digitalRead(irSensor);
  if (r == rr)
    j = j+1;
  if (r != rr){
    dj = j;
    j = 0;
    rr =r;
  }
  dt = (float)dj * T;
  
  return piii/dt;
}

float doControlE(float thetaDis,float theta){
  e = (float)(thetaDis - theta);
  //return eTemp;
  //float newEi = (eTemp + ei);
  //ei = newEi;
  return (float)e; //+ //(ki* ei));
}

float getState(){
  return (float)25.0 * getPot(potPin);
}

float getPot(int potPinIn){
  float potVal = -1.0;
  float resultVel = 0.0;
  if(potPin == potPinIn){
    potVal = analogRead(potPinIn);
    //Serial.println(potVal);
  } else {
    return e_pot;
  }
  if(potVal > (pot_mid + 15)){
    
    //pot val is between 512 and 1023
    resultVel = (float)map(potVal,512,1023,0, 1);
    //resultVel = ((float)(potVal - pot_mid)/512);// between 0 and 1
  }
  else if( potVal < (pot_mid - 15)){
    resultVel = -1.0 * (float)map(potVal,0,511,1, 0);
    //resultVel = -((float)(potVal/512));// between -1 and 0
  }
  return resultVel;
}

int powerMotor(float vel, int motorPin){
  //Serial.println(stuff);
  if(vel > 0){
    if(setDirection(motorPin, counterClockWise) != e_ok)
      return e_dir;
    else{
      return (setSpeed(motorPin,vel));
    }
    
  }
  else if(vel < 0){
    if(setDirection(motorPin, clockwise) != e_ok)
      return e_dir;
    else{
      return (setSpeed(motorPin,-vel));
    }
  }

}


int setDirection(int motorPin, int dir){
  if(motorLeft == motorPin){
    if(clockwise == dir){
      digitalWrite(motorLeft_Dir_cntl_1 , HIGH);
      digitalWrite(motorLeft_Dir_cntl_2 , LOW);
      return e_ok;
    }
    else if(counterClockWise == dir){
      digitalWrite(motorLeft_Dir_cntl_1 , LOW);
      digitalWrite(motorLeft_Dir_cntl_2 , HIGH);
      return e_ok;
    } else {
      return e_dir;
    }
  } else if( motorRight == motorPin){
    if(clockwise == dir){
      digitalWrite(motorRight_Dir_cntl_1 , HIGH);
      digitalWrite(motorRight_Dir_cntl_2 , LOW);
      return e_ok;
    }
    else if(counterClockWise == dir){
      digitalWrite(motorRight_Dir_cntl_1 , LOW);
      digitalWrite(motorRight_Dir_cntl_2 , HIGH);
      return e_ok;
    } else {
      return e_dir;
    }
  }
  return e_mot;  
}


int setSpeed(int motorPin, float speed){

  if(1.0 < speed){
    speed = 1.0;
  }
  else if (0.0 > speed){
    speed = 0.0;
  }

  int scaleSpeed = int(speed*(mot_max_val - mot_min_val));
  int pinProgram = -1;

  if(motorLeft == motorPin){
    pinProgram = motorLeft;
  }
  else if(motorRight == motorPin){
    pinProgram = motorRight;
  }
  else{
    return e_mot;
  }
  analogWrite(pinProgram,scaleSpeed);
  return scaleSpeed;
}
  


