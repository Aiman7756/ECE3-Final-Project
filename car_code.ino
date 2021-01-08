
//Aiman's Code

#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int right_nslp_pin = 11;
const int left_dir_pin=29;
const int left_pwm_pin=40;
const int right_pwm_pin=39;
const int right_dir_pin=30;

int left = 0;
int right = 0;

int linecounter = 0;

float prev_error = 0;
float error = 0;
float speedChange = 0;

const double kp = .02;
const double kd = .02;

const int speed = 45;

///////////////////////////////////
void setup() {
// put your setup code here, to run once:

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  ECE3_Init();
  Serial.begin(9600); 
  delay(2000);
  
}

void loop() {

   ECE3_read_IR(sensorValues);
    int leftCount = getEncoderCount_left();
    int rightCount = getEncoderCount_right();
   
    error = ((sensorValues[0] * -15) + (sensorValues[1] * -14) + (sensorValues[2] * -12) + (sensorValues[3] * -8) + (sensorValues[4] * 8)   + (sensorValues[5] * 12)  + (sensorValues[6] * 14)  + (sensorValues[7] * 15))/8;

      if(leftCount >= 2000 && rightCount >= 2000){
        
      if (linecounter == 0){
        digitalWrite(right_dir_pin,HIGH);
        delay(1500);
        digitalWrite(right_dir_pin, LOW);
        linecounter++;
        resetEncoderCount_left();
        resetEncoderCount_right();
      }
      else if (linecounter == 1){
        digitalWrite(left_nslp_pin,LOW);
        digitalWrite(right_nslp_pin,LOW);
      }
    }

    


   speedChange = kp * error + (kd * (prev_error - error));
    
      if (speedChange > 30){ // turning right
      left =  (speed - speedChange);
      right =  .7 * (speed + speedChange);
      }
      
      if (speedChange < -30){ //turning left
      right = (speed + speedChange);
      left =  .7 * (speed - speedChange);
      }
      
      else { //turning left
      right = (speed + speedChange);
      left =  (speed - speedChange);
      }
        

  analogWrite(left_pwm_pin,left);
  analogWrite(right_pwm_pin,right);

  prev_error = error;


  }
