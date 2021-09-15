#include <Adafruit_MCP3008.h>

/*
 * Code properly detects a T-intersection and whether or not
 * the mouse is not on a line. Shows boolean value in far right of Serial Printer.
 * 
 * Far right sensor of Jeff's mouse does not report a value below the threshold 
 * so it is not used (may vary for other mice).
 */

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

//ADC Intigers
const unsigned int ADC_1_CS = A3;
const unsigned int ADC_2_CS = A2;

//Motor Inigers
const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

unsigned int PWM_M1_VALUE = 150;
unsigned int PWM_M2_VALUE = 150;

const unsigned int BLACK = 650;

bool atIntersection = false;
bool onWhiteLine = false;

void setup() {
  Serial.begin(115200);

  //ADC Readings
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  

  //Motor
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);
}

//Motor Function
void M1_backward() {
  analogWrite(M1_IN_1, PWM_M1_VALUE);
  analogWrite(M1_IN_2, 0);
}

void M1_backward(int m1) {
  analogWrite(M1_IN_1, m1);
  analogWrite(M1_IN_2, 0);
}

void M1_forward() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM_M1_VALUE);
}

void M1_stop() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}

void M2_backward() {
  analogWrite(M2_IN_1, PWM_M2_VALUE);
  analogWrite(M2_IN_2, 0);
}

void M2_backward(int m2) {
  analogWrite(M2_IN_1, m2);
  analogWrite(M2_IN_2, 0);
}

//CUSTOM
void backward(int m1, int m2) {
  analogWrite(M1_IN_1, m1);
  analogWrite(M1_IN_2, 0);

  analogWrite(M2_IN_1, m2);
  analogWrite(M2_IN_2, 0);
}

void M2_forward() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM_M1_VALUE);
}

void M2_stop() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}

///////////////


void loop() {
  int adc1_buf[8];
  int adc2_buf[8];
  
  int t_start = micros();
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
  int t_end = micros();

  for (int i = 0; i < 8; i++) {
    Serial.print(adc1_buf[i]); Serial.print("\t");
    Serial.print(adc2_buf[i]); Serial.print("\t");
  }

  //WHEREABOUTS CONDITIONS
  //Detects 
  if ((adc2_buf[0] < BLACK) || (adc1_buf[1] < BLACK) || (adc2_buf[1] < BLACK) || (adc1_buf[2] < BLACK) || (adc2_buf[2] < BLACK) || (adc1_buf[3] < BLACK) || (adc2_buf[3] < BLACK) || (adc1_buf[4] < BLACK) || (adc2_buf[4] < BLACK) || (adc1_buf[5] < BLACK) || (adc2_buf[5] < BLACK) || (adc1_buf[6] < BLACK)) {
    onWhiteLine = true;
  }
  //Detects full white T-intersection
  //adc1_buf[0] (farthest right sensor) does not go below 650 to detect white line. All other sensors work properly.
  if ((adc2_buf[0] < BLACK) && (adc1_buf[1] < BLACK) && (adc2_buf[1] < BLACK) && (adc1_buf[2] < BLACK) && (adc2_buf[2] < BLACK) && (adc1_buf[3] < BLACK) && (adc2_buf[3] < BLACK) && (adc1_buf[4] < BLACK) && (adc2_buf[4] < BLACK) && (adc1_buf[5] < BLACK) && (adc2_buf[5] < BLACK) && (adc1_buf[6] < BLACK)) {
    atIntersection = true;
    //HANDLE INTERSECTION
    //atIntersection = false;
    //delay(3000);
    
  }

  //MOTION CONDITIONS
  /*if (((adc1_buf[3]) > 650) && (atIntersection == true) || (onWhiteLine == false)){
    M1_stop();
    M2_stop();
  }*/
  if ((atIntersection == true) || (onWhiteLine == false)){
    M1_stop();
    M2_stop();
  }
  else if((adc1_buf[3] < BLACK) && ((adc2_buf[2] - adc2_buf[3]) > 50)){
    //left sensor of middle is adc2_buf[2]
    //right sensor of middle is adc2_buf[3]
    if(PWM_M1_VALUE > 50){
     PWM_M1_VALUE = PWM_M1_VALUE - 1;
     M1_forward();
     M2_forward();
    }
    else{
       PWM_M2_VALUE = PWM_M2_VALUE + 1;
       M1_forward();
       M2_forward();
    }  
   }
  
  else if(((adc1_buf[3]) < BLACK) && ((adc2_buf[3] - adc2_buf[2]) > 50)){

    if(PWM_M2_VALUE>50){
    //left sensor of middle is adc2_buf[2]
    //right sensor of middle is adc2_buf[3]
    PWM_M2_VALUE = PWM_M2_VALUE - 1;
    M1_forward();
    M2_forward();
    }
    else{
    PWM_M1_VALUE = PWM_M1_VALUE+1;
    M1_forward();
    M2_forward();
    }
  }

  else if ((adc2_buf[0] >= BLACK) && (adc1_buf[1] >= BLACK) && (adc2_buf[1] >= BLACK) && (adc1_buf[2] >= BLACK) && (adc2_buf[2] >= BLACK) && (adc1_buf[3] >= BLACK) && (adc2_buf[3] >= BLACK) && (adc1_buf[4] >= BLACK) && (adc2_buf[4] >= BLACK) && (adc1_buf[5] >= BLACK) && (adc2_buf[5] >= BLACK) && (adc1_buf[6] >= BLACK)) {
    M1_stop();
    M2_stop();
    onWhiteLine = false;
    
  }
  else{ 
    M1_forward();
    M2_forward();
  }
  

  Serial.print(t_end - t_start);
  Serial.print("atIntersection:");
  Serial.print(atIntersection);
  Serial.print("\t");
  Serial.print("onWhiteLine:");
  Serial.print(onWhiteLine);
  Serial.println();

  delay(10);
}
