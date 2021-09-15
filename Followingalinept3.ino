#include <Adafruit_MCP3008.h>

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
   Serial.print("adc1-"); Serial.print(i);Serial.print("-"); Serial.print(adc1_buf[i]); Serial.print("\t");
   Serial.print("adc2-"); Serial.print(i);Serial.print("-"); Serial.print(adc2_buf[i]); Serial.print("\t");
  }

//  if ((adc1_buf[3]) > 680){
//    M1_stop();
//    M2_stop();
//  } 
   if ((adc1_buf[3] < 680)){
    PWM_M1_VALUE = 215;
    PWM_M2_VALUE = 215;
     M1_forward();
     M2_forward();
  }
  else if((adc2_buf[3] < 680  || adc2_buf[4] < 680 || adc2_buf[5] < 680 || adc1_buf[4] < 680 || adc1_buf[5] < 680  || adc1_buf[6] < 680 )){
    //left sensor of middle is adc2_buf[2]
    //left sensor of middle is adc2_buf[3]
//    if(PWM_M1_VALUE > 50){
//     PWM_M1_VALUE = PWM_M1_VALUE - 1;
//     M1_forward();
//     M2_forward();
//    }
//    else{
//       PWM_M2_VALUE = PWM_M2_VALUE + 3;
//       M1_forward();
//       M2_forward();
//    }  

        if( PWM_M2_VALUE > 240 ){
           PWM_M1_VALUE = PWM_M1_VALUE - 2;
        }
        PWM_M2_VALUE = PWM_M2_VALUE + 2;
        
       // PWM_M1_VALUE = PWM_M1_VALUE - 1;
        M2_forward();
       // M1_forward();
   }
  
  else if((adc2_buf[0] < 680  || adc2_buf[1] < 680 || adc2_buf[2] < 680 || adc1_buf[0] < 680 || adc1_buf[1] < 680  || adc1_buf[2] < 680 )){

//    if(PWM_M2_VALUE>50){
//    //left sensor of middle is adc2_buf[2]
//    //left sensor of middle is adc2_buf[3]
//    PWM_M2_VALUE = PWM_M2_VALUE - 2;
//    M1_forward();
//    M2_forward();
//    }
//    else{
//    PWM_M1_VALUE = PWM_M1_VALUE+2;
//    M1_forward();
//    M2_forward();
//    }

        //PWM_M2_VALUE = PWM_M2_VALUE - 1 ;
        if( PWM_M1_VALUE > 240 ){
           PWM_M1_VALUE = PWM_M1_VALUE - 2;
        }
        PWM_M1_VALUE = PWM_M1_VALUE + 1;
        M1_forward();
         M2_forward();
  }
  else{ 
    M1_stop();
    M2_stop();
  }
  

    delay(10);
    M1_stop();
    M2_stop();

  

  Serial.print(t_end - t_start);
  Serial.println();

  delay(50);
}
