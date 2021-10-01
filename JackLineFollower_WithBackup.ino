#include <Adafruit_MCP3008.h>

/*
 * Jerell Line follower with intersection detection.
 * Last edited 09/22/21
 */

 //Buzz
 const unsigned int BUZZ = 10;

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

//const unsigned int BLACK = 650;
const unsigned int BLACK = 680;

bool atRight = false;
bool atLeft = false;
bool atIntersection = false;
bool onWhiteLine = false;

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

unsigned int PWM_M1_VALUE = 50; //was 150
unsigned int PWM_M2_VALUE = 50; //was 150

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
//Function for seeing white line
bool isWhite(int x) {
  if (x < BLACK) {
    return true;
  }
  else return false;
}
//Custom Motor Functions
void M1_forward(int x) {
  PWM_M1_VALUE = x;
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, x);
}
void M2_forward(int x) {
  PWM_M2_VALUE = x;
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, x);
}
void M1_backward(int x) {
 //PWM_M1_VALUE = x;
  analogWrite(M1_IN_1, x);
  analogWrite(M1_IN_2, 0);
}
void M2_backward(int x) {
  //PWM_M2_VALUE = x;
  analogWrite(M2_IN_1, x);
  analogWrite(M2_IN_2, 0);
}
void mouse_forward(int x, int y) {
  M1_forward(x);
  M2_forward(y);
}
void mouse_backward(int x, int y) {
  M1_backward(x);
  M2_backward(y);
}
void mouse_stop() {
  M1_stop();
  M2_stop();
}
void mouse_left() {
  M2_forward(100);
  M1_backward(100);
  delay(50);
  mouse_stop();
}
void mouse_right() {
  M1_forward(100);
  M2_backward(100);
  delay(50);
  mouse_stop();
}

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
    //Serial.print(adc1_buf[i]); Serial.print("\t");
    //Serial.print(adc2_buf[i]); Serial.print("\t");
    
   //Serial.print("adc1-"); Serial.print(i);Serial.print("-"); Serial.print(adc1_buf[i]); Serial.print("\t");
   //Serial.print("adc2-"); Serial.print(i);Serial.print("-"); Serial.print(adc2_buf[i]); Serial.print("\t");
  }



  //____________________WHEREABOUTS CONDITIONS______________________________________
  //Detects when the mouse is completely off of the line
  if ((isWhite(adc2_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc2_buf[1])) || (isWhite(adc1_buf[2])) || (isWhite(adc2_buf[2])) || (isWhite(adc1_buf[3])) || (isWhite(adc2_buf[3])) || (isWhite(adc1_buf[4])) || (isWhite(adc2_buf[4])) || (isWhite(adc1_buf[5])) || (isWhite(adc2_buf[5])) || (isWhite(adc1_buf[6]))) {
    onWhiteLine = true;
  }
  else {
    onWhiteLine = false;
  }
  
  //Detects full white T-intersection
  /*adc1_buf[0] (farthest right sensor) does not go below 650 
  to detect white line. All other sensors work properly.*/
  if ((isWhite(adc2_buf[0])) && (isWhite(adc1_buf[1])) && (isWhite(adc2_buf[1])) && (isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3])) && (isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4])) && (isWhite(adc2_buf[4])) && (isWhite(adc1_buf[5])) && (isWhite(adc2_buf[5])) && (isWhite(adc1_buf[6]))) {
    atIntersection = true;
  }
  else {
    atIntersection = false;
  }

  //At Left Turn
  if ((isWhite(adc2_buf[0])) && (isWhite(adc1_buf[1])) && (isWhite(adc2_buf[1])) && (isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3]))) {
    atRight = true;
  }
   else {
    atRight = false; 
  }
  //At Right Turn
   if ((isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4])) && (isWhite(adc2_buf[4])) && (isWhite(adc1_buf[5])) && (isWhite(adc2_buf[5])) && (isWhite(adc1_buf[6]))) {
    atLeft = true;
   }
   else { 
    atLeft = false; 
   }

  //____________________TAKE ACTION BASED ON WHEREABOUTS___________________________________
  //Stop if completely off the white line
  if (onWhiteLine == false) {
       M1_backward(40);
       M2_backward(40);
       for (int i = 0; i < 10; i++) {
        tone(BUZZ, 3000+(i*200), 100);
       delay(40);
       }
       mouse_forward(40, 40);
       delay(50); 
  }
  //Stop if at an intersection MUST IMPLEMENT HANDLE INTERSECTION
  else if (atIntersection == true) {
    for (int i = 0; i < 10; i++) {
    tone(BUZZ, 3000+(i*200), 100);
    delay(100);
    }
    mouse_right();
    Serial.print("turned");
    delay(100);
  }
  else if (!atIntersection && atRight == true){ 
    mouse_stop();
    //HANDLE INTERSECTION
    //mouse_left();
    mouse_right();
    Serial.print("turned right");
    delay(100);
  }

    else if (!atIntersection && atLeft == true){ 
    mouse_stop();
    //HANDLE INTERSECTION
    mouse_left();
    //mouse_right();
    Serial.print("turned right");
    delay(100);
  }
    
  //Move forward if on white line and not at intersection
  else if (onWhiteLine && !atIntersection) {
    if ((isWhite(adc1_buf[3])) || (isWhite(adc2_buf[2])) || (isWhite(adc2_buf[3]))) {
      mouse_forward(35, 35);
    }
    else if((isWhite(adc2_buf[3])) || (isWhite(adc2_buf[4])) || (isWhite(adc2_buf[5])) || (isWhite(adc1_buf[4])) || (isWhite(adc1_buf[5])) || (isWhite(adc1_buf[6]))) {
    //left sensor of middle is adc2_buf[2]
    //left sensor of middle is adc2_buf[3]
      if( (PWM_M1_VALUE - PWM_M2_VALUE) < 2){
        if( PWM_M2_VALUE > 60 ) {
          M1_forward(PWM_M1_VALUE - 0.2); //Changed
          PWM_M1_VALUE = PWM_M1_VALUE -0.2;
        }
        //else {
          M2_forward(PWM_M2_VALUE + 0.2); //changed from +2 to +4 //removed
          //M1_forward(PWM_M1_VALUE - 4); //added //changed from -2 to -4
        //}        
        //M2_forward(PWM_M2_VALUE);
        //M1_forward(PWM_M1_VALUE); //added
     }
    }
     else if((isWhite(adc2_buf[0])) || (isWhite(adc2_buf[1])) || (isWhite(adc2_buf[2])) || (isWhite(adc1_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc1_buf[2]))) {
        if( (PWM_M2_VALUE - PWM_M1_VALUE) < 2){
          //PWM_M2_VALUE = PWM_M2_VALUE - 1 ;
          if(PWM_M1_VALUE > 60 ) {
            M2_forward(PWM_M2_VALUE - 0.2); //Changed
              PWM_M2_VALUE = PWM_M2_VALUE -0.2;

          }
          //else {
            M1_forward(PWM_M1_VALUE + 0.2); //changed from +1 to +2 //removed
          //}
          //M1_forward(PWM_M1_VALUE);
          //M2_forward(PWM_M2_VALUE);
      }
    }
    else { 
       M1_backward(250);
       M2_backward(250);
       for (int i = 0; i < 10; i++) {
      tone(BUZZ, 3000+(i*200), 100);
       delay(100);
       delay(1000); 
      }
    }
    
   delay(10);
   M1_stop();
   M2_stop();

    Serial.print(t_end - t_start);
    /*Serial.print("atIntersection:");
    Serial.print(atIntersection);
    Serial.print("\t");
    Serial.print("onWhiteLine:");
    Serial.print(onWhiteLine);
    Serial.println();*/

    //delay(20);   //changed from 30 to 60
  }
  else {
    Serial.print("end of TAKE ACTION BASED ON WHEREABOUTS");
    mouse_stop();
  }
    Serial.print("atIntersection:");
    Serial.print(atIntersection);
    Serial.print("\t");
    Serial.print("onWhiteLine:");
    Serial.print(onWhiteLine);
    Serial.print("atLeft:");
    Serial.print(atLeft);
    Serial.print("\t");
    Serial.print("atRight:");
    Serial.print(atRight);
    Serial.print("\t");
    Serial.println();

} //end void loop
