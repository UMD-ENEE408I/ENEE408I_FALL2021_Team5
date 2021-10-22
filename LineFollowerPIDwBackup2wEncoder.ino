#include <Adafruit_MCP3008.h>
#include <Encoder.h>

/*
 * Jerell Line follower with intersection detection.
 * Last edited 10/13/2021
 * 
 * Retrieved from github under "FollowtheLine_PID.ino"
 * 
 * 1080 ticks = 1 unit of maze distance
 */

//Forward
const unsigned int FULL_SPEED_FORWARD = 35;

 //Buzz
 const unsigned int BUZZ = 10;

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

//const unsigned int BLACK = 650;
const int BLACK = 680;

bool atRight = false;
bool atLeft = false;
bool atIntersection = false;
bool onWhiteLine = false;

//Encoder Declarations
const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;

int enc1_start_val = 0;
int enc1_end_val = 0;
int enc2_start_val = 0;
int enc2_end_val = 0;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

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

//PID variables
int P;
int D;
int I;
int error;
int previousError;
int PIDvalue;

float Kp = 4; //was 0.07 //was 0.1 //was 4      //Kp=4 and Ki=0.05 worked best for now
float Ki = 0.008; // 1 < Ki < 10      //0.05 < Ki < 0.5
float Kd = 0;

/*
 * Rover will move forward with power to left and right 
 * motor depending on direction.
 */
void moveCourseCorrect() {
  mouse_forward(FULL_SPEED_FORWARD + PIDvalue, FULL_SPEED_FORWARD - PIDvalue);
}

/*
 * Calculates PID
 */
void calculatePID() {
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  //PIDvalue = (Kp*P) + (Kd*D);
  //PIDvalue = (Kp*P);
  previousError = error;
}

/*
 * Returns the direction that the rover is heading.
 * Returns negative number for veering towards left.
 * The more negative (up to -5) the more to the left the rover is.
 * Returns positive for veering towards right.
 * The more positive (up to +5) the more to the right the rover is.
 * Returns 0 if otherwise on course.
 */
int isVeeringTowards(int val1, int val2, int val3, int val4, int val5, int val6, int val7, int val8, int val9, int val10, int val11, int val12, int val13) {
    if (isWhite(val6) && isWhite(val7) && isWhite(val8)) {
      return 0; 
    }
    else if (isWhite(val5) && isWhite(val6) && isWhite(val7)) return 1;
    
    else if (isWhite(val4) && isWhite(val5) && isWhite(val6)) return 2;
    
    else if (isWhite(val3) && isWhite(val4) && isWhite(val5)) return 3;
    
    else if (isWhite(val2) && isWhite(val3) && isWhite(val4)) return 4;
    
    else if (isWhite(val1) && isWhite(val2) && isWhite(val3)) return 5;
    
    else if (isWhite(val7) && isWhite(val8) && isWhite(val9)) return -1;

    else if (isWhite(val8) && isWhite(val9) && isWhite(val10)) return -2;

    else if (isWhite(val9) && isWhite(val10) && isWhite(val11)) return -3;

    else if (isWhite(val10) && isWhite(val11) && isWhite(val12)) return -4;

    else if (isWhite(val11) && isWhite(val12) && isWhite(val13)) return -5;

    else return 0;   //centered on the line 
}

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
  delay(10);
  M2_forward(100);
  M1_backward(100);
  delay(50);
  M1_forward(35);
  M2_forward(35);
  delay(50);
}
void mouse_right() {
  delay(10);
  M1_forward(100);
  M2_backward(100);
  delay(50);
  M1_forward(35);
  M2_forward(35);
  delay(50);
}

void print_encoder_loop() {
  Serial.print("enc1_start_val: ");
  Serial.print(enc1_start_val);
  Serial.print("\t");
  Serial.print("enc2_start_val: ");
  Serial.print(enc2_start_val);
  Serial.print("\t");
  Serial.print("enc1_end_val: ");
  Serial.print(enc1_end_val);
  Serial.print("\t");
  Serial.print("enc2_end_val: ");
  Serial.print(enc2_end_val);
  Serial.print("\t");
  Serial.println();
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

  //At Right Turn
  if (((isWhite(adc2_buf[0])) && (isWhite(adc1_buf[1])) && (isWhite(adc2_buf[1])) && (isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3])))&&!(((isWhite(adc2_buf[4]))||(isWhite(adc1_buf[5]))||(isWhite(adc2_buf[5]))||(isWhite(adc1_buf[6]))))) {
    atRight = true;
  }
   else {
    atRight = false; 
  }
  //At Left Turn
   if (((isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4])) && (isWhite(adc2_buf[4])) && (isWhite(adc1_buf[5])) && (isWhite(adc2_buf[5]))&&(isWhite(adc1_buf[6])))&&!((isWhite(adc1_buf[0]))||(isWhite(adc2_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc2_buf[1])))){
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
       mouse_forward(35,35);
       delay(50); 
  }
  //Stop if at an intersection MUST IMPLEMENT HANDLE INTERSECTION
  else if (atIntersection == true) {
    enc1_end_val = enc1.read();
    enc2_end_val = enc2.read();
    mouse_stop();
    for (int i = 0; i < 15; i++) {
      print_encoder_loop();
      delay(1000);
    }
    /*for (int i = 0; i < 10; i++) {
    tone(BUZZ, 3000+(i*200), 100);
    delay(100);
    }
    mouse_right();
    Serial.print("Turned right at intersection \n");
    delay(100);*/ 
  }
  else if (!atIntersection && atRight == true){ 
    enc1_end_val = enc1.read();
    enc2_end_val = enc2.read();
    mouse_stop();
    for (int i = 0; i < 15; i++) {
      print_encoder_loop();
      delay(1000);
    }
    /*mouse_stop();
    //HANDLE INTERSECTION
    //mouse_left();
     for (int i = 0; i < 10; i++) {
      //tone(BUZZ, 3000+(i*200), 100);
      delay(25);
     }
    mouse_right();
    Serial.print("Turned right \n");
    delay(100);*/
  }

    else if (!atIntersection && atLeft == true){ 
     enc1_end_val = enc1.read();
    enc2_end_val = enc2.read();
    mouse_stop();
    for (int i = 0; i < 15; i++) {
      print_encoder_loop();
      delay(1000);
    }
    /*mouse_stop();
    //HANDLE INTERSECTION
     for (int i = 0; i < 10; i++) {
      // tone(BUZZ, 3000+(i*200), 100);
      delay(25);
     }
    mouse_left();
    //mouse_right();
    Serial.print("Turned Left\n");*/  
  }
    
 //On the line
  else if (onWhiteLine && !atIntersection && !atRight && !atLeft) {
    error = isVeeringTowards(adc1_buf[0], adc2_buf[0], adc1_buf[1], adc2_buf[1], adc1_buf[2], adc2_buf[2], adc1_buf[3], adc2_buf[3], adc1_buf[4], adc2_buf[4], adc1_buf[5], adc2_buf[5], adc1_buf[6]);
    calculatePID();
    moveCourseCorrect();
    if (enc1_start_val == 0) {
      enc1_start_val = enc1.read();
      enc2_start_val = enc2.read();
    }
    /*int direction = isVeeringTowards(adc1_buf[0], adc2_buf[0], adc1_buf[1], adc2_buf[1], adc1_buf[2], adc2_buf[2], adc1_buf[3], adc2_buf[3], adc1_buf[4], adc2_buf[4], adc1_buf[5], adc2_buf[5], adc1_buf[6]);
    moveCourseCorrect(direction);*/
  }
  else {
    Serial.print("end of TAKE ACTION BASED ON WHEREABOUTS");
    mouse_stop();
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
 /* else {
    Serial.print("end of TAKE ACTION BASED ON WHEREABOUTS");
    mouse_stop();
  }*/
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

}//end void loop
