#include <Adafruit_MCP3008.h>
#include <Encoder.h>


 /* FEATURES INCLUDED:
 * - PID Controller
 * - Jerell Intersection Correction
 * - Unit Length Distance Tracking
 * - Troubleshooting backup when off the line
 * - Maze Mapping
 */   
 
//Encoder Declaraitons
const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;

int enc1_start = 0;
int enc1_end = 0;
int enc2_start = 0;
int enc2_end = 0;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

bool enc_set = false;
const int MAZE_UNIT_LENGTH = 1080; //was 1080
const int ENCODER_ERROR_BOUND = 50;
///////////////////////////////////////////////

//ADC Declarations
int adc1_buf[8];
int adc2_buf[8];
////////////////////////////////////////////
  
//Maze Mapping Declarations 
char MazeMap[11][11][8];
  /*fieds per node are:
  0: Is Node: T = True, F = False
  1: Is there a Node Forward: T = True, F = False
  2: Is there a Node to the Left: T = True, F = False
  3: Is there a Node to the Right: T = True, F = False
  4: Is There a Node Behind: T = True, F = False
  5: What Direction did we enter the Node: N = North, S = South, W = West, E = East
  6  Have we ever Visisted this Node: U = Unvisited, V = Visisted
  7: Is this Node the Exit: T = True, F = False
  */

int MazeX = 5;  //Initial X Position for the MazeMap is the Center
int MazeY = 5;  //Initial Y Position for the MazeMap is the Center
char Direction = 'N';    //Initial Direction will always be assumed to be North
//////////////////////////////////////////////////////////////////////////////////////

//Forward
const unsigned int FULL_SPEED_FORWARD = 32.5;

 //Buzz
 const unsigned int BUZZ = 10;

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

//const unsigned int BLACK = 650;
const int BLACK = 680;

//Booleans 
bool atRight = false;
bool atLeft = false;
bool atIntersection = false;
bool onWhiteLine = false;
bool atExit = false;
bool IsForward = false; 

//bool intersectioncheck = false;
int counterintcheck = 0;

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

float Kp = 4;
float Kd = 0;
float Ki = 0.008;

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

/*
 * Returns the total distance travelled relative to starting encoder tick value.
 */
int distanceTravelled(int enc_start, int enc_end) {
  return abs(enc_start - enc_end);
}

/*
 * Takes in a current enc1 and enc2 value and returns whether or not 
 * a distance of UNIT_MAZE_LENGTH has been travelled.
 * 
 * Distance is measured in units of Encoder Ticks.
 */
bool travelledUnitLength(int enc1, int enc2) {
  bool enc1_travelled_unit_length = false;
  bool enc2_travelled_unit_length = false;
  if (distanceTravelled(enc1_start, enc1) > MAZE_UNIT_LENGTH - ENCODER_ERROR_BOUND
        && distanceTravelled(enc1_start, enc1) < MAZE_UNIT_LENGTH + ENCODER_ERROR_BOUND) {
          enc1_travelled_unit_length = true;
  }
  if (distanceTravelled(enc2_start, enc2) > MAZE_UNIT_LENGTH - ENCODER_ERROR_BOUND
        && distanceTravelled(enc2_start, enc2) < MAZE_UNIT_LENGTH + ENCODER_ERROR_BOUND) {
          enc2_travelled_unit_length = true;
  }
  return (enc1_travelled_unit_length && enc2_travelled_unit_length); //if both travelled unit length return true
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

  int m;
  int l; 
  for (m = 0; m<11; m++){
    for (l = 0; l<11;l++){
      MazeMap[m][l][0] = 'X'; 
      MazeMap[m][l][1] = 'X';
      MazeMap[m][l][2] = 'X'; 
      MazeMap[m][l][3] = 'X'; 
      MazeMap[m][l][4] = 'X'; 
      MazeMap[m][l][5] = 'X'; 
      MazeMap[m][l][6] = 'X'; 
      MazeMap[m][l][7] = 'X';   
    }
  }
  MazeMap[5][5][0] = 'T';
}

void MazeMapping(int* MazeX, int* MazeY, char Direction, bool atIntersection, bool atRight, bool atLeft, bool atExit, bool travelledUnitLength, bool IsForward){
  switch (Direction){ 
    case 'N':
      *MazeY = *MazeY - 1; 
      break; 
    case 'S': 
      *MazeY = *MazeY + 1; 
      break; 
    case 'E':
      *MazeX = *MazeX + 1;
      break; 
    case 'W': 
      *MazeX = *MazeX - 1; 
      break; 
  }

  MazeMap[*MazeX][*MazeY][0] = 'T'; //Set to is node
  
  if(MazeMap[*MazeX][*MazeY][6] != 'V'){
     MazeMap[*MazeX][*MazeY][6] = 'U'; //Node is unvisited
  }
  if(atIntersection == true){
      MazeMap[*MazeX][*MazeY][1] = 'F';  //Add Forward Detection
      if(IsForward){ 
      MazeMap[*MazeX][*MazeY][1] = 'T';  //Add Forward Detection
      } 
      MazeMap[*MazeX][*MazeY][2] = 'T'; //There is a Node to the Left 
      MazeMap[*MazeX][*MazeY][3] = 'T';  //There is a Node to the right
      MazeMap[*MazeX][*MazeY][4] = 'T';   //There is a Node Behind
      MazeMap[*MazeX][*MazeY][5] = Direction;   //Direction entered is equal to the Direction variable
       switch (Direction){ 
        case 'N':
          MazeMap[(*MazeX)+1][*MazeY][0] = 'T'; //Set IsNode field to the Left to True (East and West have Nodes)
          MazeMap[(*MazeX)-1][*MazeY][0] = 'T'; //Set IsNode field to the Right to True
          break; 
        case 'S': 
          MazeMap[(*MazeX)+1][*MazeY][0] = 'T'; //Set IsNode field to the Left to True (East and West have Nodes)
          MazeMap[(*MazeX)-1][*MazeY][0] = 'T'; //Set IsNode field to the Right to True
          break; 
        case 'E':
          MazeMap[*MazeX][(*MazeY)+1][0] = 'T'; //Set IsNode field to the Left to True (North and South have nodes)
          MazeMap[*MazeX][(*MazeY)-1][0] = 'T'; //Set IsNode field to the Right to True
          break; 
        case 'W': 
          MazeMap[*MazeX][(*MazeY)+1][0] = 'T'; //Set IsNode field to the Left to True (North and South Have Nodes)
          MazeMap[*MazeX][(*MazeY)-1][0] = 'T'; //Set IsNode field to the Right to True
          break; 
      }    
  }
  
  else if(!atIntersection && atRight == true){  
      MazeMap[*MazeX][*MazeY][1] = 'F';  //Add Forward Detection
      if(IsForward){ 
      MazeMap[*MazeX][*MazeY][1] = 'T';  //Add Forward Detection
      } 
      MazeMap[*MazeX][*MazeY][2] = 'F';   //There is a Node to the Left 
      MazeMap[*MazeX][*MazeY][3] = 'T';   //There is a Node to the Right
      MazeMap[*MazeX][*MazeY][4] = 'T';   //There is a Node Behind
      MazeMap[*MazeX][*MazeY][5] = Direction;   //Direction entered is equal to the Direction variable  
      switch (Direction){ 
        case 'N':
          MazeMap[(*MazeX)+1][*MazeY][0] = 'T'; //Set IsNode field of Node to the Right to True (East has a Node)
          break; 
        case 'S': 
          MazeMap[(*MazeX)-1][*MazeY][0] = 'T'; //Set IsNode field of Node to the Right to True (West has a Node)
          break; 
        case 'E':
          MazeMap[*MazeX][(*MazeY)+1][0] = 'T'; //Set IsNode field to the Right to True (South has a Node)
          break; 
        case 'W': 
          MazeMap[*MazeX][(*MazeY)-1][0] = 'T'; //Set IsNode field to the Left to True (North has a node)
          break; 
      }  
  }
  
  else if(!atIntersection && atLeft == true){ 
      MazeMap[*MazeX][*MazeY][1] = 'F';  //Add Forward Detection
      if(IsForward){ 
      MazeMap[*MazeX][*MazeY][1] = 'T';  //Add Forward Detection
      } 
      MazeMap[*MazeX][*MazeY][2] = 'T';   //There is a Node to the Left 
      MazeMap[*MazeX][*MazeY][3] = 'F';   //There is no Node to the right
      MazeMap[*MazeX][*MazeY][4] = 'T';   //There is a Node Behind
      MazeMap[*MazeX][*MazeY][5] = Direction;   //Direction entered is equal to the Direction variable
      switch (Direction){ 
        case 'N':
          MazeMap[(*MazeX)-1][*MazeY][0] = 'T'; //Set IsNode field of Node to the Right to True
          break; 
        case 'S': 
          MazeMap[(*MazeX)+1][*MazeY][0] = 'T'; //Set IsNode field of Node to the Right to True
          break; 
        case 'E':
          MazeMap[*MazeX][(*MazeY)-1][0] = 'T'; //Set IsNode field to the Right to True
          break; 
        case 'W': 
          MazeMap[*MazeX][(*MazeY)+1][0] = 'T'; //Set IsNode field to the Left to True
          break; 
      }  
  }
  
  else if(travelledUnitLength == true){
      MazeMap[*MazeX][*MazeY][1] = 'T';   //Add Forward Detection
      MazeMap[*MazeX][*MazeY][2] = 'F';   //There is no node to the Left 
      MazeMap[*MazeX][*MazeY][3] = 'F';   //There is no node to the right
      MazeMap[*MazeX][*MazeY][4] = 'T';   //There is a Node Behind
      MazeMap[*MazeX][*MazeY][5] = Direction;   //Direction entered is equal to the Direction variable
      switch (Direction){ 
        case 'N':
          MazeMap[*MazeX][(*MazeY)-1][0] = 'T'; //Set IsNode field to the Front is True
          break; 
        case 'S': 
          MazeMap[*MazeX+1][(*MazeY)+1][0] = 'T'; //Set IsNode field to the Front
          break; 
        case 'E':
          MazeMap[(*MazeX)+1][*MazeY][0] = 'T'; //Set IsNode field to the Front is True
          break; 
        case 'W': 
          MazeMap[(*MazeX)-1][*MazeY][0] = 'T'; //Set IsNode field to the Front is True
          break; 
      }  
  }

  if(atExit == true){
      MazeMap[*MazeX][*MazeY][5] = Direction;   //Direction entered is equal to the Direction variable
      MazeMap[*MazeX][*MazeY][7] = 'T';          //The Mouse has reached the Exit this is the end node
  }
     Serial.print("My Postion: "); Serial.print("X = "); Serial.print(*MazeX); Serial.print(" Y = "); Serial.print(*MazeY); Serial.print("\n");
     Serial.print("My Direction: "); Serial.print(Direction); Serial.print("\n \n");
     
     for(int i = 0; i<11; i++){
      for(int j = 0; j<11; j++){
        Serial.print("Node-");
        Serial.print(j);
        Serial.print(i);
        Serial.print(": ");
        Serial.print(MazeMap[j][i][0]);
        Serial.print("\t"); 
      }
      Serial.print("\n");
    }
}

// ADC Read
void ADCRead(){ 
   for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  } 
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
   M2_forward(40);
   M1_backward(40);
   delay(150);
   do {
     M2_forward(70);
     M1_backward(80);
     delay(40);
     mouse_stop();
     delay(200);
     ADCRead(); 
      } while (!isWhite(adc1_buf[2])); //while (!isWhite(adc1_buf[3]) || !isWhite(adc2_buf[3]));
 //Move Forward After
     //Move Forward After
     P = 0;
     I = 0;
  switch (Direction){ 
        case 'N':
          Direction = 'W';
          break; 
        case 'S': 
          Direction = 'E'; 
          break; 
        case 'E':
          Direction = 'N';
          break; 
        case 'W': 
          Direction = 'S';
          break; 
      }  
}
void mouse_right() {
  M1_forward(40);
  M2_backward(40);
  delay(150);
  do {
    M1_forward(70);
    M2_backward(80);
    delay(40);
    mouse_stop();
    delay(200);
    ADCRead(); 
  } while (!isWhite(adc2_buf[3])); //while (!isWhite(adc1_buf[3]) || !isWhite(adc2_buf[3]));
 //Move Forward After
           P = 0;
           I = 0;
  switch (Direction){ 
        case 'N':
          Direction = 'E';
          break; 
        case 'S': 
          Direction = 'W'; 
          break; 
        case 'E':
          Direction = 'S';
          break; 
        case 'W': 
          Direction = 'N';
          break; 
      } 
}

void loop() {
  //int adc1_buf[8];
  //int adc2_buf[8];
  
  int t_start = micros();
/*  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }*/ 
  ADCRead(); 
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
    counterintcheck = counterintcheck + 1;
    if (counterintcheck == 3){
      counterintcheck = 0;
    }
    
  }
   else {
    atRight = false; 
  }
  //At Left Turn
   if (((isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4])) && (isWhite(adc2_buf[4])) && (isWhite(adc1_buf[5])) && (isWhite(adc2_buf[5]))&&(isWhite(adc1_buf[6])))&&!((isWhite(adc1_buf[0]))||(isWhite(adc2_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc2_buf[1])))){
    atLeft = true;
    counterintcheck = counterintcheck + 1;
    if (counterintcheck == 3){
      counterintcheck = 0;
    }
   }
   else { 
    atLeft = false; 
   }

  //____________________TAKE ACTION BASED ON WHEREABOUTS___________________________________
  //Not on the White Line
  if (onWhiteLine == false) {
    mouse_stop();
    delay(1000);

    //Move Forward
    mouse_forward(37,37);
    delay(350);
    mouse_stop();
    delay(1000);
    mouse_right(); 

         /*
              //int speed = 0;
               M1_forward(45);
               M2_backward(45);
               delay(200);
          do {
               M1_forward(45);
               M2_backward(45);
               delay(5);
                mouse_stop();
                t_start = micros();
        for (int i = 0; i < 8; i++) {
          adc1_buf[i] = adc1.readADC(i);
           adc2_buf[i] = adc2.readADC(i);
        }
          t_end = micros();
          //speed = speed + 0.
         } while(!isWhite(adc1_buf[3]) || !isWhite(adc2_buf[2]));*/ 
         
     
  }
  
  //Stop if at an intersection MUST IMPLEMENT HANDLE INTERSECTION
  else if (atIntersection == true) {
    for (int i = 0; i < 10; i++) {
    tone(BUZZ, 3000+(i*200), 100);
    delay(100);
    }    
    //Move Forward
    mouse_forward(37,37);
    delay(350);
    mouse_stop();
    ADCRead(); 
    if(isWhite(adc1_buf[2]) || isWhite(adc1_buf[3]) || isWhite(adc2_buf[3])){ 
      IsForward = true; 
    }
    delay(1000);
    MazeMapping(&MazeX, &MazeY, Direction, atIntersection, atRight, atLeft, atExit, travelledUnitLength, IsForward); 
    IsForward = false; 
    //Turn right
    mouse_right(); 
    
    Serial.print("Turned right at intersection \n");
    //delay(100); 
  }

  //Right Turn
  else if (!atIntersection && atRight == true){ 
      mouse_stop();
    //HANDLE INTERSECTION
    for (int i = 0; i < 10; i++) {
      //tone(BUZZ, 3000+(i*200), 100);
      delay(25);
    }
     
     if (counterintcheck == 0){
      mouse_stop();
      M1_forward(40);
      delay(100);
      mouse_stop();
      delay(1000);
      counterintcheck = counterintcheck + 1;
     } 
     else {
         enc_set = false;
         //Move Forward
         mouse_forward(37,37);
         delay(350);
         mouse_stop();
         ADCRead(); 
         if(isWhite(adc1_buf[2]) || isWhite(adc1_buf[3]) || isWhite(adc2_buf[3])){ 
            IsForward = true; 
         }
         delay(1000);
         MazeMapping(&MazeX, &MazeY, Direction, atIntersection, atRight, atLeft, atExit, travelledUnitLength, IsForward); 
         mouse_right(); 
         IsForward = false; 
          /* 
          //int speed = 0;
               M1_forward(45);
               M2_backward(45);
               delay(200);
          do {
               M1_forward(45);
               M2_backward(45);
               delay(5);
                mouse_stop();
                t_start = micros();
        for (int i = 0; i < 8; i++) {
          adc1_buf[i] = adc1.readADC(i);
           adc2_buf[i] = adc2.readADC(i);
        }
          t_end = micros();
          //speed = speed + 0.
         } while(!isWhite(adc1_buf[3]) || !isWhite(adc2_buf[2]));
           P = 0;
           I = 0;
        for (int i = 0; i < 8; i++) {
           adc1_buf[i] = adc1.readADC(i);
           adc2_buf[i] = adc2.readADC(i);
        }
           error = isVeeringTowards(adc1_buf[0], adc2_buf[0], adc1_buf[1], adc2_buf[1], adc1_buf[2], adc2_buf[2], adc1_buf[3], adc2_buf[3], adc1_buf[4], adc2_buf[4], adc1_buf[5], adc2_buf[5], adc1_buf[6]);
           calculatePID();
           moveCourseCorrect();*/
          
     }
    Serial.print("Turned right \n");
    delay(100);
  }

    // Left Turn
    else if (!atIntersection && atLeft == true){ 
    mouse_stop();
    //HANDLE INTERSECTION
   for ( int i = 0; i < 10; i++) {
      //tone(BUZZ, 3000+(i*200), 100);
      delay(25);
    }
    
   if (counterintcheck == 0){
     mouse_stop();
     M2_forward(40);
     delay(100);
     mouse_stop();
     delay(1000);
     counterintcheck = counterintcheck + 1;
     } else {
         //Move Forward
         mouse_forward(37,37);
         delay(350);
         mouse_stop();
         ADCRead(); 
         if(isWhite(adc1_buf[2]) || isWhite(adc1_buf[3]) || isWhite(adc2_buf[3])){ 
            IsForward = true; 
         }
         delay(1000);
         MazeMapping(&MazeX, &MazeY, Direction, atIntersection, atRight, atLeft, atExit, travelledUnitLength, IsForward); 
         mouse_left(); 
         IsForward = false; 

          //int speed = 0;
          /*
            M2_forward(45);
            M1_backward(45);
            delay(200);
          do {
               M2_forward(45);
               M1_backward(45);
               delay(5);
                mouse_stop();
                t_start = micros();
        for (int i = 0; i < 8; i++) {
           adc1_buf[i] = adc1.readADC(i);
           adc2_buf[i] = adc2.readADC(i);
        }
          t_end = micros();
         } while (!isWhite(adc1_buf[3]) || !isWhite(adc2_buf[3]));
         //Move Forward After
           P = 0;
           I = 0;
        for (int i = 0; i < 8; i++) {
           adc1_buf[i] = adc1.readADC(i);
           adc2_buf[i] = adc2.readADC(i);
        }
           error = isVeeringTowards(adc1_buf[0], adc2_buf[0], adc1_buf[1], adc2_buf[1], adc1_buf[2], adc2_buf[2], adc1_buf[3], adc2_buf[3], adc1_buf[4], adc2_buf[4], adc1_buf[5], adc2_buf[5], adc1_buf[6]);
           calculatePID();
           moveCourseCorrect();
           */
     }
    Serial.print("Turned Left\n");
    delay(100);  
  }
    
 //On the line
  else if (onWhiteLine && !atIntersection && !atRight && !atLeft) {
    error = isVeeringTowards(adc1_buf[0], adc2_buf[0], adc1_buf[1], adc2_buf[1], adc1_buf[2], adc2_buf[2], adc1_buf[3], adc2_buf[3], adc1_buf[4], adc2_buf[4], adc1_buf[5], adc2_buf[5], adc1_buf[6]);
    calculatePID();
    moveCourseCorrect();

    if (enc_set == false) {
      enc1_start = enc1.read();
      enc2_start = enc2.read();
      enc_set = true;
    }
    else if (travelledUnitLength(enc1.read(), enc2.read())) {
      MazeMapping(&MazeX, &MazeY, Direction, atIntersection, atRight, atLeft, atExit, travelledUnitLength, IsForward);
      Serial.println();
      tone(BUZZ, 5400, 100); //play sound to denote MAZE_UNIT_LENGTH travelled
      delay(100);
      Serial.print("Travelled Unit Length...Do something with this fact!");
      Serial.println();
      enc_set = false;
    }
    else if (!travelledUnitLength(enc1.read(), enc2.read())) {
      /*Serial.println();
      Serial.print("not travelled unit length");
      Serial.println();*/
    }
  }
  
  else {
    Serial.print("end of TAKE ACTION BASED ON WHEREABOUTS");
    mouse_stop();
  }   
  
   delay(10);
   M1_stop();
   M2_stop();

   // Serial.print(t_end - t_start);
    /*Serial.print("atIntersection:");
    Serial.print(atIntersection);
    Serial.print("\t");
    Serial.print("onWhiteLine:");
    Serial.print(onWhiteLine);
    Serial.println();*/

 /* else {
    Serial.print("end of TAKE ACTION BASED ON WHEREABOUTS");
    mouse_stop();
  }*/
  /* Serial.print("atIntersection:");
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
    Serial.println();*/
    
}//end void loop
