#include <Adafruit_MCP3008.h>
#include <Encoder.h>
#include <ArduinoBLE.h>

#define DEBUG false //set this to true to only begin when connected to Serial monitor
#define MAP_DEBUG false //set this to true to Serial.print map data
#define ADC_BUFF_DEBUG false //set this to true to Serial print ADCbuff values
#define ENABLE_MOVEMENT true //set this to true for the mouse to be able to move

//_________________________________MOUSE MODE____________________________________
/*
 * The mouse mode determines whether it will report maze data or receive
 * instructions on where to travel. Only ONE mode must be true at a time.
 */
 bool isScout;
 bool isFollowing;

 bool ble_init; //state for whether or not ble is connected to Jetson

// These UUIDs have been randomly generated. - they must match between the Central and Peripheral devices
// Any changes you make here must be suitably made in the Python program as well
BLEService nanoService("13012F00-F8C3-4F4A-A8F4-15CD926DA146"); // BLE Service

// BLE Characteristics - custom 128-bit UUID, read and writable by central device
BLEByteCharacteristic instrCharacteristic("13012F01-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic mouseDataCharacteristic("13012F02-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite);
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
const int FULL_UNIT_LENGTH = 270; 
const int LENGTH_AFTER_INT = 80; 
int MAZE_UNIT_LENGTH = FULL_UNIT_LENGTH; //was 1080
const int ENCODER_ERROR_BOUND = 60;
///////////////////////////////////////////////

//ADC Declarations
int adc1_buf[8];
int adc2_buf[8];
////////////////////////////////////////////
  
//Maze Mapping Declarations 
char MazeMap[100][100][8];
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

int MazeX = 0;  //Initial X Position for the MazeMap is the Center
int MazeY = 49;  //Initial Y Position for the MazeMap is the Center
char Direction = 'N';    //Initial Direction will always be assumed to be North
//////////////////////////////////////////////////////////////////////////////////////

//Forward
const unsigned int FULL_SPEED_FORWARD = 35;

 //Buzz
 const unsigned int BUZZ = 10;

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

//const unsigned int BLACK = 650;
const int BLACK = 670; //Changed to 685 from 680 for Testing. 

//Booleans 
bool atRight = false;
bool atLeft = false;
bool atIntersection = false;
bool onWhiteLine = false;
bool atExit = false;
bool IsForward = false; 
bool atDeadEnd = false;

//bool intersectioncheck = false;
int counterintcheck = 0;
int commands[] = {1,3,2,3,3,2,2,3,2};
int counterforcommand = 0;

//int endcheck
int endchecks = 0;

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
  mouse_forward(FULL_SPEED_FORWARD + PIDvalue, (FULL_SPEED_FORWARD)- PIDvalue);
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
  Serial.begin(9600);

  if (DEBUG) {
    while (!Serial);
  }

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("FOLLOWER");
  BLE.setAdvertisedService(nanoService);

  // add the characteristic to the service
  nanoService.addCharacteristic(instrCharacteristic);
  nanoService.addCharacteristic(mouseDataCharacteristic);

  // add service
  BLE.addService(nanoService);

  // set the initial value for the characeristic:
  instrCharacteristic.writeValue(0);
  mouseDataCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();
  delay(100);
  Serial.println("Arduino Nano BLE LED Peripheral Service Started");
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  

  //Motor
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);

  int m;
  int l; 
  for (m = 0; m<50; m++){
    for (l = 0; l<50;l++){
      MazeMap[m][l][0] = 'X'; 
      MazeMap[m][l][1] = 'X';
      MazeMap[m][l][2] = 'X'; 
      MazeMap[m][l][3] = 'X'; 
      MazeMap[m][l][4] = 'X'; 
      MazeMap[m][l][5] = 'X'; 
      MazeMap[m][l][6] = 'U'; //All Nodes are Unvisited to Start
      MazeMap[m][l][7] = 'X';   
    }
  }
  
  MazeMap[MazeX][MazeY][0] = 'T';   //Mark the Node for the Start position location
  MazeMap[MazeX][MazeY][6] = 'V';   //Mark the Start Position to Visited
}

void MazeMapping(int* MazeX, int* MazeY, char Direction, bool atIntersection, bool atRight, bool atLeft, bool atExit, bool travelledUnitLength, bool IsForward, bool atDeadEnd){
  /*  Meaning of Third array parameter (May need Adjustment Eventually)
   * [0]: Map of Nodes (True for there is a Node in this X,Y position 
   * [1]: Is there is a Node Forward from original Direction entered (T or F)
   * [2]: Is there a Node to the left of the original direction entered (T or F)
   * [3]: Is there a Node to the right of the orginal direction entered (T or F)
   * [4]: Is there a Node behind the original direction entered (T or F) 
   * [5]: What was the direction you entered the Node at (N/S/E/W)
   * [6]: Is the Node Visited or Unvisited (V or U) 
   * [7]: Is the Mouse at the Exit
   */
  
  // Adjust X and Y Coordinates to the new posiotion
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
     MazeMap[*MazeX][*MazeY][6] = 'V'; //Set the Node to Visited
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

  else if(atDeadEnd == true){
    MazeMap[*MazeX][*MazeY][1] = 'F';   //Add Forward Detection
      MazeMap[*MazeX][*MazeY][2] = 'F';   //There is no node to the Left 
      MazeMap[*MazeX][*MazeY][3] = 'F';   //There is no node to the right
      MazeMap[*MazeX][*MazeY][4] = 'T';   //There is a Node Behind
      MazeMap[*MazeX][*MazeY][5] = Direction;   //Direction entered is equal to the Direction variable 
  }
  
  if(atExit == true){
      MazeMap[*MazeX][*MazeY][5] = Direction;   //Direction entered is equal to the Direction variable
      MazeMap[*MazeX][*MazeY][7] = 'T';          //The Mouse has reached the Exit this is the end node
  }
     //Serial.print("My Postion: "); Serial.print("X = "); Serial.print(*MazeX); Serial.print(" Y = "); Serial.print(*MazeY); Serial.print("\n");
     //Serial.print("My Direction: "); Serial.print(Direction); Serial.print("\n \n");

   if (MAP_DEBUG) {
        // MAZE MAP of NODES
         for(int i = 0; i<50; i++){
          for(int j = 0; j<50; j++){
            Serial.print("Node-");
            Serial.print(j);
            Serial.print(i);
            Serial.print(": ");
            Serial.print(MazeMap[j][i][0]);
            Serial.print("\t"); 
          }
          Serial.print("\n");
        }
        Serial.print("\n");
        Serial.print("\n");
        
        /*
        //MAZE MAP OF Visited vs UNVISITED NODES
        for(int i = 0; i<11; i++){
          for(int j = 0; j<11; j++){
            Serial.print("Node-");
            Serial.print(j);
            Serial.print(i);
            Serial.print(": ");
            Serial.print(MazeMap[j][i][6]);
            Serial.print("\t"); 
          }
          Serial.print("\n");
        }
        */
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
void mouse_deadend(){
   for(int i = 0; i<3;i++){
     M2_forward(80);
     M1_backward(85);
     delay(40);
     mouse_stop();
     delay(200);
   }
   do {
     M2_forward(80);
     M1_backward(80);
     delay(40);
     mouse_stop();
     delay(200);
     ADCRead(); 
     
      } while (!isWhite(adc1_buf[3])); //while (!isWhite(adc1_buf[3]) || !isWhite(adc2_buf[3]));
     //Reset PID Controller
     P = 0;
     I = 0;
     switch (Direction){ 
        case 'N':
          Direction = 'S';
          break; 
        case 'S': 
          Direction = 'N'; 
          break; 
        case 'E':
          Direction = 'W';
          break; 
        case 'W': 
          Direction = 'E';
          break; 
      }  
}
void mouse_left() {
   for(int i = 0; i<3;i++){
     M2_forward(80);
     M1_backward(80);
     delay(40);
     mouse_stop();
     delay(200);
   }
   do {
     M2_forward(80);
     M1_backward(80);
     delay(40);
     mouse_stop();
     delay(200);
     ADCRead(); 
//     if((isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3])) && (isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4]))) { 
//             mouse_stop();
//             for (int i = 0; i < 10; i++) {
//                tone(BUZZ, 3000+(i*200), 100);
//                delay(150);
//             }    
//            mouse_stop();
//            delay(5000);
//         }
      } while (!isWhite(adc1_buf[3])); //while (!isWhite(adc1_buf[3]) || !isWhite(adc2_buf[3]));
     //Reset PID Controller
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
  for(int i = 0; i<3;i++){
     M1_forward(80);
     M2_backward(80);
     delay(40);
     mouse_stop();
     delay(200);
  }
  do {
    M1_forward(80);
    M2_backward(80);
    delay(40);
    mouse_stop();
    delay(200);
    ADCRead(); 
//    if((isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3])) && (isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4]))) { 
//             mouse_stop();
//             for (int i = 0; i < 10; i++) {
//                tone(BUZZ, 3000+(i*200), 100);
//                delay(150);
//             }    
//            mouse_stop();
//            delay(5000);
//         }
  } while (!isWhite(adc1_buf[3])); //while (!isWhite(adc1_buf[3]) || !isWhite(adc2_buf[3]));
 //Reset PID Controller
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

// Navigation Deciosion Making------------------------------------------------------------------------------------
//Returns True if the mouse has visited the Node to the Right and False if it hasn't
boolean Check_Visited_Right(int* MazeX, int* MazeY){ 
  switch (Direction){
    case 'N':
      if(MazeMap[(*MazeX)+1][*MazeY][6] == 'V'){
        return true;
      }
      else{
        return false;
      }
      break; 
    case 'S': 
      if(MazeMap[(*MazeX)-1][*MazeY][6] == 'V'){
        return true;
      }
      else{
        return false; 
      }
      break; 
    case 'E':
       if(MazeMap[(*MazeX)][*MazeY+1][6] == 'V'){
        return true;
       }
       else{
        return false;
       }
      break; 
    case 'W': 
      if(MazeMap[(*MazeX)][*MazeY-1][6] == 'V'){
        return true;
      }
        else{
          return false; 
      }
      break;  
  }
}
//Returns True if the mouse has already visited the Node Forward and False if It hasn't visited that Node Yet
boolean Check_Visited_Forward(int* MazeX, int* MazeY){
  switch (Direction){
    case 'N':
      if(MazeMap[(*MazeX)][*MazeY-1][6] == 'V'){
        return true;
      }
      else{
        return false;
      }
      break; 
    case 'S': 
      if(MazeMap[(*MazeX)][*MazeY+1][6] == 'V'){
        return true;
      }
      else{
        return false; 
      }
      break; 
    case 'E':
       if(MazeMap[(*MazeX)+1][*MazeY][6] == 'V'){
        return true;
       }
       else{
        return false;
       }
      break; 
    case 'W': 
      if(MazeMap[(*MazeX)-1][*MazeY][6] == 'V'){
        return true;
      }
        else{
          return false; 
      }
      break;  
  }
}
//Returns True if the Mouse Has Visited the Node to the Left already and False if the mouse hasn't
boolean Check_Visited_Left(int* MazeX, int* MazeY){ 
  switch (Direction){
    case 'N':
      if(MazeMap[(*MazeX)-1][*MazeY][6] == 'V'){
        return true;
      }
      else{
        return false;
      }
      break; 
    case 'S': 
      if(MazeMap[(*MazeX)+1][*MazeY][6] == 'V'){
        return true;
      }
      else{
        return false; 
      }
      break; 
    case 'E':
       if(MazeMap[(*MazeX)][*MazeY-1][6] == 'V'){
        return true;
       }
       else{
        return false;
       }
      break; 
    case 'W': 
      if(MazeMap[(*MazeX)][*MazeY+1][6] == 'V'){
        return true;
      }
        else{
          return false; 
      }
      break;  
  }
}

void Navigate_Right_Priority(int MazeX, int MazeY, bool atIntersection,bool atRight,bool atLeft,bool IsForward){ 

  //No Forward Node detected
  if(IsForward == false){
    
    //At T Interseciotn
    if(atIntersection == true){
      if((Check_Visited_Right(&MazeX, &MazeY)) == false){
        mouse_right(); //Go Right if the mouse has not visited the node to the right
      }
      else if((Check_Visited_Right(&MazeX, &MazeY)) == true){
        if((Check_Visited_Left(&MazeX, &MazeY)) == false){
          mouse_left();
        }
        else{
          mouse_right(); 
        }
      }
    }
    
    //At Right Turn 
    else if(!atIntersection && atRight == true){ 
      mouse_right(); //Go Right (Only Option)
    }
    
    //At Left Turn
    else if(!atIntersection && atLeft == true){ 
      mouse_left(); //Go Left (Only Option)
    }
  }

  //There is a Forward Node Detected
  else if(IsForward == true){ 
    //At 4-way Interseciotn
    if(atIntersection == true){
      if((Check_Visited_Right(&MazeX, &MazeY)) == false){
        mouse_right(); //Go Right if the mouse has not visited the node to the right
      }
      else if((Check_Visited_Right(&MazeX, &MazeY)) == true){
        if((Check_Visited_Forward(&MazeX, &MazeY)) == false){ //If right node has already been visited but forward node has not go forward
         
        }
        else if((Check_Visited_Forward(&MazeX, &MazeY)) == true){
          if((Check_Visited_Left(&MazeX, &MazeY)) == false){
            mouse_left(); //Go Left if both the right Node and Forward Node have already been visited
          }
          else{
            mouse_right(); //If all Nodes have been visited already go right
          }
        }
      }
    }

    //At Right Turn With Forward Node
    else if(!atIntersection && atRight == true){ 
      if((Check_Visited_Right(&MazeX, &MazeY)) == false){
        mouse_right(); //Go Right if the mouse has not visited the node to the right
      }
      else if((Check_Visited_Right(&MazeX, &MazeY)) == true){
        if((Check_Visited_Forward(&MazeX, &MazeY)) == false){
         //Go Forward if forward node is unvisited and right node has been visited
        }
        else{
          mouse_right(); //Go right if both nodes have been visited
        }
      }
    }

    //At Left Turn with Forward Node
    else if(!atIntersection && atLeft == true){ 
      if((Check_Visited_Forward(&MazeX, &MazeY)) == false){
         //Go Forward if the mouse has not visited the forward node
      }
      else if((Check_Visited_Forward(&MazeX, &MazeY)) == true){
        if((Check_Visited_Left(&MazeX, &MazeY)) == false){
          mouse_left(); //Go Left if left node is unvisited and forward node has been visited
        }
        else{
           //Go Forward if both nodes have been visited
        }
      }
    }
  }
}




void loop() {
  // listen for BLE centrals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());


    // while the central is still connected to peripheral:
    while (central.connected()) {

      int t_start = micros();
      /*  for (int i = 0; i < 8; i++) {
          adc1_buf[i] = adc1.readADC(i);
          adc2_buf[i] = adc2.readADC(i);
        }*/
      ADCRead();
      int t_end = micros();

      if (ADC_BUFF_DEBUG) {
        for (int i = 0; i < 8; i++) {
          Serial.print(adc1_buf[i]); Serial.print("\t");
          Serial.print(adc2_buf[i]); Serial.print("\t");

          Serial.print("adc1-"); Serial.print(i); Serial.print("-"); Serial.print(adc1_buf[i]); Serial.print("\t");
          Serial.print("adc2-"); Serial.print(i); Serial.print("-"); Serial.print(adc2_buf[i]); Serial.print("\t");
        }
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
      if (((isWhite(adc2_buf[0])) && (isWhite(adc1_buf[1])) && (isWhite(adc2_buf[1])) && (isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3]))) && !(((isWhite(adc2_buf[4])) || (isWhite(adc1_buf[5])) || (isWhite(adc2_buf[5])) || (isWhite(adc1_buf[6]))))) {
        atRight = true;
        /*counterintcheck = counterintcheck + 1;
          if (counterintcheck == 3){
          counterintcheck = 0;
          }*/

      }
      else {
        atRight = false;
      }
      //At Left Turn
      if (((isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4])) && (isWhite(adc2_buf[4])) && (isWhite(adc1_buf[5])) && (isWhite(adc2_buf[5])) && (isWhite(adc1_buf[6]))) && !((isWhite(adc1_buf[0])) || (isWhite(adc2_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc2_buf[1])))) {
        atLeft = true;
        /*counterintcheck = counterintcheck + 1;
          if (counterintcheck == 3){
          counterintcheck = 0;
          }*/
      }
      else {
        atLeft = false;
      }

      //____________________TAKE ACTION BASED ON WHEREABOUTS___________________________________
      //Not on the White Line (Dead End)


      //Stop if at an intersection MUST IMPLEMENT HANDLE INTERSECTION
      if (atIntersection == true) {
        if (ENABLE_MOVEMENT) {
          mouseDataCharacteristic.writeValue(0x80000000);
          delay(1000);

          for (int i = 0; i < 10; i++) {
            tone(BUZZ, 3000 + (i * 200), 100);
            delay(100);
          }
          //Move Forward
          enc_set = false;
          MAZE_UNIT_LENGTH = LENGTH_AFTER_INT;
          for (int i = 0; i < 30; i++) {
            mouse_forward(40, 40);
            delay(10);
            ADCRead();
            //Checks if intersection is detected through end corss
            if ((isWhite(adc2_buf[0])) && (isWhite(adc1_buf[1])) && (isWhite(adc2_buf[1])) && (isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3])) && (isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4])) && (isWhite(adc2_buf[4])) && (isWhite(adc1_buf[5])) && (isWhite(adc2_buf[5])) && (isWhite(adc1_buf[6]))) {
              atIntersection = true;
              endchecks = endchecks + 1;
            }
            if (endchecks >= 20) {
              mouse_stop();
              for (int i = 0; i < 10; i++) {
                tone(BUZZ, 3000 + (i * 200), 100);
                delay(150);
              }
              mouse_stop();
              delay(5000);
            }
          }
          mouse_stop();
          endchecks = 0;


          ADCRead();
          //If Still on White Line there is a Forward
          if ((isWhite(adc2_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc2_buf[1])) || (isWhite(adc1_buf[2])) || (isWhite(adc2_buf[2])) || (isWhite(adc1_buf[3])) || (isWhite(adc2_buf[3])) || (isWhite(adc1_buf[4])) || (isWhite(adc2_buf[4])) || (isWhite(adc1_buf[5])) || (isWhite(adc2_buf[5])) || (isWhite(adc1_buf[6]))) {
            IsForward = true;
          }
          delay(1000);
          MazeMapping(&MazeX, &MazeY, Direction, atIntersection, atRight, atLeft, atExit, travelledUnitLength, IsForward, atDeadEnd);
          //Do Navigation

          if (instrCharacteristic.written()) {
            uint8_t CURRENT_STATE = 0b1111 & instrCharacteristic.value();;
            //printBinary(instrCharacteristic.value());
            Serial.println(CURRENT_STATE);
            /*
              ADD STATES HERE
            */

            if (CURRENT_STATE == 1) {
              mouse_forward(40, 40);
              delay(20);
            } else if (CURRENT_STATE == 2) {
              mouse_left();
            } else if (CURRENT_STATE == 3) {
              mouse_right();
            }
          }
          
          mouse_stop();
          //mouseDataCharacteristic.writeValue(0);


          IsForward = false;
          Serial.print("Turned right at intersection \n");
          //delay(100);
        }
      }

      //Right Turn
      else if (!atIntersection && atRight == true) {
        if (ENABLE_MOVEMENT) {
          mouseDataCharacteristic.writeValue(0x80000000);
          delay(1000);

          mouse_stop();
          //HANDLE INTERSECTION
          for (int i = 0; i < 10; i++) {
            //tone(BUZZ, 3000+(i*200), 100);
            delay(25);
          }

          if (counterintcheck == 0) {
            mouse_stop();
            /* M1_forward(45);
              delay(100);
              mouse_stop();
              delay(1000);*/
            counterintcheck = counterintcheck + 1;
          }
          else {
            enc_set = false;
            MAZE_UNIT_LENGTH = LENGTH_AFTER_INT;
            //Move Forward and Check For Intersection By looking for right turn
            int i = 0;
            for (i = 0; i < 25; i++) {
              mouse_forward(40, 40);
              delay(10);
              ADCRead();
              //If Left Turn Detected actually at T-Intersection
              if (((isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4])) && (isWhite(adc2_buf[4])) && (isWhite(adc1_buf[5])) && (isWhite(adc2_buf[5])) && (isWhite(adc1_buf[6]))) && !((isWhite(adc1_buf[0])) || (isWhite(adc2_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc2_buf[1])))) {
                atIntersection = true;
              }

            }
            mouse_stop();
            ADCRead();
            //If still on White Line there is a Forward
            if ((isWhite(adc2_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc2_buf[1])) || (isWhite(adc1_buf[2])) || (isWhite(adc2_buf[2])) || (isWhite(adc1_buf[3])) || (isWhite(adc2_buf[3])) || (isWhite(adc1_buf[4])) || (isWhite(adc2_buf[4])) || (isWhite(adc1_buf[5])) || (isWhite(adc2_buf[5])) || (isWhite(adc1_buf[6]))) {
              IsForward = true;
            }
            //Checks if intersection is detected through end corss
            if ((isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3])) && (isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4]))) {
              mouse_stop();
              for (int i = 0; i < 10; i++) {
                tone(BUZZ, 3000 + (i * 200), 100);
                delay(150);
              }
              mouse_stop();
              delay(5000);
            }
            delay(1000);
            MazeMapping(&MazeX, &MazeY, Direction, atIntersection, atRight, atLeft, atExit, travelledUnitLength, IsForward, atDeadEnd);
            //Do Navigation
            if (instrCharacteristic.written()) {
              uint8_t CURRENT_STATE = 0b1111 & instrCharacteristic.value();;
              //printBinary(instrCharacteristic.value());
              Serial.println(CURRENT_STATE);
              /*
                ADD STATES HERE
              */

              if (CURRENT_STATE == 1) {
                mouse_forward(40, 40);
                delay(20);
              } else if (CURRENT_STATE == 2) {
                mouse_left();
              } else if (CURRENT_STATE == 3) {
                mouse_right();
              }
            }
            mouse_stop();
            IsForward = false;
            counterintcheck = 0;
           // mouseDataCharacteristic.writeValue(0);

          }
          Serial.print("Turned right \n");
          delay(100);
        }
      }

      // Left Turn
      else if (!atIntersection && atLeft == true) {
        if (ENABLE_MOVEMENT) {
           mouseDataCharacteristic.writeValue(0x80000000);
           delay(1000);

          mouse_stop();
          //HANDLE INTERSECTION
          for ( int i = 0; i < 10; i++) {
            //tone(BUZZ, 3000+(i*200), 100);
            delay(25);
          }

          if (counterintcheck == 0) {
            mouse_stop();
            /*M2_forward(40);
              delay(100);
              mouse_stop();
              delay(1000);*/
            counterintcheck = counterintcheck + 1;
          } else {
            enc_set = false;
            MAZE_UNIT_LENGTH = LENGTH_AFTER_INT;
            //Move Forward and Check For Intersection By looking for right turn
            int i = 0;
            for (i = 0; i < 25; i++) {
              mouse_forward(40, 40);
              delay(10);
              ADCRead();
              //If Right Turn Detected at intersection
              if (((isWhite(adc2_buf[0])) && (isWhite(adc1_buf[1])) && (isWhite(adc2_buf[1])) && (isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3]))) && !(((isWhite(adc2_buf[4])) || (isWhite(adc1_buf[5])) || (isWhite(adc2_buf[5])) || (isWhite(adc1_buf[6]))))) {
                atIntersection = true;
              }

            }

            mouse_stop();
            ADCRead();
            //If Still on White Line there is a Forward
            if ((isWhite(adc2_buf[0])) || (isWhite(adc1_buf[1])) || (isWhite(adc2_buf[1])) || (isWhite(adc1_buf[2])) || (isWhite(adc2_buf[2])) || (isWhite(adc1_buf[3])) || (isWhite(adc2_buf[3])) || (isWhite(adc1_buf[4])) || (isWhite(adc2_buf[4])) || (isWhite(adc1_buf[5])) || (isWhite(adc2_buf[5])) || (isWhite(adc1_buf[6]))) {
              IsForward = true;
            }
            //Checks if intersection is detected through end corss
            if ((isWhite(adc1_buf[2])) && (isWhite(adc2_buf[2])) && (isWhite(adc1_buf[3])) && (isWhite(adc2_buf[3])) && (isWhite(adc1_buf[4]))) {
              mouse_stop();
              for (int i = 0; i < 10; i++) {
                tone(BUZZ, 3000 + (i * 200), 100);
                delay(150);
              }
              mouse_stop();
              delay(5000);
            }
            delay(1000);
            MazeMapping(&MazeX, &MazeY, Direction, atIntersection, atRight, atLeft, atExit, travelledUnitLength, IsForward, atDeadEnd);
            //Do Navigation
            if (instrCharacteristic.written()) {
              uint8_t CURRENT_STATE = 0b1111 & instrCharacteristic.value();;
              //printBinary(instrCharacteristic.value());
              Serial.println(CURRENT_STATE);
              /*
                ADD STATES HERE
              */

              if (CURRENT_STATE == 1) {
                mouse_forward(40, 40);
                delay(20);
              } else if (CURRENT_STATE == 2) {
                mouse_left();
              } else if (CURRENT_STATE == 3) {
                mouse_right();
              }
            }
            IsForward = false;
            counterintcheck = 0;
           // mouseDataCharacteristic.writeValue(0);

          }
          Serial.print("Turned Left\n");
          delay(100);
        }
      }

      //On the line
      else if (onWhiteLine && !atIntersection && !atRight && !atLeft) {
        if (ENABLE_MOVEMENT) {

          error = isVeeringTowards(adc1_buf[0], adc2_buf[0], adc1_buf[1], adc2_buf[1], adc1_buf[2], adc2_buf[2], adc1_buf[3], adc2_buf[3], adc1_buf[4], adc2_buf[4], adc1_buf[5], adc2_buf[5], adc1_buf[6]);
          calculatePID();
          moveCourseCorrect();

          if (enc_set == false) {
            enc1_start = enc1.read();
            enc2_start = enc2.read();
            enc_set = true;
          }
          else if (travelledUnitLength(enc1.read(), enc2.read())) {
            MazeMapping(&MazeX, &MazeY, Direction, atIntersection, atRight, atLeft, atExit, travelledUnitLength, IsForward, atDeadEnd);
            MAZE_UNIT_LENGTH = FULL_UNIT_LENGTH;
            Serial.println();
            mouse_stop();
            delay(1000);
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
      }



      else {
        Serial.print("end of TAKE ACTION BASED ON WHEREABOUTS");
        mouse_stop();
      }

      delay(10);
      M1_stop();
      M2_stop();
      counterforcommand = counterforcommand + 1;






      // if the remote device wrote to the characteristic,

      mouseDataCharacteristic.writeValue(0);
     
    }


    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }

}


template<typename T>        //prints out all the binary bits of a variable
void printBinary(T value) {
  int num_bits = sizeof(value) * 8;
  for (int i = 0; i < num_bits; i++) {
    Serial.print((value & (1 << num_bits - i - 1)) ? "1" : "0");
  } Serial.println();
}
