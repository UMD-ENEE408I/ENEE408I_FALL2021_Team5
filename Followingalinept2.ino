#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = A3;
const unsigned int ADC_2_CS = A2;
const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int PWM_VALUE = 200;
void M2_forward() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 255);
}
void M1_forward() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 220);
}
void M1_stop() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}
void M2_stop() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}

void setup() {
  Serial.begin(115200);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
}

void loop() {
  int adc1_buf[8];
  int adc2_buf[8];

  int t_start = micros();
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
    
    if ( (adc1_buf[i] < 690 && adc1_buf[i] > 100 ) || (adc2_buf[i] < 690 && adc2_buf[i] > 100 )){
         M1_forward();
         M2_forward();
         break;
        Serial.print("Here");
         
    } else {
      M1_stop();
      M2_stop();
    }
    //delay(50);
  }
  int t_end = micros();

  for (int i = 0; i < 8; i++) {
    Serial.print(adc1_buf[i]); Serial.print("\t");
    Serial.print(adc2_buf[i]); Serial.print("\t");
  }

  Serial.print(t_end - t_start);
  Serial.println();

  delay(10);
}
