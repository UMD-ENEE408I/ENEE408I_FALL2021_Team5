/*
 * ble_led_peripheral
 * 
 * Last edited: 10/31/21
 * 
 * This is the code for the peripheral device that makes
 * it turn on and off its LED and update a characteristic
 * to show that the LED is on or off. This characteristic
 * will be read by the central client device.
 * 
 * Summary:
 * -Successfully connects to the central device
 * -Successfully flashes LED on and off
 * -Fails to communicate its characteristics to the
 *  central device.
 */

#include <ArduinoBLE.h>

//BLE Declarations
char* uuid1 = "6558e9a0-3cee-4e34-b467-154bd805929a";
char* ServiceCharacteristicuuid1 = "6558e9a1-3cee-4e34-b467-154bd805929a";
BLEService ScoutService(uuid1); //new BLE service that contains characteristics
//BLEBoolCharacteristic onWhiteLine(uuid1, BLERead | BLENotify); //client can read value and will be notified of changes
BLEByteCharacteristic onLED(ServiceCharacteristicuuid1, BLERead | BLENotify); //client can read value and will be notified of changes
//scout1.addCharacteristic(onLED);

/*
 * Turns on built-in LED for 2 seconds, informs that the LED is on via BLE
 * Then turns off the LED, informing that LED is off via BLE
 */
void LEDblink() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  onLED.writeValue(1);
  delay(1000);                       // wait for 2 seconds
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  onLED.writeValue(0);
  delay(1000);  
}

/*
 * performs LEDblink() a specified input amount of times.
 */
void testBLE(int times) {
  for (int i = 1; i < times; i++) {
    LEDblink();
  }
}

void setup() {
  Serial.begin(115200);

  // Set the connection interval to be as fast as possible (about 40 Hz)
  BLE.setConnectionInterval(0x0006, 0x0006); 

  //while (!Serial); //does not begin until Serial monitor is opened  

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  pinMode(LED_BUILTIN, OUTPUT); //for BLE blinker test

  //BLE.setLocalName("Arduino Nano 33 BLE (Peripheral)");
  BLE.setLocalName("LED");
  BLE.setAdvertisedService(ScoutService);
  ScoutService.addCharacteristic(onLED);
  BLE.addService(ScoutService);
  onLED.writeValue(0);
  BLE.advertise();

  Serial.println("Nano 33 BLE (Peripheral Device)");
  Serial.println(" ");
}

void connectToCentral() {
  BLEDevice central = BLE.central();
  Serial.println("- Discovering central device...");
  delay(500);

  if (central) {
    Serial.println("* Connected to central device!");
    Serial.print("* Device MAC address: ");
    Serial.println(central.address());
    Serial.println(" ");

    while (central.connected()) {
      /*if (onLED.written()) {
         byte val = onLED.value();
         writeGesture(gesture);
       }*/
       testBLE(40);
    }
    
    Serial.println("* Disconnected to central device!");
  }
}

void loop() {
  connectToCentral();
  
}
