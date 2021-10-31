/*
 * ble_led_central
 * 
 * Last edited: 10/31/21
 * 
 * This is the program for the central client device 
 * that will read whether or not the LED of the peripheral
 * device is on or off.
 * 
 * Summary:
 * -Successfully connects to peripheral device
 * -Fails to find peripheral device characteristics
 * 
 */

#include <ArduinoBLE.h>

//BLE Declarations
char* uuid1 = "6558e9a0-3cee-4e34-b467-154bd805929a";
char* ServiceCharacteristicuuid1 = "6558e9a1-3cee-4e34-b467-154bd805929a";

//Buzz
const unsigned int BUZZ = 10;

void setup() {
  Serial.begin(115200);


  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Nano 33 BLE (Central)"); 
  BLE.advertise();

  Serial.println("Arduino Nano 33 BLE Sense (Central Device)");
  Serial.println(" ");
}

/*
 * Connects to peripheral device
 */
void connectToPeripheral(char* particular_uuid, char* particular_characteristic_uuid){
  BLEDevice peripheral;
  
  Serial.println("- Discovering peripheral device...");

  do
  {
    int scan_status = BLE.scanForUuid(particular_uuid);
    /*if (scan_status) {
      //play success sound if
      for (int i = 0; i < 10; i++) {
        tone(BUZZ, 3000+(i*200), 100);
        delay(100);
      }
    }
    else {
      //play failure sound
      for (int i = 10; i > 1; i--) {
        tone(BUZZ, 3000+(i*200), 100);
        delay(100);
      }
    }*/
    
    peripheral = BLE.available();
  } while (!peripheral);
  
  if (peripheral) {
    Serial.println("* Peripheral device found!");
    Serial.print("* Device MAC address: ");
    Serial.println(peripheral.address());
    Serial.print("* Device name: ");
    Serial.println(peripheral.localName());
    Serial.print("* Advertised service UUID: ");
    Serial.println(peripheral.advertisedServiceUuid());
    Serial.println(" ");
    BLE.stopScan();
    controlPeripheral(peripheral, particular_characteristic_uuid);
  }
}

/*
 * Controls the connected peripheral
 */
void controlPeripheral(BLEDevice peripheral, char* particular_characteristic_uuid) {
  Serial.println("- Connecting to peripheral device...");

  if (peripheral.connect()) {
    Serial.println("* Connected to peripheral device!");
    Serial.println(" ");
  } else {
    Serial.println("* Connection to peripheral device failed!");
    Serial.println(" ");
    return;
  }

  Serial.println("- Discovering peripheral device attributes...");
  if (peripheral.discoverAttributes()) {
    Serial.println("* Peripheral device attributes discovered!");
    Serial.println(" ");
  } else {
    Serial.println("* Peripheral device attributes discovery failed!");
    Serial.println(" ");
    peripheral.disconnect();
    return;
  }

  BLECharacteristic LEDcharacteristic = peripheral.characteristic(particular_characteristic_uuid);

  if (!LEDcharacteristic) {
    Serial.println("* Peripheral device does not have LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    if (LEDcharacteristic.valueUpdated()) {
      //bool onLED = false;
      //LEDcharacteristic.readValue(onLED); //read value and place in onLED variable above
      byte onLED = 0;
      LEDcharacteristic.readValue(onLED);

      if (onLED) {
        Serial.println("LED on");
      }
      else {
        Serial.println("LED off");
      }
    }   
  }
}

void loop() {
  connectToPeripheral(uuid1, ServiceCharacteristicuuid1); //BLE test
}
