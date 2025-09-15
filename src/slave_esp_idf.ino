/*****************************************************************************************
*     Serial Bluetooth communication between 2 ESP32 boards - OK1TK www.ok1tk.com        *
*                   The CLIENT (SLAVE) part code v1.21 - 01 May 2023                     *
******************************************************************************************/
#include "BluetoothSerial.h"                                                  // BT: Include the Serial bluetooth library 
#define LED_BT 2                                                              // BT: Internal LED (or LED on the pin D2) for the connection indication (connected solid/disconnected blinking)
unsigned long previousMillis;                                                 // BT: Variable used for comparing millis counter (LED blinking)
bool ledBtState = false;                                                      // BT: Variable used to chage the indication LED state
bool MasterConnected;                                                         // BT: Variable used to store the current connection state (true=connected/false=disconnected)
String device_name = "ESP32-BT-Slave";                                        // BT: Variable used to store the CLIENT(slave) bluetooth device name
String MACadd = "03:B4:16:72:36:9C";                                          // BT: Variable used to store the CLIENT(slave) bluetooth Mac address; Use your own MAC address
uint8_t receivedData[8];     // incoming 8-byte payload
uint32_t checksumSlave;      // calculated CRC32
uint8_t checksumBuffer[4];   // send buffer (4 bytes)

// BT: Bluetooth availabilty check
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)                           
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
// BT: Serial Bluetooth availabilty check
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;                                                     // BT: Set the Object SerialBT

//Checksum function
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    crc = crc ^ byte;
    for (uint8_t j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}

//Calculate and send checksum
void returnCRC32() {
  // STEP 1: Wait until 8 bytes are available
  while (SerialBT.available() < 8) {
    delay(1);
  }

  // STEP 2: Read into buffer
  SerialBT.readBytes(receivedData, 8);

  // Debug: print received data
  Serial.print("Slave received data: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("%02X ", receivedData[i]);
  }
  Serial.println();

  // STEP 3: Calculate checksum
  checksumSlave = calculateCRC32(receivedData, 8);
  Serial.printf("Slave calculated checksum: %08X\n", checksumSlave);

  // STEP 4: Pack checksum into 4 bytes (big-endian)
  checksumBuffer[0] = (checksumSlave >> 24) & 0xFF;
  checksumBuffer[1] = (checksumSlave >> 16) & 0xFF;
  checksumBuffer[2] = (checksumSlave >> 8) & 0xFF;
  checksumBuffer[3] = checksumSlave & 0xFF;

  // STEP 5: Send 4 bytes back
  SerialBT.write(checksumBuffer, 4);
  Serial.println("Slave sent checksum back to master...");

  delay(1000);
}

// BT: Bt_Status callback function
void Bt_Status (esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {

  if (event == ESP_SPP_SRV_OPEN_EVT) {                                        // BT: Checks if the SPP Server connection is open, the event comes 
    Serial.println ("Client Connected");                                      // BT: Write to the serial monitor
    digitalWrite (LED_BT, HIGH);                                              // BT: Turn ON the bluetooth indication LED (solid light)
    MasterConnected = true;                                                   // BT: Set the variable true = SERVER is connected to the CLIENT
  }
  else if (event == ESP_SPP_CLOSE_EVT ) {                                     // BT: Checks if the SPP connection is closed, the event comes
    Serial.println ("Client Disconnected");                                   // BT: Write to the serial monitor
    digitalWrite (LED_BT, LOW);                                               // BT: Turn OFF the bluetooth indication LED
    MasterConnected = false;                                                  // BT: Set the variable false = SERVER connection lost
  }
}

void setup() {
  pinMode(LED_BT,OUTPUT);                                                     // BT: Set up the onboard LED pin as output
  Serial.begin(115200);                                                       // Sets the data rate in bits per second (baud) for serial data transmission
  
  SerialBT.register_callback (Bt_Status);                                     // BT: Define the Bt_Status callback
  SerialBT.begin(device_name);                                                // BT: Starts the bluetooth device with the name stored in the device_name variable
  Serial.printf("The device with name \"%s\" and MAC address \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), MACadd.c_str());
}

void loop() {
  // BT: Blinking the bluetooth indication LED if the connection is not established
  if (!MasterConnected) {
    if (millis() - previousMillis >= 500) {                                     // BT: Checks if 500ms is passed
      if (ledBtState == false) {                                                // BT: Checks the leddState and toggles it 
      ledBtState = true;
    }
    else {
      ledBtState = false;
    }
    digitalWrite(LED_BT, ledBtState);                                             // BT: Set LED ON/OFF based onthe ledBtState variable
    previousMillis = millis();                                                    // BT: Set previousMillis to current millis
    }
  }

  if (MasterConnected) {
    returnCRC32();
  }



  // BT: Data send/receive via bluetooth
  if (Serial.available()) {                                                       // BT: Checks if there are data from the serial monitor available
    SerialBT.write(Serial.read());                                                // BT: Sends the data via bluetooth
  }
  if (SerialBT.available()) {                                                     // BT: Checks if there are data from the bluetooth available
    Serial.write(SerialBT.read());                                                // BT: Write the data to the serial monitor
  }
}