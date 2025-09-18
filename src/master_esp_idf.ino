/*****************************************************************************************
*     Serial Bluetooth communication between 2 ESP32 boards                              *
*     The SERVER (Master) part code v1.0 - 18 September 2025                             *
******************************************************************************************/
#include "BluetoothSerial.h"                                                              // BT: Include the Serial bluetooth library
#define LED_BT_BLUE 2                                                                     // BT: Internal LED (or LED on the pin D2) for the connection indication (connected LED ON / disconnected LED OFF)
#define LED_BT_RED 15                                                                     // BT: LED (LED on the pin D4) for the connection indication (connected LED OFF / disconnected LED ON)
unsigned long previousMillisReconnect;                                                    // BT: Variable used for comparing millis counter for the reconnection timer
unsigned long previousMillis;
bool ledBtState = false;                                                                  // BT: Variable used to chage the indication LED state
bool SlaveConnected;                                                                      // BT: Variable used to store the current connection state (true=connected/false=disconnected)
int recatt = 0;                                                                           // BT: Variable used to count the reconnection attempts
uint8_t randomData[8];       // data to send
uint32_t checksumMaster;     // master’s own checksum
uint32_t checksumPast;
uint8_t checksumBuffer[4];   // receive buffer (4 bytes for CRC32)

// BT: Bluetooth availabilty check
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

String myName = "ESP32-BT-Master";                                                        // BT: Variable used to store the SERVER(Master) bluetooth device name; just for prinitng
String slaveName = "ESP32-BT-Slave";                                                      // BT: Variable used to store the CLIENT(Slave) bluetooth device name; just for prinitng; just for printing in this case
String MACadd = "44:1D:64:F1:AF:86";                                                      // This only for printing
uint8_t address[6] = { 0x44, 0x1D, 0x64, 0xF1, 0xAF, 0x86 };
BluetoothSerial SerialBT;                                                                 // BT: Set the Object SerialBT

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

//Send and calculate checksum
void checkCRC32() {
    // STEP 1: Generate random 8-byte data
  for (int i = 0; i < 8; i++) {
    randomData[i] = (uint8_t) random(0, 256);
  }

  // Debug: print generated data
  Serial.print("Master generated data: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("%02X ", randomData[i]);
  }
  Serial.println();

  // STEP 2: Calculate master checksum
  checksumMaster = calculateCRC32(randomData, 8);
  Serial.printf("Master calculated checksum: %08X\n", checksumMaster);

  // STEP 3: Send 8 bytes to slave
  SerialBT.write(randomData, 8);

  // STEP 4: Wait for 4-byte checksum from slave
  previousMillis = 0;
  while (SerialBT.available() < 4) {
    if (millis() - previousMillis >= 3000) { // 3 second timeout
      break;
    }
    delay(1);
  }
  SerialBT.readBytes(checksumBuffer, 4);

  // STEP 5: Reconstruct uint32_t from 4 received bytes (big-endian order)
  uint32_t checksumSlave =
      ((uint32_t)checksumBuffer[0] << 24) |
      ((uint32_t)checksumBuffer[1] << 16) |
      ((uint32_t)checksumBuffer[2] << 8)  |
      ((uint32_t)checksumBuffer[3]);

  
  if (checksumPast == checksumSlave) {
    SlaveConnected = false;
    return;
  } else {
    checksumPast = checksumSlave;
  }
  

  // Debug: print received checksum
  Serial.printf("Master received checksum from slave: %08X\n", checksumSlave);

  // STEP 6: Compare
  if (checksumMaster == checksumSlave) {
    Serial.println("✅ Checksums match");
  } else {
    Serial.println("❌ Checksums do not match");
  }
  //delay(1000);
}

// BT: Bt_Status callback function
void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_OPEN_EVT) {                                                        // BT: Checks if the SPP connection is open, the event comes// event == Client connected
    Serial.println ("Client Connected");                                                  // BT: Write to the serial monitor
    digitalWrite (LED_BT_BLUE, HIGH);                                                     // BT: Turn ON the BLUE bluetooth indication LED (solid light)
    digitalWrite (LED_BT_RED, LOW);                                                       // BT: Turn OFF the RED bluetooth indication LED
    SlaveConnected = true;                                                                // BT: Set the variable true = CLIENT is connected to the SERVER
    recatt = 0;                                                                           // BT: Reset the reconnect attempts counter
  }
  else if (event == ESP_SPP_CLOSE_EVT) {                                                  // BT: event == Client disconnected
    Serial.println("Client Disconnected");                                                // BT: Write to the serial monitor
    digitalWrite(LED_BT_RED, HIGH);                                                       // BT: Turn ON the RED bluetooth indication LED (solid light)
    digitalWrite(LED_BT_BLUE, LOW);                                                       // BT: Turn OFF the BLUE bluetooth indication LED
    SlaveConnected = false;                                                               // BT: Set the variable false = CLIENT connection lost
  }
}

void setup() {
  pinMode(LED_BT_BLUE, OUTPUT);                                                           // BT: Set up the onboard LED pin as output
  pinMode(LED_BT_RED, OUTPUT);                                                            // BT: Set up the onboard LED pin as output
  digitalWrite(LED_BT_RED, HIGH);                                                         // BT: Turn ON the RED LED = no connection established
  SlaveConnected = false;                                                                 // BT: Set the variable false = CLIENT is not connected
  Serial.begin(115200);                                                                   // Sets the data rate in bits per second (baud) for serial data transmission

  SerialBT.register_callback(Bt_Status);                                                  // BT: Define the Bt_Status callback
  SerialBT.begin(myName, true);                                                           // BT: Starts the bluetooth device with the name stored in the myName variable as SERVER(Master)
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
  SlaveConnect();                                                                         // BT: Calls the bluetooth connection function to cnnect to the CLIENT(Slave)
}

void SlaveConnect() {                                                                     // BT: This function connects/reconnects to the CLIENT(Slave)
  Serial.println("Function BT connection executed");                                      // BT: Write to the serial monitor
  Serial.printf("Connecting to slave BT device named \"%s\" and MAC address \"%s\" is started.\n", slaveName.c_str(), MACadd.c_str());  // BT: Write to the serial monitor
  SerialBT.connect(address);                                                              // BT: Establishing the connection with the CLIENT(Slave) with the Mac address stored in the address variable
}

void loop() {

  if (!SlaveConnected) {                                                                  // BT: Condition to evalute if the connection is established/lost 
    if (millis() - previousMillisReconnect >= 10000) {                                    // BT: Check that 10000ms is passed
      previousMillisReconnect = millis();                                                 // BT: Set previousMillisReconnect to current millis
      recatt++;                                                                           // BT: Increase the the reconnection attempts counter +1 
      Serial.print("Trying to reconnect. Attempt No.: ");                                 // BT: Write to the serial monitor
      Serial.println(recatt);                                                             // BT: Write the attempts count to the serial monitor
      Serial.println("Stopping Bluetooth...");                                            // BT: Write to the serial monitor
      SerialBT.end();                                                                     // BT: Close the bluetooth device
      Serial.println("Bluetooth stopped !");                                              // BT: Write to the serial monitor
      Serial.println("Starting Bluetooth...");                                            // BT: Write to the serial monitor
      SerialBT.begin(myName, true);                                                       // BT: Starts the bluetooth device with the name stored in the myName variable as SERVER(Master)
      Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
      SlaveConnect();                                                                     // BT: Calls the bluetooth connection function to cnnect to the CLIENT(Slave)
    }
  }

  if (SlaveConnected) {
    checkCRC32();
  }

  // BT: Data send/receive via bluetooth
  if (Serial.available()) {                                                               // BT: Checks if there are data from the serial monitor available
    SerialBT.write(Serial.read());                                                        // BT: Sends the data via bluetooth
  }
  if (SerialBT.available()) {                                                             // BT: Checks if there are data from the bluetooth available
    Serial.write(SerialBT.read());                                                        // BT: Write the data to the serial monitor
  }
}
