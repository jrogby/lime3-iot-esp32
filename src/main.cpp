#include <Arduino.h>
#include <string>
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define LOG_TAG ""
#else
#include "esp_log.h"
static const char* LOG_TAG = "BLEAdvertising";
#endif
#include "CRC16.h"
#include "CRC.h"
#include <Adafruit_Sleepydog.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define BLE_DEVICE_NAME "ORANGE"
const char *UPDATER_WIFI_PASSWORD = "123456789";
const uint32_t BLE_PASSWORD = 123456789;


// Set pins
#define RXD2 16           // Serial2, To where? Display?
//const int SERIAL2_RX = 16;
#define TXD2 17           // Serial2, ..
const int LOCK_PIN = 14;  // Existing lock or DIY?
#define BUZZZER_PIN 12    // Connect a buzzer to this pin
#define SHOCK_PIN 13      // Existing hw or DIY?


// Controller Codes
namespace EscCmdCode {
byte hearthBeatEscByte[16] ={ 0x46, 0x43, 0x11, 0x01, 0x00, 0x08, 0x4C, 0x49, 0x4D, 0x45, 0x42, 0x49, 0x4B, 0x45, 0xBE, 0x8A };
byte onEscByte[9] =         { 0x46, 0x43, 0x16, 0x61, 0x00, 0x01, 0xF1, 0xF2, 0x8F };
byte offEscByte[9] =        { 0x46, 0x43, 0x16, 0x61, 0x00, 0x01, 0xF0, 0xE2, 0xAE };
byte lightOnEscByte[9] =    { 0x46, 0x43, 0x16, 0x12, 0x00, 0x01, 0xF1, 0x2B, 0x26 };
byte lightOffEscByte[9] =   { 0x46, 0x43, 0x16, 0x12, 0x00, 0x01, 0xF0, 0x3B, 0x07 };
byte lightBlinkEscByte[9] = { 0x46, 0x43, 0x16, 0x13, 0x00, 0x01, 0x06, 0xC2, 0x6A };
}

// Command codes from ESP32 to the ESC (Controller) ... same as above..
// struct EscCmd {
//   byte HEARTBEAT[16] =  { 0x46, 0x43, 0x11, 0x01, 0x00, 0x08, 0x4C, 0x49, 0x4D, 0x45, 0x42, 0x49, 0x4B, 0x45, 0xBE, 0x8A };
//   byte ESC_ON[9] =      { 0x46, 0x43, 0x16, 0x61, 0x00, 0x01, 0xF1, 0xF2, 0x8F };
//   byte ESC_OFF[9] =     { 0x46, 0x43, 0x16, 0x61, 0x00, 0x01, 0xF0, 0xE2, 0xAE };
//   byte LIGHT_ON[9] =    { 0x46, 0x43, 0x16, 0x12, 0x00, 0x01, 0xF1, 0x2B, 0x26 };
//   byte LIGHT_OFF[9] =   { 0x46, 0x43, 0x16, 0x12, 0x00, 0x01, 0xF0, 0x3B, 0x07 };
//   byte LIGHT_ALT[9] =   { 0x46, 0x43, 0x16, 0x13, 0x00, 0x01, 0x06, 0xC2, 0x6A };
// } escCmd;

// Display Status Codes
namespace DisplayStatus {
  const String SCAN               = "21";
  const String ERROR              = "22";
  const String PAUSED             = "23";
  const String LOCKED             = "24";
  const String DONE               = "25";
  const String CHARGING           = "26";
  const String DRIVING            = "31";
  const String DRIVING_LOW_BAT    = "41";
  const String DRIVING_ALERT      = "42";
  const String DRIVING_NO_PARKING = "43";
  const String DRIVING_NO_RIDING  = "44";
  const String DRIVING_MAX_SPEED  = "45";
  const String UPGRADING          = "51";
}

// Status
bool deviceConnected = false; // set by BLE cb
bool oldDeviceConnected = false; // set by BLE cb

namespace app_state {}

byte isUnlocked = 0;
byte controllerIsOn = 0;
byte lightIsOn = 0;
byte unlockForEver = 0;
byte speed = 0;
byte alarmIsOn = 0;
byte throttle = 1;
byte battery = 0x00;
byte isCharging = 0x00;
String customDisplayStatus = "";


namespace settings {
uint8_t max_speed = 28;
  namespace alarm {
  uint8_t delay = 200;
  uint8_t reps = 15;
  }
}

bool isSending = false;

RTC_DATA_ATTR int bootCount = 0;

// BLE
#define SERVICE_UUID                  "653bb0e0-1d85-46b0-9742-3b408f4cb83f"
#define CHARACTERISTIC_UUID_MAIN      "00c1acd4-f35b-4b5f-868d-36e5668d0929"
#define CHARACTERISTIC_UUID_SETTINGS  "7299b19e-7655-4c98-8cf1-69af4a65e982"
#define CHARACTERISTIC_UUID_DEBUG     "83ea7700-6ad7-4918-b1df-61031f95cf62"
BLEServer *pServer = nullptr;


/*** beeps.ino ***/
namespace beep {
using namespace settings;
void alarm()          {for(int i=0;i<alarm::reps;i++){tone(BUZZZER_PIN,3000);delay(alarm::delay);noTone(BUZZZER_PIN);delay(alarm::delay);}}
void connected()      {tone(BUZZZER_PIN,300,100);delay(100);tone(BUZZZER_PIN,400,100);delay(100);tone(BUZZZER_PIN,500,100);delay(100);noTone(BUZZZER_PIN);}
void disconnected()   {tone(BUZZZER_PIN,300,100);delay(100);}
void lock()           {tone(BUZZZER_PIN,500,100);delay(100);tone(BUZZZER_PIN,400,100);delay(100);noTone(BUZZZER_PIN);}
void unlock_forever() {tone(BUZZZER_PIN,800,100);delay(100);tone(BUZZZER_PIN,800,100);delay(100);}
void unlock()         {tone(BUZZZER_PIN,400,100);delay(100);tone(BUZZZER_PIN,500,100);delay(100);noTone(BUZZZER_PIN);}
void ready()          {tone(BUZZZER_PIN,300,100);delay(100);tone(BUZZZER_PIN,400,100);delay(100);tone(BUZZZER_PIN,500,100);delay(100);noTone(BUZZZER_PIN);}
}

void send_command(byte* cmd, size_t len) {
  while(isSending); // wait
  isSending = true;
  Serial.write(cmd, len);
  delay(100);
  isSending = false;
}

// void send_command(EscCmd c) {
//   while(isSending); // wait
//   isSending = true;
//   // Serial.write(c, sizeof(&c));
//   delay(100);
//   isSending = false;
// }






/*** utilso.ino ***/
void lockScooter() {
  isUnlocked = 0;
  beep::lock();
}

void unlockScooter() {
  isUnlocked = 1;
  digitalWrite(LOCK_PIN, HIGH);
  beep::unlock();
}

void turnOnController() {
  controllerIsOn = 1;
  digitalWrite(LOCK_PIN, HIGH);
}

void turnOffController() {
  controllerIsOn = 0;
  digitalWrite(LOCK_PIN, LOW);
}




/*** ble_security.ino ***/
class MySecurity : public BLESecurityCallbacks {

  uint32_t onPassKeyRequest() {
    ESP_LOGI("MySecurity", "PassKeyRequest");
    return 123456789;
  }

  void onPassKeyNotify(uint32_t pass_key) {
    ESP_LOGI("MySecurity", "The passkey Notify number:%d", pass_key);
  }

  bool onConfirmPIN(uint32_t pass_key) {
    ESP_LOGI("MySecurity", "The passkey YES/NO number:%d", pass_key);
    vTaskDelay(5000);
    return true;
  }

  bool onSecurityRequest() {
    ESP_LOGI("MySecurity", "SecurityRequest");
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
    ESP_LOGI("MySecurity", "Authentication Complete");
    beep::connected();
  }
};


/*** controller.ino ***/
// CHARACTERISTIC_UUID_MAIN "00c1acd4-f35b-4b5f-868d-36e5668d0929"
class MainBLECallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pc) {
    auto val = pc->getValue();
    ESP_LOGD("MainBLECallback", "Write main %s (%i)", val.c_str(), sizeof(val.c_str()));

    if (val.length()>0) {
      String cmd = "";
      for(int i=0;i < val.length();i++){cmd = cmd + val[i];}
      
      // lock_scooter
      if(cmd=="lock") {
        ESP_LOGI("MainBLECallback", "Lock scooter");
        beep::lock(); unlockForEver = 0;
        send_command(EscCmdCode::offEscByte, sizeof(EscCmdCode::offEscByte)); isUnlocked = 0;
        send_command(EscCmdCode::lightOffEscByte, sizeof(EscCmdCode::lightOffEscByte)); lightIsOn = 0;
      }

      // unlock_scooter
      if(cmd=="unlock") {
        ESP_LOGI("MainBLECallback", "Unlock scooter");
        beep::unlock(); unlockForEver = 0;
        if(!controllerIsOn) {
          digitalWrite(LOCK_PIN, HIGH);
          delay(3000);
          controllerIsOn = 1;
          }
        send_command(EscCmdCode::onEscByte, sizeof(EscCmdCode::onEscByte)); isUnlocked = 1;
        send_command(EscCmdCode::lightOnEscByte, sizeof(EscCmdCode::lightOnEscByte)); lightIsOn = 1;
      }
      
      // controller_on
      if(cmd=="on") {
        ESP_LOGI("MainBLECallback", "Controller on");
        digitalWrite(LOCK_PIN, HIGH); controllerIsOn = 1;
      }

      // controller_off
      if(cmd=="off") {
        ESP_LOGI("MainBLECallback", "Controller off");
        digitalWrite(LOCK_PIN, LOW); controllerIsOn = 0; isUnlocked = 1;
      }

      // unlock_scooter_forever
      if(cmd=="unlockforever") {
        ESP_LOGI("MainBLECallback", "Unlock Forever");
        unlockForEver = 1; unlockScooter();
        beep::unlock_forever();
      }

      // trigger_alarm
      if(cmd == "alarm") {
        ESP_LOGI("MainBLECallback", "Alarm");
        send_command(EscCmdCode::lightBlinkEscByte, sizeof(EscCmdCode::lightBlinkEscByte));
        digitalWrite(LOCK_PIN, HIGH);
        beep::alarm();
        send_command(EscCmdCode::lightBlinkEscByte, sizeof(EscCmdCode::lightBlinkEscByte));
        if(controllerIsOn == 0) digitalWrite(LOCK_PIN, LOW);
      }

      // scooter_light_on
      if(cmd=="lighton") {
        ESP_LOGI("MainBLECallback", "Lights on");
        send_command(EscCmdCode::lightOnEscByte, sizeof(EscCmdCode::lightOnEscByte));
        lightIsOn = 1;
      }

      // scooter_light_off
      if(cmd=="lightoff") {
        ESP_LOGI("MainBLECallback", "Lights off");
        send_command(EscCmdCode::lightOffEscByte, sizeof(EscCmdCode::lightOffEscByte));
        lightIsOn = 0;
      }

      // shutdown (was same.. more like sleep..)
      if(cmd=="shutdown" || cmd == "reboot") {
        ESP_LOGI("MainBLECallback", "shotdown/reboot");
        digitalWrite(LOCK_PIN, LOW);
        controllerIsOn = 0;
        isUnlocked = 0;
        lightIsOn = 0;
        esp_deep_sleep_start();
      }
    }
  }
};

/*** display.ino ***/
// Thats the code to control the display

// Define CRC8 settings
namespace crc {
const uint8_t width   = 8;
const uint8_t poly    = 0x31;
const uint8_t init    = 0x0a;
const bool    refin   = true;
const bool    refout  = true;
const uint8_t xorout  = 0x00;
const uint8_t check   = 0xc1;
const uint8_t residue = 0x00;
}

// Currently I only know how to turn off the LED
void turnOffDisplayLed() {
  byte myByte[19] = { 0x4C, 0x42, 0x44, 0x43, 0x50, 0x01, 0x10, 0x1B, 0x00, 0x08, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00 };
  byte checksum = crc8(myByte, sizeof(myByte), crc::poly, crc::init, crc::xorout, crc::refin, crc::refout);
  myByte[18] = checksum;
  delay(300);
  for (int i = 0; i < sizeof(myByte); i += 1)
    Serial2.write(myByte[i]);
  delay(300);
}

void sendDisplayCommand(byte speed, byte battery, String status) {
  
  // speed = (speed/50.0) * 500.0;
  String SPEED_HEX = String(speed, HEX);
  // Add leading zeros to speed if necessary
  while (SPEED_HEX.length() < 4)
    SPEED_HEX = "0" + SPEED_HEX;

  // Convert battery to hex
  String BATT_HEX = String(battery, HEX);
  BATT_HEX = BATT_HEX.length() < 2 ? "0" + BATT_HEX : BATT_HEX;
  
  // Create hex command
  String input_str = "4C42444350011011000901" + status + "01" + BATT_HEX + "02" + SPEED_HEX + "0100";
  int input_len = input_str.length();

  byte input_bytes[input_len/2]; // byte array to store the converted values

  for (int i=0; i < input_len; i+=2)
    input_bytes[i / 2] = strtoul(input_str.substring(i, i + 2).c_str(), NULL, 16);

  // Generate checksum
  uint8_t crc_value = crc8(input_bytes, input_len/2, crc::poly, crc::init, crc::xorout, crc::refin, crc::refout);

  // Concatenate input hex string and CRC8 checksum
  String output_str = input_str + String(crc_value, HEX);

  // Convert concatenated string to bytes and send to display
  for (int i = 0; i < output_str.length(); i += 2) {
    uint8_t byte = strtoul(output_str.substring(i,i+2).c_str(),nullptr,16);
    Serial2.write(byte);
  }
}


/*** controller.ino ***/
BLECharacteristic *pDebugCharacteristic;
// This is the code for the original controller
unsigned long read_controller_prev = 0;
const long read_controller_interval = 500;

void read_controller() {
  unsigned long now = millis();

  if(now - read_controller_prev >= read_controller_interval) {
    read_controller_prev = now;

    if (Serial.available() == 42) {
      byte cmd[42];
      Serial.readBytes(cmd, 42);
      pDebugCharacteristic->setValue(cmd, sizeof(cmd));
      pDebugCharacteristic->notify();

      uint16_t new_checksum = crc16(cmd, sizeof(cmd) - 2, 0x1021, 0x0000, 0x0000, false, false);
      uint16_t old_checksum = (uint16_t(cmd[40]) << 8) | uint16_t(cmd[41]);  // get a pointer to the last two bytes of the command array and interpret them as a uint16_t

      // Check if the command has the correct checksum
      if (old_checksum == new_checksum) {
        speed       = (cmd[8] / 172.0) * settings::max_speed;
        battery     = cmd[19];
        throttle    = cmd[28];
        isCharging  = cmd[21];
        isUnlocked  = cmd[23] == 0xF1 ? 1 : 0;
        lightIsOn   = cmd[29] == 0x4D ? 1 : 0;
      }
    } else {}

    while (Serial.available() > 0)
      char t = Serial.read();
  }
}


//UARTTaskCode: send command to display every 300ms
void command_display_update(void *pvParameters) {
  for (;;) {
    if (isUnlocked == 1)        sendDisplayCommand(speed, battery, customDisplayStatus != "" ? customDisplayStatus : DisplayStatus::DRIVING);
    else {
      if(isCharging)            sendDisplayCommand(speed, battery, customDisplayStatus != "" ? customDisplayStatus : DisplayStatus::CHARGING);
      else if(deviceConnected)  sendDisplayCommand(speed, battery, customDisplayStatus != "" ? customDisplayStatus : DisplayStatus::LOCKED);
      else                      sendDisplayCommand(speed, battery, customDisplayStatus != "" ? customDisplayStatus : DisplayStatus::SCAN);
    }
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}



/*** settings.ino ***/
// CHARACTERISTIC_UUID_SETTINGS "7299b19e-7655-4c98-8cf1-69af4a65e982"
BLECharacteristic *pSettingsCharacteristic;

class SettingsBLECallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string val = pCharacteristic->getValue();
    ESP_LOGD("SettingsBLECallback", "Write setting %s (%i)", val.c_str(), sizeof(val.c_str()));

    char data[4];
    memcpy(data, val.c_str(), 4);

    switch (data[0]) {
      
      case 0x01: // handle max_speed settings
        settings::max_speed = data[1];
        ESP_LOGI("SettingsBLECallback", "Changing max_speed to %d.", settings::max_speed);
        break;
      
      case 0x02: // handle alarm settings
        settings::alarm::delay  = data[1];
        settings::alarm::reps   = data[2];
        ESP_LOGI("SettingsBLECallback", "Set alarm delay to %d, alarm reps to %d.", settings::alarm::delay, settings::alarm::reps);
        break;
      
      case 0x03:
        customDisplayStatus = data[1] == 0x00 ? "" : String(data[1], HEX); // misstyped 0
        ESP_LOGI("SettingsBLECallback", "Set custom display status to %s.", customDisplayStatus.c_str());
        break;
      
      case 0x4:
        ESP_LOGI("SettingsBLECallback", "Turn display off.");
        turnOffDisplayLed();
        break;
    }
    
    byte settings[] = { settings::max_speed, settings::alarm::delay, settings::alarm::reps };
    pSettingsCharacteristic->setValue(settings, sizeof(settings));
    pSettingsCharacteristic->notify();
  }
};

BLECharacteristic *pMainCharacteristic;


TaskHandle_t UARTTask; // Display Task

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

/*** setup.ino ***/
void setup() {
  
  /** TODO: Replace by using ReactESP */
  xTaskCreatePinnedToCore(command_display_update, "UART Update Display", 5000, nullptr, 1, &UARTTask, 0);

  pinMode(LOCK_PIN,OUTPUT); digitalWrite(LOCK_PIN,HIGH); controllerIsOn=1;

  // Sleep wakeup on TouchPad3 (GPIO15)
  //touchSleepWakeUpEnable(T3,40);
  // Sleep wakeup to shock sensor
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,1);

  Serial.begin(115200); // was 9600 for com to controller? or just console?
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  ESP_LOGI("BLE", "Device init");
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  /* Required in authentication process to provide displaying and/or input passkey or yes/no butttons confirmation */
  BLEDevice::setSecurityCallbacks(new MySecurity());
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Main
  pMainCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_MAIN,BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  pMainCharacteristic->setCallbacks(new MainBLECallback());
  pMainCharacteristic->addDescriptor(new BLE2902());
  ESP_LOGI("BLE", "Created pMainCharacteristic");
  
  // Debug
  pDebugCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_DEBUG,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  pDebugCharacteristic->addDescriptor(new BLE2902());
  ESP_LOGI("BLE", "Created pDebugCharacteristic");
  
  // Settings
  pSettingsCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_SETTINGS,BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  pSettingsCharacteristic->setCallbacks(new SettingsBLECallback());
  pSettingsCharacteristic->addDescriptor(new BLE2902());
  ESP_LOGI("BLE", "Created pSettingsCharacteristic");

  pService->start();
  ESP_LOGI("BLE", "BLE Service started");

  BLEAdvertising *pad = pServer->getAdvertising();
  pad->addServiceUUID(SERVICE_UUID);
  pad->start();
  ESP_LOGI("BLE", "Advertising service");
  
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;  //bonding with peer device after authentication
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;                     //set the IO capability to No output No input
  uint8_t key_size = 16;                                       //the key size should be 7~16 bytes
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  //set static passkey
  uint32_t passkey = 123456;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  uint8_t oob_support = ESP_BLE_OOB_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  //    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
  /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribut to you,
    and the response key means which key you can distribut to thye Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribut to you,
    and the init key means which key you can distribut to the slave. */
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  
  ESP_LOGI("BLE", "Init complete"); beep::ready();
  turnOffDisplayLed();
}


/*** loop.ino ***/
unsigned long prevMillis = 0;
const long linterval = 250;

void loop() {
  // Go to deep sleep if more than 1 hour have passed
  if((3600 -(millis()/1000)) <= 0) {digitalWrite(LOCK_PIN,LOW); controllerIsOn=0; esp_deep_sleep_start();}
  // Sound alarm condition: if(digitalRead(SHOCK_PIN) == HIGH) alarmBeeb();

  // Every ~250ms, when device is connected, set/update main & settings characteristics
  unsigned long currMillis = millis();
  if(currMillis - prevMillis >= linterval) {
    prevMillis = currMillis;
    if (deviceConnected) {
      byte settings[] = {settings::max_speed,settings::alarm::delay,settings::alarm::reps};
      pSettingsCharacteristic->setValue(settings, sizeof(settings));

      byte commands[] = {isUnlocked,unlockForEver,speed,battery,throttle,lightIsOn,controllerIsOn,isCharging,alarmIsOn};
      pMainCharacteristic->setValue(commands, sizeof(commands));
      
      pMainCharacteristic->notify();
    }
  }

  // Send heartbeat
  if(controllerIsOn && !isSending)
    send_command(EscCmdCode::hearthBeatEscByte, sizeof(EscCmdCode::hearthBeatEscByte));
  
  // When disconnecting
  if(!deviceConnected && oldDeviceConnected) {
    ESP_LOGI("BLE", "Device disconnected");
    if(unlockForEver == 0) {lockScooter(); digitalWrite(LOCK_PIN, LOW); controllerIsOn = 0;}
    beep::disconnected();
    delay(500); // the bluetooth stack may need this
    
    ESP_LOGI("BLE", "Restart advertising");
    pServer->startAdvertising(); // restart advertising
    oldDeviceConnected = deviceConnected;
  }
  
  // When connecting
  if(deviceConnected && !oldDeviceConnected) {
    ESP_LOGI("BLE", "Device connected");
    unlockForEver = 0;
    oldDeviceConnected = deviceConnected;
  }
  
  // When controller is on
  if(controllerIsOn)
    read_controller();
}
