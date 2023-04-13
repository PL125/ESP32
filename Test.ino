#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2904.h>
#include <sys/time.h>
#include "esp32-hal-cpu.h"        // Support CPU clock change
#include "DualFunctionButton.h"   // https://github.com/BerranRemzi/dual-function-button



//#define SECURE      // Require confirmation before connections
#define MULTI_CON     // Support multiple clients connected simultaneously
#define DEBUG


#define BLE_DEVICE_NAME "ESALD"
#define TOUTCH_GPIO 4

#define POWER_BUTTON_GPIO 2
#define POWER_BUTTON (1ULL << POWER_BUTTON_GPIO)
#define BUTTON_PIN_BITMASK POWER_BUTTON

#define ESALD_SERVICE_UUID                      "1775cd75-a329-4607-a652-9a0a69ba77c8"
#define ESALD_CHARACTERISTIC_UUID               "71495a29-c548-4aa6-a75b-61959767faa2"
#define DAY_DATE_TIME_CHARACTERISTIC_UUID       "2A0A"        // 0x2A0A   00002a0a-0000-1000-8000-00805f9b34fb
#define BATTERY_LEVEL_STATE_CHARACTERISTIC_UUID "2A1B"        // 0x2A1B   00002a1b-0000-1000-8000-00805f9b34fb


#ifdef MULTI_CON
/*
  #undef BTM_SEC_MAX_DEVICE_RECORDS
  #define BTM_SEC_MAX_DEVICE_RECORDS  1 // Allow only one bonded device
*/
#endif


bool deviceConnected = false;
DualFunctionButton offButton(POWER_BUTTON_GPIO, 2000, INPUT);

// Create BLE Advertiser
BLEAdvertising *bleAdvertising;


// Create BLE Characteristic for Day Date Time
BLECharacteristic characteristicDayDateTime(
  DAY_DATE_TIME_CHARACTERISTIC_UUID,
  BLECharacteristic::PROPERTY_WRITE |
  BLECharacteristic::PROPERTY_NOTIFY
);


// Create BLE Characteristic for Battery Level State
BLECharacteristic characteristicBatteryLevelState(
  BATTERY_LEVEL_STATE_CHARACTERISTIC_UUID,
  BLECharacteristic::PROPERTY_READ |
  BLECharacteristic::PROPERTY_NOTIFY
);


// Create BLE Characteristic for ESALD
BLECharacteristic characteristic_ESALD(
  ESALD_CHARACTERISTIC_UUID,
  BLECharacteristic::PROPERTY_READ |
  BLECharacteristic::PROPERTY_NOTIFY
);


// Create BLE Descriptor for ESALD Characteristic
BLEDescriptor ESALD_Descriptor_2901(BLEUUID((uint16_t)0x2901));
BLE2904 *ESALD_Descriptor_2904 = new BLE2904();



int setTime(uint16_t year, uint8_t month, uint8_t day, uint8_t weekDay, uint8_t hour, uint8_t minutes, uint8_t seconds) {
  struct timeval tv;
  time_t t;
  struct tm tm;

  if (weekDay >= 0)
    tm.tm_wday = weekDay;
  tm.tm_mday = day;
  tm.tm_mon = (month - 1);
  tm.tm_year = (year - 1900);
  tm.tm_hour = hour;
  tm.tm_min = minutes;
  tm.tm_sec = seconds;

  t = mktime(&tm);
  tv.tv_sec = t;
  tv.tv_usec = 0;

  return settimeofday(&tv, NULL);
}


bool timeIsSet() {
    time_t now;
    struct tm timeInfo;

    //time(&now);
    now = time(NULL);
    localtime_r(&now, &timeInfo);

    if (timeInfo.tm_year > 70) {
        return true;
    }

  return false;
}


void printTime() {
  struct tm timeInfo;
  time_t now = time(NULL);

  localtime_r(&now, &timeInfo);
  //gmtime_r(&now, &timeInfo);

  //char buf[64] = "";
  //strftime(buf, sizeof(buf), "%c", &timeInfo);
  Serial.print("Current time: ");
  Serial.println(asctime(&timeInfo));
}


bool millisecondsPassed(unsigned long startMillis, int interval) {
  if (millis() - startMillis >= interval) {
    return true;
  }
  
  return false;
}


uint16_t getTouchValue(uint8_t pin) {
  return touchRead(pin);
}


int8_t getBatteryLevel() {
  //return random(1, 101); // generate random number between 1 & 100
  return 74;
}


uint8_t parseBatteryState(uint8_t *state) {
  return (state[0] & 0x03) | ((state[1] & 0x03) << 2) | ((state[2] & 0x03) << 4) | ((state[3] & 0x03) << 6);
}


byte getBatteryState() {
  /*uint8_t state[4] = { 0 };

  for(int i = 0; i < (sizeof(state) / sizeof(state[0])); i++) {
    state[i] = random(0, 4); // generate random number between 0 & 3
  }*/
  uint8_t state[4] = {0, 1, 2, 3};

  return parseBatteryState(state);
}


void getBatteryLevelState(byte *batteryLevelState) {
  int8_t level = getBatteryLevel();
  byte state = getBatteryState();   //uint8_t state = getBatteryState();

  batteryLevelState[0] = (byte) level;
  batteryLevelState[1] = (byte) state;
#ifdef DEBUG
  Serial.print("Battery Level = ");
  Serial.println(batteryLevelState[0]);
  Serial.print("Battery State = ");
  Serial.println(batteryLevelState[1], BIN);
#endif
}


float randomFloat(float minf, float maxf) {
// https://forum.arduino.cc/index.php?topic=371564.0
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // Use 1ULL<<63 to generate random double values
}


std::string parseBloodValues(float *blood) {
  int len = snprintf(NULL, 0, "%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;", blood[0], blood[1],
    blood[2], blood[3], blood[4], blood[5], blood[6], blood[7], blood[8], blood[9]);
  char str[len];
  //char *str = malloc((len + 1) * sizeof(char));
  
  sprintf(str, "%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;", blood[0], blood[1],
    blood[2], blood[3], blood[4], blood[5], blood[6], blood[7], blood[8], blood[9]);
  //snprintf(str, len + 1, "%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;", blood[0], blood[1],
//    blood[2], blood[3], blood[4], blood[5], blood[6], blood[7], blood[8], blood[9]);

/*
  std::string str = "";
  char *pStr = 0;

  int size = asprintf(&pStr, "%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;", blood[0], blood[1],
    blood[2], blood[3], blood[4], blood[5], blood[6], blood[7], blood[8], blood[9]);

  if (size > 0) {
    str = pStr;
  }
  free(pStat);
*/
  return str;
}


std::string getBloodValues() {
  float blood[10] = { 0 };

  blood[0] = randomFloat(0.01, 50.00);
  blood[1] = randomFloat(0.01, 50.00);
  blood[2] = randomFloat(0.01, 50.00);
  blood[3] = randomFloat(0.01, 50.00);
  blood[4] = randomFloat(0.01, 50.00);
  blood[5] = randomFloat(0.01, 50.00);
  blood[6] = randomFloat(0.01, 50.00);
  blood[7] = randomFloat(0.01, 50.00);
  blood[8] = randomFloat(0.01, 50.00);
  blood[9] = randomFloat(0.01, 50.00);
#ifdef DEBUG
  Serial.println("Blood values");
  for(int i = 0; i < (sizeof(blood) / sizeof(blood[0])); i++) {
    Serial.println(blood[i]);
  }
#endif
  return parseBloodValues(blood);
}


// Callbacks for client connection and disconnection
class bleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
    uint16_t connId = pServer->getConnId();       // uint16_t connId = param->connect.conn_id;
#ifdef DEBUG
    Serial.println("Connect");
    BLEAddress *address = new BLEAddress(param->connect.remote_bda);
    Serial.print("Address: ");
    Serial.println(address->toString().c_str());
    Serial.print("Id: ");
    Serial.println(connId);
#endif
    deviceConnected = true;

    characteristicDayDateTime.indicate();

#ifdef MULTI_CON
    // Start advertising again so other clients can also connect
    bleAdvertising->start();
#endif

    // Refresh cache
    esp_ble_gattc_search_service(4, connId, NULL);
  }

  void onDisconnect(BLEServer *pServer) {
#ifdef DEBUG
    Serial.println("Disconnect");
#endif
    if (pServer->getConnectedCount() <= 0) {
      deviceConnected = false;
    }
  }
};

#ifdef SECURE
class bleSecurityCallbacks: public BLESecurityCallbacks {

  uint32_t onPassKeyRequest() {
#ifdef DEBUG 
    Serial.println("onPassKeyRequest");
#endif
    return 123456;
  }

  bool onConfirmPIN(uint32_t pass_key) {
#ifdef DEBUG
    Serial.println("onConfirmPIN");
#endif
    vTaskDelay(5000);
    return true;
  }

  bool onSecurityRequest() {
#ifdef DEBUG
    Serial.println("onSecurityRequest");
#endif
    return true;
  }

  void onPassKeyNotify(uint32_t pass_key) {
#ifdef DEBUG
    Serial.println("onPassKeyNotify");
#endif
    Serial.print("PIN: ");
    Serial.println(pass_key);
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
#ifdef DEBUG
    Serial.println("onAuthenticationComplete");
#endif
    if (cmpl.success) {
      uint16_t length;
      esp_ble_gap_get_whitelist_size(&length);
      ESP_LOGD(LOG_TAG, "size: %d", length);
    }
  }
};
#endif

class dayDateTimeCallbackHandler: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string dayDateTime = pCharacteristic->getValue();

// Get Day Date Time
    if (dayDateTime.length() >= 7) {
      uint16_t year = (dayDateTime[0] & 0xFF) | (dayDateTime[1] << 8);
      uint8_t month = dayDateTime[2] & 0xFF;
      uint8_t day = dayDateTime[3] & 0xFF;
      uint8_t hour = dayDateTime[4] & 0xFF;
      uint8_t minute = dayDateTime[5] & 0xFF;
      uint8_t second = dayDateTime[6] & 0xFF;
      uint8_t dayOfWeek = -1;

      if (dayDateTime.length() >= 8) {
        dayOfWeek = dayDateTime[7] & 0xFF;
      }
#ifdef DEBUG
  Serial.printf("Weekday -> %d \n", dayOfWeek);
  Serial.printf("Date -> %02d/%02d/%02d \n", day, month, year);
  Serial.printf("Hour -> %02d:%02d:%02d \n", hour, minute, second);
#endif
// Set ESP32 internal RTC
      int ret = setTime(year, month, day, dayOfWeek, hour, minute, second);
    }
#ifdef DEBUG
    else {
      Serial.print("Received bad data = ");
      Serial.print(dayDateTime.c_str());
      Serial.println();
    }
#endif
  }
};


class batteryLevelStateCallbackHandler: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    static byte aux[2];
    byte batteryLevelState[2];

    getBatteryLevelState(batteryLevelState);

    if (memcmp(aux, batteryLevelState, sizeof(batteryLevelState)) != 0) {
      memcpy(aux, batteryLevelState, sizeof(batteryLevelState));

      characteristicBatteryLevelState.setValue(batteryLevelState, 2);
    }
  }
};


class ESALD_CallbackHandler: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
  }
};


void initBleServer() {
  BLEDevice::init(std::string(BLE_DEVICE_NAME));
  /*BLEDevice::setMTU(80);  // MTU should be larger than 23 and lower or equal to 517
  BLEDevice::setPower(ESP_PWR_LVL_P7);*/

#ifdef SECURE
  //
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);                                                             //BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);

  // Set security callbacks
  BLEDevice::setSecurityCallbacks(new bleSecurityCallbacks());

  // Set characteristics permissions
  characteristicDayDateTime.setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
#endif

  // Create BLE server
  BLEServer *bleServer = BLEDevice::createServer();

  // Set server callbacks
  bleServer->setCallbacks(new bleServerCallbacks());

  // Create BLE service
  BLEService *ESALDService = bleServer->createService(ESALD_SERVICE_UUID);

  // Add service characteristics
  ESALDService->addCharacteristic(&characteristicDayDateTime);
  ESALDService->addCharacteristic(&characteristicBatteryLevelState);
  ESALDService->addCharacteristic(&characteristic_ESALD);

  // Set descriptor 0x2904
  ESALD_Descriptor_2904->setFormat(BLE2904::FORMAT_UINT8);
  ESALD_Descriptor_2904->setNamespace(1);
  ESALD_Descriptor_2904->setUnit(0x27ad);

  // Set descriptor 0x2901
  ESALD_Descriptor_2901.setValue("Blood values");
  ESALD_Descriptor_2901.setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED);

  // Add characteristic descriptors
  characteristic_ESALD.addDescriptor(&ESALD_Descriptor_2901);
  characteristic_ESALD.addDescriptor(ESALD_Descriptor_2904);
  characteristic_ESALD.addDescriptor(new BLE2902());

  // Set characteristics callback handlers
  characteristicDayDateTime.setCallbacks(new dayDateTimeCallbackHandler());
  characteristicBatteryLevelState.setCallbacks(new batteryLevelStateCallbackHandler());
  characteristic_ESALD.setCallbacks(new ESALD_CallbackHandler());
  
#ifdef DEBUG
  Serial.println(characteristicDayDateTime.getUUID().toString().c_str());
#endif

  // Start BLE service
  ESALDService->start();

  // Retrieve the advertising object that can be used to advertise the existence of the server
  bleAdvertising = bleServer->getAdvertising();

  // Add a service uuid to exposed list of services
  bleAdvertising->addServiceUUID(ESALDService->getUUID());  // pAdvertising->addServiceUUID(ESALD_SERVICE_UUID);

#ifdef SECURE
  BLESecurity *bleSecurity = new BLESecurity();
  bleSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND); //ESP_LE_AUTH_REQ_SC_BOND
  bleSecurity->setCapability(ESP_IO_CAP_OUT);
  bleSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  //bleSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  //bleSecurity->setKeySize();
#endif

  // Start advertising
  bleAdvertising->start();
}


void wake() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  uint64_t mask = esp_sleep_get_ext1_wakeup_status();

  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 :
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1 :
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER :
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP :
      Serial.println("Wakeup caused by ULP program");
      break;
    case ESP_SLEEP_WAKEUP_GPIO :
      Serial.println("Wakeup caused by GPIO (light sleep only)");
      break;
    case ESP_SLEEP_WAKEUP_UART :
      Serial.println("Wakeup caused by UART (light sleep only)");
      break;
    default :
      Serial.print("Wakeup was not caused by deep sleep: ");
      Serial.println(wakeup_reason);
      break;
  }
  
  //rtc_gpio_deinit(gpio_num);  //After wake up from sleep, IO pad(s) used for wakeup will be configured as RTC IO. Before using these pads as digital GPIOs, reconfigure them using rtc_gpio_deinit(gpio_num) function.
}


void hibernate() {
  pinMode(POWER_BUTTON_GPIO, INPUT);

  //turnOffWifi();
  //turnOffBluetooth();

  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);   //RTC peripherals are powered down, internal pullup and pulldown resistors will be disabled
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);

//  rtc_gpio_isolate(GPIO_NUM_12);                                    //On ESP32-WROVER module, GPIO12 is pulled up externally. GPIO12 has also an internal pulldown in the ESP32 chip.
                                                                      //This means that in deep sleep, some current will flow through these external and internal resistors, increasing deep sleep current.
  esp_deep_sleep_start();
}



void setup() {
  touch_pad_init(); // Initialize touch pads
  Serial.begin(115200);

  //wake();
  printTime();
  initBleServer();
  setCpuFrequencyMhz(80); // Set CPU clock to 80MHz
}


void loop() {
  static bool busy = false;
  static unsigned long start;

  if ((getTouchValue(TOUTCH_GPIO) <= 25) && !busy) {
    busy = true;
    start = millis();
    characteristic_ESALD.setValue(getBloodValues());
    characteristic_ESALD.notify();
  }

  if (busy && millisecondsPassed(start, 2000)) {
    busy = false;
  }

  if (offButton.longPress()) {
#ifdef DEBUG
    Serial.println("Going to deep sleep mode now");
#endif
    hibernate();
  }
}






void convert(short value) {
  byte ret[4];
  //short value = 0x9A76;
  
  // Little Endian
  ret[0] = (byte)(value & 0xFF);
  ret[1] = (byte)((value >> 8) & 0xFF);
  
  
  // Big Endian
  ret[0] = (byte)((value >> 8) & 0xFF);
  ret[1] = (byte)(value & 0xFF);
}

void stopBleServer() {
#ifdef DEBUG
    Serial.println("stopBleServer");
#endif
  // Stop advertising
  esp_err_t errRc = esp_ble_gap_stop_advertising(); //bleAdvertising->stop();
  
  Serial.println(errRc);  //https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/esp32/esp_err.h
}

void bleClearBondedDevices() {
  int numBondedDevices = esp_ble_get_bond_device_num();

  if (numBondedDevices > 0) {
#ifdef DEBUG
    Serial.print("Bonded devices: ");
    Serial.println(numBondedDevices);
#endif

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * numBondedDevices);
    esp_ble_get_bond_device_list(&numBondedDevices, dev_list);

    for (int i = 0; i < numBondedDevices; i++) {
#ifdef DEBUG
      Serial.print("Removing device: ");
      Serial.println(BLEAddress(dev_list[i].bd_addr).toString().c_str());
#endif
      esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
  }  
}

void turnOffWifi() {
  esp_wifi_stop();
  esp_wifi_deinit();
}

void turnOffBluetooth() {
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
}

/*
enum state0 {
 unknown = 0,
 not_Supported,
 not_Present,
 present
}; 

enum state1 {
 unknown = 0,
 not_Supported,
 not_Discharging,
 discharging
};

enum state3 {
 unknown = 0,
 not_Chargeable,
 not_Charging,
 charging
};

enum state4 {
 unknown = 0,
 not_Supported,
 good_Level,
 critically_Low_Level
};
*/
