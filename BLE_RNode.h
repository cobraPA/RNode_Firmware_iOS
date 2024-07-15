/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/

/*
From

*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
//#include <BLEAdvertising.h>
// commented usage of 2902 for now...
//#include <BLE2902.h>

/*
Uses BLE only implementation, use full stack if 
we continue supporting Classic Bluetooth for Android
#include <NimBLEDevice.h>
*/

/*


Some setup based on BLE 'UART' model from here
https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/Arduino/BLE_uart/BLE_uart.ino
*/

//#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331d00d"
//#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361bd01d"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361bd02d"

/*
 * Optionally, the defined UART UUIDs from Nordic.  This is expected to
 * be a BLE to UART (serial) bridge.
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
 */

#define BT_DEV_ADDR_LEN 6
#define BT_DEV_HASH_LEN 16

//char bt_dh[BT_DEV_HASH_LEN];
//char bt_devname[11];

bool deviceConnected = false;
bool oldDeviceConnected = false;

BLECharacteristic *pCharacteristicTx;

// RX Buffer
FIFOBuffer BLE_FIFO;
uint8_t BLE_Buffer[CONFIG_UART_BUFFER_SIZE+1];

// BLE transmit consolidation buffer
FIFOBuffer BLE_TX_FIFO;
uint8_t BLE_TX_Buffer[400];


//----------------------------

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      bt_state = BT_STATE_BLE_CONNECTED;
      cable_state = CABLE_STATE_DISCONNECTED;

      // ble debug
      Serial.println("Connect..");

    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      //bt_state = BT_STATE_ON;
      bt_state = BT_STATE_BLE_DISCONNECTED;
      //stopRadio();
      // not available here
      // ble debug
      Serial.println("Disconnect..");
      delay(500);
      pServer->startAdvertising(); // restart advertising
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      /*std::string*/ String rxValue = pCharacteristic->getValue();

      for (int i = 0; i < rxValue.length(); i++)
          if (!fifo_isfull(&BLE_FIFO)) {
            fifo_push(&BLE_FIFO, rxValue[i] );
          }

        //Serial.print("RX: ");
        //for (int i = 0; i < rxValue.length(); i++)
          //Serial.print(rxValue[i]);

        //Serial.println();

  /*
      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
      */

    }
};


class MySecurity : public BLESecurityCallbacks {

	uint32_t onPassKeyRequest(){
    ESP_LOGI(LOG_TAG, "PassKeyRequest");
    uint32_t pass_key = 112233;
    Serial.println("Key request ");
    Serial.println(pass_key);
		return pass_key;
	}
	void onPassKeyNotify(uint32_t pass_key){
    ESP_LOGI(LOG_TAG, "The passkey Notify number:%d", pass_key);
    Serial.print("Key notify ");
    Serial.print(pass_key);
    Serial.print("  state ");
    Serial.println(bt_state);
    //bt_state == BT_STATE_PAIRING and bt_ssp_pin != 0
    // Enable display of passkey on RNode
    bt_state = BT_STATE_PAIRING;
    bt_ssp_pin = pass_key;
	}
	bool onConfirmPIN(uint32_t pass_key){
    ESP_LOGI(LOG_TAG, "The passkey YES/NO number:%d", pass_key);
    Serial.print("confirm pin ");
    Serial.println(pass_key);
		return true;
	}
	bool onSecurityRequest(){
    ESP_LOGI(LOG_TAG, "SecurityRequest");
    Serial.println("Sec request");
		return true;
	}

	void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
		ESP_LOGI(LOG_TAG, "Starting BLE work!");
    //esp_ble_gap_update_whitelist(true, cmpl.bd_addr);
    esp_ble_gap_update_whitelist(true, cmpl.bd_addr, BLE_WL_ADDR_TYPE_RANDOM);//BLE_ADDR_TYPE_PUBLIC
    Serial.println("Auth compl!");
    bt_state = BT_STATE_BLE_CONNECTED;
    // display checks this as flag, leave it in place?
    // no, indicates pair in progress. we are done.
    bt_ssp_pin = 0;

	}
};
/*
C:\Users\cobra\Documents\Arduino\RNode_Firmware\BLE_RNode.h:140:52: error: too few arguments to function 'esp_err_t esp_ble_gap_update_whitelist(bool, uint8_t*, esp_ble_wl_addr_type_t)'
     esp_ble_gap_update_whitelist(true, cmpl.bd_addr);

In file included from C:\Users\cobra\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.14\libraries\BLE\src/BLEDevice.h:12,
                 from C:\Users\cobra\Documents\Arduino\RNode_Firmware\BLE_RNode.h:11,
                 from C:\Users\cobra\Documents\Arduino\RNode_Firmware\Utilities.h:61,
                 from C:\Users\cobra\Documents\Arduino\RNode_Firmware\RNode_Firmware.ino:18:
C:\Users\cobra\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.14/tools/sdk/esp32s3/include/bt/host/bluedroid/api/include/api/esp_gap_ble_api.h:1573:11: note: declared here
 esp_err_t esp_ble_gap_update_whitelist(bool add_remove, esp_bd_addr_t remote_bda, esp_ble_wl_addr_type_t wl_addr_type);
*/


bool ble_init() {
  //Serial.begin(115200);
  Serial.println("Starting BLE work!");
  // move to global - config.h
  //static char bt_devname[15];

  // Initialise serial communication
  memset(BLE_Buffer, 0, sizeof(BLE_Buffer));
  fifo_init(&BLE_FIFO, BLE_Buffer, CONFIG_UART_BUFFER_SIZE);

  memset(BLE_TX_Buffer, 0, sizeof(BLE_TX_Buffer));
  fifo_init(&BLE_TX_FIFO, BLE_TX_Buffer, sizeof(BLE_TX_Buffer)-1);

  ////char *data = (char*)malloc(BT_DEV_ADDR_LEN+1);
  ////data[BT_DEV_ADDR_LEN] = EEPROM.read(eeprom_addr(ADDR_SIGNATURE));

  BLEDevice::init("RNode");
  //BLEDevice::init(bt_devname);

  BLEAddress myAddr = BLEDevice::getAddress();
  esp_bd_addr_t* bda_ptr = myAddr.getNative();
  //uint8_t* bda_ptr[6] = myAddr.getNative();
            char *data = (char*)malloc(BT_DEV_ADDR_LEN+1);
            for (int i = 0; i < BT_DEV_ADDR_LEN; i++) {
                data[i] =  (*bda_ptr)[i];
                //data[i] =  myAddr.address[i];
            }
// moved up for init
//            data[BT_DEV_ADDR_LEN] = EEPROM.read(eeprom_addr(ADDR_SIGNATURE));
            unsigned char *hash = MD5::make_hash(data, BT_DEV_ADDR_LEN);
            memcpy(bt_dh, hash, BT_DEV_HASH_LEN);
            sprintf(bt_devname, "RNode_%02X%02X", bt_dh[14], bt_dh[15]);
            bt_devname[10]=0;
            free(data);

//BLEAdvertisementData *advData = new BLEAdvertisementData();
// Neither made a difference when added to advertisement later from iPhone
//advData->setShortName(bt_devname);
//advData->setName(bt_devname);

// For now, once we have address, then deinit and re-init with
// known name
  BLEDevice::deinit();
  BLEDevice::init(bt_devname);
  BLEDevice::setSecurityCallbacks(new MySecurity());

// Heltec device returned RNode_5949 for first test device

      // ble debug
    Serial.println(bt_devname);

  //BLEDevice::deinit(true);
  // init here does not seem to change Bluetooth
  // device name in initial testing
  //BLEDevice::init(bt_devname);


  // Remove for NimBLE
  ////BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  #if HAS_DISPLAY
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);
  #else
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  #endif

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristicRx = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE |
                                         /* support no-response writes */
                                         BLECharacteristic::PROPERTY_WRITE_NR 
                                         /* BLECharacteristic::PROPERTY_WRITE */
                                         /* NimBLE
                                         NIMBLE_PROPERTY::READ |
                                         NIMBLE_PROPERTY::READ_ENC |
                                         NIMBLE_PROPERTY::WRITE |
                                         NIMBLE_PROPERTY::WRITE_ENC */
                                       );

  //pCharacteristicRx->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  #if HAS_DISPLAY
  pCharacteristicRx->setAccessPermissions(ESP_GATT_PERM_WRITE_ENC_MITM);
  #else
  pCharacteristicRx->setAccessPermissions(ESP_GATT_PERM_WRITE_ENCRYPTED);
  #endif
  //pCharacteristicRx->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM);
  // reports
  // E (13439) BT_GATT: gatts_write_attr_perm_check - GATT_INSUF_AUTHENTICATION: MITM required
 
  pCharacteristicRx->setCallbacks(new MyCallbacks());

  //BLECharacteristic *pCharacteristicTx
  pCharacteristicTx = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_TX,
                                         /*BLECharacteristic::PROPERTY_WRITE  | */
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  // Not required?  Used in BLE UART sample
  //pCharacteristicTx->addDescriptor(new BLE2902());
  #if HAS_DISPLAY
  pCharacteristicTx->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM);
  #else
  pCharacteristicTx->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED);
  #endif


  // sample write
//  pCharacteristicTx->setValue("Hello World says Neil");
//  pCharacteristicTx->notify();

  pService->start();
  //BLEAdvertising *pAdvertising = pServer->getAdvertising();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  // from https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  
  // Neither of these worked (to show device
  // name on iOS)
  //pAdvertising->setShortName(bt_devname);
  //pAdvertising->setAdvertisementData(*advData);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  ////pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  //pAdvertising->start();

  BLESecurity *pSecurity = new BLESecurity();
  #if HAS_DISPLAY
  //pSecurity->setStaticPIN(112233); 
  //pSecurity->set
  //pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);
  ////pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
  // see above for MITM error
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
//  Serial.println("Characteristic defined! Now you can read it in your phone!");
  ////pSecurity->setCapability(ESP_IO_CAP_IO);  // review key on phone (confirm pin)
  //pSecurity->setCapability(ESP_IO_CAP_NONE); // can't use, will pair but we set MITM on characteristics, so they will fail (read/write)
  pSecurity->setCapability(ESP_IO_CAP_OUT);  // type in pin on phone (key notify)
  //pSecurity->setCapability(ESP_IO_CAP_KBDISP);  // review on phone (confirm pin)

  #else  // no display
  pSecurity->setStaticPIN(123456); 
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
  pSecurity->setCapability(ESP_IO_CAP_KBDISP);  // review on phone (confirm pin)
  
  #endif
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  return true;
}


#define FEND			0xC0
int RX_Fend_flag = 0;

void BLEWrite(uint8_t byte) {
//  char val[2];
//  val[0] = byte;
//  val[1] = 0;
  //std::string send( (char) byte);
  //Serial.println("BLEWrite..");
//  pCharacteristicTx->setValue(val);
//  pCharacteristicTx->notify();

  uint8_t outbuf[400];

  if (!fifo_isfull(&BLE_TX_FIFO)) {
    fifo_push(&BLE_TX_FIFO, byte );
  } else {
    Serial.println("TX fifo full");
  }
  //Serial.print("fifo ");
  //Serial.println( (char)byte);
  switch( RX_Fend_flag) {
    case 0:
          if (byte == FEND) {
            RX_Fend_flag = 1;
            //Serial.println("found FEND");
          }
          break;
    case 1:
          if (byte == FEND) {
            RX_Fend_flag = 0;
            // send it
            uint8_t data = 0;
            int txLen = 0;

            // grab first FEND
            if (!fifo_isempty(&BLE_TX_FIFO)) {
              char sbyte = fifo_pop(&BLE_TX_FIFO);
              data = sbyte;
              outbuf[txLen++] = data;
            }
            int done = 0;
            while( !done) {
              char sbyte = fifo_pop(&BLE_TX_FIFO);
              data = sbyte;
              outbuf[txLen++] = data;
              //Serial.print("out len");
              //Serial.println(txLen);
              if( data == FEND ) done = 1;
            }

              // set with size
              //void setValue(uint8_t* data, size_t size);
            pCharacteristicTx->setValue(outbuf, txLen);
            pCharacteristicTx->notify();

          }
          break;
    default:
      break;
  }

}

uint8_t BLERead( void ) {
  uint8_t data = 0;

  //Serial.print("BLERead ");
  #if MCU_VARIANT == MCU_ESP32
      //buffer_serial();
      if (!fifo_isempty(&BLE_FIFO)) {
        char sbyte = fifo_pop(&BLE_FIFO);
        data = sbyte;
      }
  #else
    if (!fifo_isempty_locked(&BLE_FIFO)) {
      char sbyte = fifo_pop(&BLE_FIFO);
      data = sbyte;
    }
  #endif

  //Serial.print(data);

  return data;
}

bool BLE_data_available( void ) {
  bool stat = false;

  #if MCU_VARIANT == MCU_ESP32
      if (!fifo_isempty(&BLE_FIFO)) {
        stat = true;
      }
  #else
    if (!fifo_isempty_locked(&BLE_FIFO)) {
        stat = true;
    }
  #endif

  return stat;
}
