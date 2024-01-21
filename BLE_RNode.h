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

      Serial.println("Connect..");

    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      bt_state = BT_STATE_NA;

      Serial.println("Disconnect..");
      delay(500);
      pServer->startAdvertising(); // restart advertising
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

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



bool ble_init() {
  //Serial.begin(115200);
  //Serial.println("Starting BLE work!");
  static char bt_devname[15];

  // Initialise serial communication
  memset(BLE_Buffer, 0, sizeof(BLE_Buffer));
  fifo_init(&BLE_FIFO, BLE_Buffer, CONFIG_UART_BUFFER_SIZE);

  memset(BLE_TX_Buffer, 0, sizeof(BLE_TX_Buffer));
  fifo_init(&BLE_TX_FIFO, BLE_TX_Buffer, sizeof(BLE_TX_Buffer)-1);


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
            data[BT_DEV_ADDR_LEN] = EEPROM.read(eeprom_addr(ADDR_SIGNATURE));
            unsigned char *hash = MD5::make_hash(data, BT_DEV_ADDR_LEN);
            memcpy(bt_dh, hash, BT_DEV_HASH_LEN);
            sprintf(bt_devname, "RNode_%02X%02X", bt_dh[14], bt_dh[15]);
            bt_devname[10]=0;
            free(data);

// Heltec device returned RNode_5949 for first test device

    Serial.println(bt_devname);

  //BLEDevice::deinit(true);
  // init here does not seem to change Bluetooth
  // device name in initial testing
  //BLEDevice::init(bt_devname);


  /*
   * Here we have implemented simplest security. This kind security does not provide authentication
   */
  // Remove for NimBLE
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

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

  // sample write
//  pCharacteristicTx->setValue("Hello World says Neil");
//  pCharacteristicTx->notify();

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
//  Serial.println("Characteristic defined! Now you can read it in your phone!");


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
