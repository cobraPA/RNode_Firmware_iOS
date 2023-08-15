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


#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define BT_DEV_ADDR_LEN 6
#define BT_DEV_HASH_LEN 16

//char bt_dh[BT_DEV_HASH_LEN];
//char bt_devname[11];

bool ble_init() {
  //Serial.begin(115200);
  //Serial.println("Starting BLE work!");
  static char bt_devname[15];

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
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
//  Serial.println("Characteristic defined! Now you can read it in your phone!");


  return true;
}
