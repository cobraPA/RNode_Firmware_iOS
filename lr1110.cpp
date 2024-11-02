// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license.

// Modifications and additions copyright 2023 by Mark Qvist
// Obviously still under the MIT license.

#include "Boards.h"

#if MODEM == LR1110
#include "lr1110.h"

// todo, why is this in modem?
#if MCU_VARIANT == MCU_ESP32
//  #if MCU_VARIANT == MCU_ESP32 and !defined(CONFIG_IDF_TARGET_ESP32S3)
//    #include "soc/rtc_wdt.h"
//  #endif

// todo, why is this in modem?
//https://github.com/espressif/esp-idf/issues/8855
#include "hal/wdt_hal.h"

  #define ISR_VECT IRAM_ATTR
#else
  #define ISR_VECT
#endif

volatile uint8_t TXCompl_flag = 0;

#define OP_RF_FREQ_6X               0x86
#define OP_SLEEP_6X                 0x84
#define OP_STANDBY_6X               0x80
#define OP_TX_6X                    0x83
#define OP_RX_6X                    0x82
#define OP_PA_CONFIG_6X             0x95
#define OP_SET_IRQ_FLAGS_6X         0x08 // also provides info such as
                                      // preamble detection, etc for
                                      // knowing when it's safe to switch
                                      // antenna modes
#define OP_CLEAR_IRQ_STATUS_6X      0x02
#define OP_GET_IRQ_STATUS_6X        0x12
#define OP_RX_BUFFER_STATUS_6X      0x13
#define OP_PACKET_STATUS_6X         0x14 // get snr & rssi of last packet
#define OP_CURRENT_RSSI_6X          0x15
#define OP_MODULATION_PARAMS_6X     0x8B // bw, sf, cr, etc.
#define OP_PACKET_PARAMS_6X         0x8C // crc, preamble, payload length, etc.
#define OP_STATUS_6X                0xC0
#define OP_TX_PARAMS_6X             0x8E // set dbm, etc
#define OP_PACKET_TYPE_6X           0x8A
#define OP_BUFFER_BASE_ADDR_6X      0x8F
#define OP_READ_REGISTER_6X         0x1D
#define OP_WRITE_REGISTER_6X        0x0D
#define OP_DIO3_TCXO_CTRL_6X        0x97
#define OP_DIO2_RF_CTRL_6X          0x9D
#define OP_CAD_PARAMS               0x88
#define OP_CALIBRATE_6X             0x89
#define OP_RX_TX_FALLBACK_MODE_6X   0x93
#define OP_REGULATOR_MODE_6X        0x96
#define OP_CALIBRATE_IMAGE_6X       0x98

#define MASK_CALIBRATE_ALL          0x7f

#define IRQ_TX_DONE_MASK_6X         0x01
#define IRQ_RX_DONE_MASK_6X         0x02
#define IRQ_HEADER_DET_MASK_6X      0x10
#define IRQ_PREAMBLE_DET_MASK_6X    0x04
#define IRQ_PAYLOAD_CRC_ERROR_MASK_6X 0x40
#define IRQ_ALL_MASK_6X             0b0100001111111111

#define MODE_LONG_RANGE_MODE_6X     0x01

#define OP_FIFO_WRITE_6X            0x0E
#define OP_FIFO_READ_6X             0x1E
#define REG_OCP_6X                0x08E7
#define REG_LNA_6X                0x08AC // no agc in sx1262
#define REG_SYNC_WORD_MSB_6X      0x0740
#define REG_SYNC_WORD_LSB_6X      0x0741
#define REG_PAYLOAD_LENGTH_6X     0x0702 // https://github.com/beegee-tokyo/SX126x-Arduino/blob/master/src/radio/sx126x/sx126x.h#L98
#define REG_RANDOM_GEN_6X         0x0819

#define MODE_TCXO_3_3V_6X           0x07
#define MODE_TCXO_3_0V_6X           0x06
#define MODE_TCXO_2_7V_6X           0x06
#define MODE_TCXO_2_4V_6X           0x06
#define MODE_TCXO_2_2V_6X           0x03
#define MODE_TCXO_1_8V_6X           0x02
#define MODE_TCXO_1_7V_6X           0x01
#define MODE_TCXO_1_6V_6X           0x00

#define MODE_STDBY_RC_6X            0x00
#define MODE_STDBY_XOSC_6X          0x01
#define MODE_FALLBACK_STDBY_RC_6X   0x20
#define MODE_IMPLICIT_HEADER        0x01
#define MODE_EXPLICIT_HEADER        0x00

#define SYNC_WORD_6X              0x1424

#define XTAL_FREQ_6X (double)32000000
#define FREQ_DIV_6X (double)pow(2.0, 25.0)
#define FREQ_STEP_6X (double)(XTAL_FREQ_6X / FREQ_DIV_6X)

#if defined(NRF52840_XXAA)
  extern SPIClass spiModem;
  #define SPI spiModem
#endif

//extern SPIClass SPI;

#define MAX_PKT_LENGTH           255

lr11xx::lr11xx() :
//  _spiSettings(8E6, MSBFIRST, SPI_MODE0),
  _spiSettings(8e8, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN), _busy(LORA_DEFAULT_BUSY_PIN), _rxen(LORA_DEFAULT_RXEN_PIN),
  _frequency(0),
  _txp(0),
  _sf(0x07),
  _bw(0x04),
  _cr(0x01),
  _ldro(0x00),
  _packetIndex(0),
  _preambleLength(18),
  _implicitHeaderMode(0),
  _payloadLength(255),
  _crcMode(1),
  _fifo_tx_addr_ptr(0),
  _fifo_rx_addr_ptr(0),
  _packet{0},
  _preinit_done(false),
  _onReceive(NULL)
{
  // overide Stream timeout value
  setTimeout(0);
  modulationDirty = packetParamsDirty = 1;
}

bool lr11xx::preInit() {

  Serial.println("XXXXX");
  Serial.println("PreInit");
  Serial.println("XXXXX");

#if 0
  NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
  {
    // wait for start
  }
#endif

  // generate byte of random with built in 
  // Random Number Gen (RNG)
  NRF_RNG->EVENTS_VALRDY = 0;
  NRF_RNG->TASKS_START = 1;
  while (NRF_RNG->EVENTS_VALRDY == 0)
  {
    // wait for start
  }
  uint8_t rand1 = NRF_RNG->VALUE;
  NRF_RNG->TASKS_STOP = 1;

  Serial.print("lr1110 random ");
  Serial.println(rand1);


//    SPI.begin();

//---
//void Wm1110Hardware::reset()
// reset LR1110
//---
//ss high
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);

//nreset low 200us
//nreset high 200us
// delays 1ms here
//  reset();
//busy should go low and radio in standby

//    waitOnBusy();

///// --------------------------
// PA setup

// tx power setup
// ral_lr11xx_bsp_get_tx_cfg -->
// lr11xx_get_tx_cfg - radio_firmware_updater - platformio
// HP and LP PA enabled below 2.4G
//case LR11XX_WITH_LF_LP_HP_PA:

// 2.4G is an option, when lower freq, then set:
//  pa_type = LR11XX_WITH_LF_LP_HP_PA;  // = 2
    //output_params->pa_ramp_time = LR11XX_RADIO_RAMP_48_US;  // = 2






///// --------------------------
// tcxo
// uint32_t startup_time_ms = smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );   = return 30;
//   *xosc_cfg                = RAL_XOSC_CFG_TCXO_RADIO_CTRL;  // = 1
//    *supply_voltage          = LR11XX_SYSTEM_TCXO_CTRL_1_8V; // =  2
//    // tick is 30.52µs
//    *startup_time_in_tick = lr11xx_radio_convert_time_in_ms_to_rtc_step( startup_time_ms );
  // return ( uint32_t ) ( time_in_ms * LR11XX_RTC_FREQ_IN_HZ / 1000 );  
 // #define LR11XX_RTC_FREQ_IN_HZ 32768UL

#if 0
// tcxo notes
lr11xx_status_t lr11xx_system_set_tcxo_mode( const void* context, const lr11xx_system_tcxo_supply_voltage_t tune,
                                             const uint32_t timeout )
{
    const uint8_t cbuffer[LR11XX_SYSTEM_SET_TCXO_MODE_CMD_LENGTH] = {  // 6
        ( uint8_t ) ( LR11XX_SYSTEM_SET_TCXO_MODE_OC >> 8 ),   // 279 = 0x0117
        ( uint8_t ) ( LR11XX_SYSTEM_SET_TCXO_MODE_OC >> 0 ),
        ( uint8_t ) tune,
        ( uint8_t ) ( timeout >> 16 ),
        ( uint8_t ) ( timeout >> 8 ),
        ( uint8_t ) ( timeout >> 0 ),
    };
#endif
///// --------------------------


//  pinMode(_ss, OUTPUT);
//  digitalWrite(_ss, HIGH);

  Serial.println("==================");
  Serial.println("------------------");
  Serial.print("lr1110 _ss ");
  Serial.println(_ss);



#if 1
  #if BOARD_MODEL == BOARD_RNODE_NG_22 || BOARD_MODEL == BOARD_HELTEC_LORA32_V3 || BOARD_MODEL == BOARD_HELTEC_CAPSULE_V3 || BOARD_MODEL == BOARD_HELTEC_CAPSULE_V3 || BOARD_MODEL == BOARD_HELTEC_WIRELESS_PAPER_1_1
    SPI.begin(pin_sclk, pin_miso, pin_mosi, pin_cs);
  #else
    SPI.begin();
  #endif
#endif


// XXXXXXXXXXXXXXXXX  -- move these out so we can do after reset
#if 0

// SetDioAsRfSwitch
//C:\Users\cobra\AppData\Local\Arduino15\packages\Seeeduino\hardware\nrf52\1.1.8\libraries\LBM_WM1110\src\internal\lbm_hal\ral_lr11xx_bsp.c
// ral_lr11xx_bsp_get_rf_switch_cfg()
//uint8_t RfSwEn = 0b1111;  // enable  DIO10 - DIO9 - DIO8 - DIO7 - DIO6 - DIO5
uint8_t RfSwEn = 0b1111;  // enable  DIO10 - DIO8 - DIO7 - DIO6 - DIO5
// standby none
#if 0
// initial match to firmware / Wio tracker dev board
uint8_t rx_enable = 0b1;  // switch 0
uint8_t tx_enable = 0b11;  // switch 0 and 1
uint8_t tx_hp_enable = 0b10;  // switch 1
#else  // T1000-E 
uint8_t rx_enable = 0b1001; 
uint8_t tx_enable = 0b1011;  
uint8_t tx_hp_enable = 0b1010;  
#endif
#if 0
uint8_t gnss_enable = 0b100;  // switch 2
uint8_t wifi_enable = 0b1000;  // switch 3
#else
uint8_t gnss_enable = 0b0;  // switch 2 - internal GNSS
uint8_t wifi_enable = 0b0;  // switch 3 - Wifi scanner
#endif

uint8_t cmd5[13] = {10,0,0x1,0x12, 
  RfSwEn, 
  0, // none - standby

// 0x9, 0xb, 0xa ?  1001, 1011, 1010

  rx_enable,
  tx_enable,
  tx_hp_enable,
  0,  // unused
  gnss_enable,
  wifi_enable
  };

cmdTransfer("dio sw cfg", cmd5);


///// XXXXXXXX DISABLED

#if defined(BOARD_WIO_TRACK_1110)
// set tcxo mode
// tune = 2, 1.8V - wio tracker dev
// time 983 = 0x3d7
//uint8_t cmd4[10] = {6,0,0x1,0x17,2, 0, 0x3, 0xd7};
// 1.6V, longer delay
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x3, 0xf2};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)
uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x0, 0xa4};
#else // BOARD_WIO_T1000E 
// tune = 0 , 1.6V tracker t1000-e
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x3, 0xd7};
// longer delay
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x6, 0xd7};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)
uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x0, 0xa4};

#endif

cmdTransfer("tcxo", cmd4);



//setpackettype
uint8_t cmd[10] = {3,0,0x2,0xe,2};

cmdTransfer("packettype lora", cmd);

// geterrors
uint8_t cmd3[10] = {2,3, 1,0xd};

cmdTransfer("errors?", cmd3);


// setmodulationparams

//uint8_t cmd2[10] = {6,0, 0x02, 0x0f, 0x07 /*SF*/, 0x05 /*BW*/, 0x04 /*CR*/, 0 /*LDR*/};
uint8_t cmd2[10] = {6,0, 0x02, 0x0f, _sf /*SF*/, _bw /*BW*/, _cr /*CR*/, _ldro /*LDR*/};
cmdTransfer("modparams", cmd2);

// geterrors
//uint8_t cmd3[10] = {2,3, 1,0xd};

cmdTransfer("errors", cmd3);


// setpacketparams

// setPAconfig

// setTXParams


// XXXXXXXXXXXXXXXXX  -- move these out so we can do after reset
#endif



#if 0

// op  0x0106  read 32
// 0x0101 - ver register
  Serial.println(" preInit sx126x - ver = ");
  //Serial.println(A32Transfer(0x0106, 0x0101, 0));
  A32Transfer(0x0101, 0x0101, 0);
  //Serial.print(" lsb ");
  //Serial.println(readRegister(0x102));
  //A32Transfer(0x0101, 0x0101, 0);

  // check version (retry for up to 2 seconds)
  // TODO: Actually read version registers, not syncwords
  long start = millis();
  uint8_t syncmsb;
  uint8_t synclsb;
  while (((millis() - start) < 2000) && (millis() >= start)) {
      syncmsb = readRegister(REG_SYNC_WORD_MSB_6X);
      synclsb = readRegister(REG_SYNC_WORD_LSB_6X);
      if ( uint16_t(syncmsb << 8 | synclsb) == 0x1424 || uint16_t(syncmsb << 8 | synclsb) == 0x4434) {
          break;
      }
  Serial.print("sx126x - syncmsb ");
  Serial.print(syncmsb);
  Serial.print(" syncmlsb ");
  Serial.println(synclsb);
      delay(100);
  }

// op  0x0106  read 32
// 0x0101 - ver register
  Serial.print("sx126x - ver msb ");
  Serial.print(A32Transfer(0x0106, 0x101, 0));
  Serial.print(" lsb ");
  Serial.println(readRegister(0x102));


  if ( uint16_t(syncmsb << 8 | synclsb) != 0x1424 && uint16_t(syncmsb << 8 | synclsb) != 0x4434) {
      return false;
  }

#endif

  _preinit_done = true;
  return true;
}


void lr11xx::doSetup(void)
{

Serial.println("Doing Setup  - dio  DCDC  - TCXO     >>>>>>>>>>>>");


#if defined(BOARD_WIO_TRACK_1110)
// set tcxo mode
// tune = 2, 1.8V - wio tracker dev
// time 983 = 0x3d7
////uint8_t cmd4[10] = {6,0,0x1,0x17,2, 0, 0x3, 0xd7};
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x3, 0xd7};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x0, 0xa4};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)  - 1.8V
uint8_t cmd4[10] = {6,0,0x1,0x17,2, 0, 0x0, 0xa4};
#else // BOARD_WIO_T1000E 
// tune = 0 , 1.6V tracker t1000-e
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x3, 0xd7};
// test with 0xa4 delay
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x0, 0xa4};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x0, 0xa4};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)  - 1.8V
uint8_t cmd4[10] = {6,0,0x1,0x17,2, 0, 0x0, 0xa4};
#endif

cmdTransfer("tcxo", cmd4);


#if 0
// skip, has TCXO

// Config LF clk
// ConfigLfClock
// external DIO11
//uint8_t cmd8[6] = {3,0,0x1,0x16, 0b10};
// LF crystal, wait
uint8_t cmd8[6] = {3,0,0x1,0x16, 0b101};
// LF RC, no wait
//uint8_t cmd8[6] = {3,0,0x1,0x16, 0b0};
cmdTransfer("setRegMode", cmd8);
#endif

// SetRegMode
// enable DC-DC for TX and when hi accuracy is needed
uint8_t cmd7[6] = {3,0,0x1,0x10, 1};
cmdTransfer("setRegMode", cmd7);


// SetDioAsRfSwitch
//C:\Users\cobra\AppData\Local\Arduino15\packages\Seeeduino\hardware\nrf52\1.1.8\libraries\LBM_WM1110\src\internal\lbm_hal\ral_lr11xx_bsp.c
// ral_lr11xx_bsp_get_rf_switch_cfg()
uint8_t RfSwEn = 0b1111;  // enable  DIO10 - DIO9 - DIO8 - DIO7 - DIO6 - DIO5
// standby none

// wio tracker dev
#if defined(BOARD_WIO_TRACK_1110)
uint8_t rx_enable = 0b1;  // switch 0
uint8_t tx_enable = 0b11;  // switch 0 and 1
uint8_t tx_hp_enable = 0b10;  // switch 1
#else // BOARD_WIO_T1000E 
uint8_t rx_enable = 0b1001; 
uint8_t tx_enable = 0b1011;  
uint8_t tx_hp_enable = 0b1010;  
#endif
#if 0
uint8_t gnss_enable = 0b100;  // switch 2
uint8_t wifi_enable = 0b1000;  // switch 3
#else
uint8_t gnss_enable = 0b0;  // switch 2 - internal GNSS
uint8_t wifi_enable = 0b0;  // switch 3 - Wifi scanner
#endif


uint8_t cmd5[12] = {10,0,0x1,0x12, 
  RfSwEn, 
  0, // none - standby
  rx_enable,
  tx_enable,
  tx_hp_enable,
  0,  // unused
  gnss_enable,
  wifi_enable
  };

cmdTransfer("dio sw cfg", cmd5);

#if 0
//Move these up

// Config LF clk
// ConfigLfClock
// external DIO11
//uint8_t cmd8[6] = {3,0,0x1,0x16, 0b10};
// LF crystal, wait
//uint8_t cmd8[6] = {3,0,0x1,0x16, 0b101};
// LF RC, no wait
uint8_t cmd8[6] = {3,0,0x1,0x16, 0b0};
cmdTransfer("setRegMode", cmd8);


// SetRegMode
// enable DC-DC for TX and when hi accuracy is needed
uint8_t cmd7[6] = {3,0,0x1,0x10, 1};
cmdTransfer("setRegMode", cmd7);
#endif

// ConfigLfClock XXX not used with TCXO XXXX
// use crystal, wait for ready
//uint8_t cmd8[6] = {3,0,0x1,0x16, 0b101};
//cmdTransfer("LfClk", cmd8);

#if 0
#if defined(BOARD_WIO_TRACK_1110)
// set tcxo mode
// tune = 2, 1.8V - wio tracker dev
// time 983 = 0x3d7
////uint8_t cmd4[10] = {6,0,0x1,0x17,2, 0, 0x3, 0xd7};
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x3, 0xd7};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x0, 0xa4};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)  - 1.8V
uint8_t cmd4[10] = {6,0,0x1,0x17,2, 0, 0x0, 0xa4};
#else // BOARD_WIO_T1000E 
// tune = 0 , 1.6V tracker t1000-e
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x3, 0xd7};
// test with 0xa4 delay
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x0, 0xa4};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)
//uint8_t cmd4[10] = {6,0,0x1,0x17,0, 0, 0x0, 0xa4};
// shorter 5000uS (RadioLib default) 0xa4  (5000 / 30.52)  - 1.8V
uint8_t cmd4[10] = {6,0,0x1,0x17,2, 0, 0x0, 0xa4};
#endif

cmdTransfer("tcxo", cmd4);
#endif


//setpackettype
uint8_t cmd[10] = {3,0,0x2,0xe,2};

cmdTransfer("packettype lora", cmd);

// geterrors
uint8_t cmd3[10] = {2,3, 1,0xd};

cmdTransfer("errors?", cmd3);

// SyncWord / Private network (should be 0x12)
//uint8_t cmd6[5] = {3,0, 0x2,0x8, 0x0};
// public - should be 0x34
//uint8_t cmd6[5] = {3,0, 0x2,0x8, 0x1};
//cmdTransfer("PrvtNet", cmd6);
// SetLoRaSyncWord
//uint8_t cmd6[5] = {3,0, 0x2,0x2b, 0x14};
uint8_t cmd6[5] = {3,0, 0x2,0x2b, 0x12};
cmdTransfer("SyncWord", cmd6);

// SetRssiCalibration
// EVK defaults 600MHz - 2GHz
uint8_t cmd9[16] = {13,0, 0x2,0x29, 0x22, 0x32, 0x43, 0x45, 0x64,
    0x55, 0x66, 0x76, 0x6, 0x0, 0x0 };
cmdTransfer("SetRssiCal", cmd9);

#if 0
// setmodulationparams

//uint8_t cmd2[10] = {6,0, 0x02, 0x0f, 0x07 /*SF*/, 0x05 /*BW*/, 0x04 /*CR*/, 0 /*LDR*/};
uint8_t cmd2[10] = {6,0, 0x02, 0x0f, _sf /*SF*/, _bw /*BW*/, _cr /*CR*/, _ldro /*LDR*/};
cmdTransfer("modparams", cmd2);
#endif


// geterrors
//uint8_t cmd3[10] = {2,3, 1,0xd};

//cmdTransfer("errors", cmd3);


// setpacketparams

// setPAconfig

// setTXParams

}


uint8_t ISR_VECT lr11xx::readRegister(uint16_t address)
{
  return singleTransfer(OP_READ_REGISTER_6X, address, 0x00);
}

void lr11xx::writeRegister(uint16_t address, uint8_t value)
{
    singleTransfer(OP_WRITE_REGISTER_6X, address, value);
}

uint8_t ISR_VECT lr11xx::singleTransfer(uint8_t opcode, uint16_t address, uint8_t value)
{
    waitOnBusy();

    uint8_t response;

    digitalWrite(_ss, LOW);

    SPI.beginTransaction(_spiSettings);
    SPI.transfer(opcode);
    SPI.transfer((address & 0xFF00) >> 8);
    SPI.transfer(address & 0x00FF);
    if (opcode == OP_READ_REGISTER_6X) {
        SPI.transfer(0x00);
    }
    response = SPI.transfer(value);
    SPI.endTransaction();

    digitalWrite(_ss, HIGH);

    return response;
}



//int lr11xx::decode_stat(uint8_t stat, uint8_t print_it)
int lr11xx::decode_stat(uint8_t stat)
{
  int doErr = 0;

  Serial.print((stat & 1) ? "  Int - " : "  ___ - ");
  switch ((stat>>1) & 7)
  {
    case 0:
      Serial.println("CMD_FAIL");
      doErr = 1;
      break;
    case 1:
      Serial.println("CMD_PERR");
      break;
    case 2:
      Serial.println("CMD_OK");
      break;
    case 3:
      Serial.println("CMD_DAT");
      break;
  }
//  Serial.print(" - Int ");
//  Serial.println(stat & 1);

#if 0
  if(doErr) {
    Serial.println("get errors");

    uint8_t cmd2[8] = {5,0, 0x01, 0x0d, 0x0, 0x0,0x0};
    cmdTransfer("modparams", cmd2);
    Serial.print("Fail errors ");
    Serial.print(cmd2[5] & 0x1);
    Serial.println(cmd2[6]);

  }
#endif
  return doErr;
}

int lr11xx::decode_stat(const char *prt,uint8_t stat1,uint8_t stat2,uint8_t print_it)
{
  int doErr = 0;

  //if (!print_it) return 0;
  

//  Serial.print((stat & 1) ? "  Int - " : "  ___ - ");
  switch ((stat1>>1) & 7)
  {
    case 0:
      Serial.print(prt);
      Serial.println("        ---- > CMD_FAIL");
      doErr = 1;
      break;
    case 1:
      Serial.println("        ---- > CMD_PERR");
      break;
    case 2:
      if(print_it) Serial.println("CMD_OK");
      break;
    case 3:
      if(print_it) Serial.println("        ---- > CMD_DAT");
      break;
  }

  return doErr;

}


int lr11xx::decode_stat(uint8_t stat, uint8_t stat2)
{
  int cmd_fail = decode_stat(stat, 1);
  switch ((stat2 & 0xf0)>>4)
  {
    case 0:
      Serial.print("RST CLR");
      break;
    case 1:
      Serial.print("RST A-Anlg");
      break;
    case 2:
      Serial.print("RST A-Rpin");
      break;
    case 3:
      Serial.print("RST A-Sys");
      break;
    case 4:
      Serial.print("RST A-WDog");
      break;
    case 5:
      Serial.print("RST A-IOCD rstrt");
      break;
    case 6:
      Serial.print("RST A-RTC");
      break;
    default:
      Serial.print("RST ACT");
      break;
  }
  switch ((stat2>>1) & 7)
  {
    case 0:
      Serial.print("  Sleep");
      break;
    case 1:
      Serial.print("  STD RC");
      break;
    case 2:
      Serial.print("  STD Xtc");
      break;
    case 3:
      Serial.print("  FS");
      break;
    case 4:
      Serial.print("  RX");
      break;
    case 5:
      Serial.print("  TX");
      break;
    case 6:
      Serial.print("  RDO");
      break;
  }
  switch (stat2 & 1)
  {
    case 0:
      Serial.println("  bload");
      break;
    case 1:
      Serial.println("  flash");
      break;
  }
  return cmd_fail;
}


uint16_t ISR_VECT lr11xx::cmdTransfer(const char *prt, uint8_t *cmd)
{
  // disable prints with 0! 
  return cmdTransfer(prt, cmd, 0);
}


uint16_t ISR_VECT lr11xx::cmdTransfer(const char *prt, uint8_t *cmd, bool print_it)
{
  uint8_t cnt = cmd[0];
  uint8_t cnt_in = cmd[1];
  uint8_t response;
  uint8_t stat1,stat2;
  uint8_t cmd_fail;

  //print_it=0;

  if(print_it) {
    Serial.print("go cmd -");
    Serial.println(prt);
  }

    waitOnBusy();
    digitalWrite(_ss, LOW);
    SPI.beginTransaction(_spiSettings);

  for (int x=0; x<cnt; x++)
  {
    response = SPI.transfer(cmd[2+x]);
    if (print_it) Serial.println(response);
    cmd[2+x] = response;
    if(!x)
      stat1=response;
    // decode stat1, stat2
    if(x==1)
      if (print_it) cmd_fail = decode_stat(stat1,response);
  }
  decode_stat(prt,stat1,response,print_it);

  for (int x=0; x<cnt_in; x++)
  {
    response = SPI.transfer(0);
    if (print_it) {Serial.print(response); Serial.print(" "); }
    cmd[2+cnt+x] = response;
    // noooo
    //if(!x)
    //  if (print_it) decode_stat(response);
  }
    SPI.endTransaction();
    digitalWrite(_ss, HIGH);

    if (print_it) Serial.println(" done\n----");


  return 0;
}

uint16_t ISR_VECT lr11xx::A32Transfer(uint16_t opcode, uint16_t address, uint16_t value)
{
    waitOnBusy();

    uint8_t response;
    uint16_t response_msb;
    uint16_t response_lsb;

    digitalWrite(_ss, LOW);

    Serial.println("go");
    SPI.beginTransaction(_spiSettings);
    Serial.println(SPI.transfer((opcode & 0xFF00) >> 8));
    Serial.println(SPI.transfer(opcode & 0x00FF));
    //SPI.transfer((address & 0xFF00) >> 8);
    //SPI.transfer(address & 0x00FF);
    //if (opcode == OP_READ_REGISTER_6X) {
    //    SPI.transfer(0x00);
    //}
    ////SPI.transfer((value & 0xFF00) >> 8);
    //SPI.transfer(value & 0x00FF);
    //response_msb = SPI.transfer(value);
    //response_lsb = SPI.transfer(value);
    response = SPI.transfer(0);
    Serial.println(response);
    if (response & 0x4) {
    Serial.println(SPI.transfer(0));
    Serial.println(SPI.transfer(0));
    Serial.println(SPI.transfer(0));
    Serial.println(SPI.transfer(0));
    }
    SPI.endTransaction();
    Serial.println("end");

    digitalWrite(_ss, HIGH);

    return (response_msb << 8) | response_lsb;
}

void lr11xx::rxAntEnable()
{
  if (_rxen != -1) {
    digitalWrite(_rxen, HIGH);
  }
}

void lr11xx::loraMode() {
    // enable lora mode on the SX1262 chip
    uint8_t mode = MODE_LONG_RANGE_MODE_6X;
    executeOpcode(OP_PACKET_TYPE_6X, &mode, 1);
}

void lr11xx::waitOnBusy() {
    unsigned long time = millis();
    long count = 0;
    if (_busy != -1) {
        while (digitalRead(_busy) == HIGH)
        {
          count++;
            if (millis() >= (time + 300)) {
              Serial.println("waitonbusy timeout!!");
                break;
            }
            // do nothing
        }
        //Serial.println("busy = ");
        //Serial.println(count);
    }
}

void lr11xx::executeOpcode(uint8_t opcode, uint8_t *buffer, uint8_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    SPI.beginTransaction(_spiSettings);
    SPI.transfer(opcode);

    for (int i = 0; i < size; i++)
    {
        SPI.transfer(buffer[i]);
    }

    SPI.endTransaction();

    digitalWrite(_ss, HIGH);
}

void lr11xx::executeOpcodeRead(uint8_t opcode, uint8_t *buffer, uint8_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    SPI.beginTransaction(_spiSettings);
    SPI.transfer(opcode);
    SPI.transfer(0x00);

    for (int i = 0; i < size; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }

    SPI.endTransaction();

    digitalWrite(_ss, HIGH);
}

// todo
void lr11xx::writeBuffer(const uint8_t* buffer, size_t size)
{

  uint8_t cmd[4] = {2,0,0x1, 0x9 };
  //uint8_t cnt = cmd[0];
  //uint8_t cnt_in = cmd[1];
  uint8_t response;

  //Serial.println("go cmd - writeBuf");

  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);

  for (int x=0; x<2; x++)
  {
    response = SPI.transfer(cmd[2+x]);
    //Serial.println(response);
    //if(!x)
    //  decode_stat(response);
  }
  // write data (TX)
  for (int x=0; x<size; x++)
  {
    response = SPI.transfer(buffer[x]);
  }

  SPI.endTransaction();

  digitalWrite(_ss, HIGH);


#if 0
  Serial.println("WWWWWW\nwrite\n      WWWWWWW");
    waitOnBusy();

    digitalWrite(_ss, LOW);

    SPI.beginTransaction(_spiSettings);
    SPI.transfer(OP_FIFO_WRITE_6X);
    SPI.transfer(_fifo_tx_addr_ptr);

    for (int i = 0; i < size; i++)
    {
        SPI.transfer(buffer[i]);
        _fifo_tx_addr_ptr++;
    }

    SPI.endTransaction();

    digitalWrite(_ss, HIGH);
#endif
}

// lr1110
void lr11xx::readBuffer(uint8_t* buffer, size_t size)
{
  uint8_t cmd[7] = {4,0,0x1, 0xa, _fifo_rx_addr_ptr, size};
  uint8_t cnt = cmd[0];
  //uint8_t cnt_in = cmd[1];
  uint8_t response;

    Serial.println("go cmd - readBuf");
    //Serial.println(prt);

    waitOnBusy();
    digitalWrite(_ss, LOW);
    SPI.beginTransaction(_spiSettings);

  for (int x=0; x<cnt; x++)
  {
    response = SPI.transfer(cmd[2+x]);
    Serial.println(response);
    if(!x)
      decode_stat(response);
  }
    response = SPI.transfer(0);
    decode_stat(response);
  for (int x=0; x<size; x++)
  {
    response = SPI.transfer(0);
    Serial.print(response);
    buffer[x] = response;
  }
    SPI.endTransaction();
    digitalWrite(_ss, HIGH);

    Serial.println("done\n----");


  #if 0
    waitOnBusy();

    digitalWrite(_ss, LOW);

    SPI.beginTransaction(_spiSettings);
    SPI.transfer(OP_FIFO_READ_6X);
    SPI.transfer(_fifo_rx_addr_ptr);
    SPI.transfer(0x00);

    for (int i = 0; i < size; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }

    SPI.endTransaction();

    digitalWrite(_ss, HIGH);
  #endif
}

// lr1110
void lr11xx::setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, int ldro) {

if(modulationDirty==0) return;
modulationDirty = 0;


#if 0
  uint8_t buf[8];


  buf[0] = sf;
  buf[1] = bw;
  buf[2] = cr; 
  // low data rate toggle
  buf[3] = ldro;
  // unused params in LoRa mode
  buf[4] = 0x00; 
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
#endif

  Serial.print("Set Modulation params - ");

  uint8_t cmd2[10] = {6,0, 0x02, 0x0f, sf /*SF*/, bw /*BW*/, cr /*CR*/, ldro /*LDR*/};
  Serial.print("sf ");
  Serial.print(cmd2[4]);
  Serial.print(" bw ");
  Serial.print(cmd2[5]);
  Serial.print(" cr ");
  Serial.print(cmd2[6]);
  Serial.print(" ldro ");
  Serial.println(cmd2[7]);

  cmdTransfer("modparams", cmd2);

//  executeOpcode(OP_MODULATION_PARAMS_6X, buf, 8);

}

void lr11xx::getErrors(void)
{
  // geterrors
  uint8_t cmd3[10] = {2,3, 1,0xd};

  cmdTransfer("err cmd", cmd3);

  Serial.print("====         err dat EEEEEE ");
  Serial.print(cmd3[4]);
  Serial.print(" ");
  Serial.println(cmd3[5]);

  uint8_t cmd2[5] = {2,0, 1,0xe};

  cmdTransfer("err clr", cmd2);

}

// lr1110 done
void lr11xx::setPacketParams(long preamble, uint8_t headermode, uint8_t length, uint8_t crc) {

  if(packetParamsDirty==0) return;
  packetParamsDirty=0;

  //setpackettype
  uint8_t cmd[10] = {3,0,0x2,0xe,2};

  cmdTransfer("packettype lora", cmd);

  // packet params
  uint8_t buf[10] = {8,0, 0x02, 0x10 };

  buf[4] = uint8_t((preamble & 0xFF00) >> 8);
  buf[5] = uint8_t((preamble & 0x00FF));
  buf[6] = headermode;
  buf[7] = length;
  buf[8] = crc;
  // standard IQ setting (no inversion)
  buf[9] = 0x00; 

#if 1
  Serial.print("Packet params  - preamble ");
  Serial.print(preamble);
  Serial.print(" headermode ");
  Serial.print(headermode);
  Serial.print(" len ");
  Serial.print(length);
  Serial.print(" crc  ");
  Serial.println(crc);
  //Serial.println("\n....");
#endif

  cmdTransfer("packet params", buf);

  // test
  //getErrors();

#if 0
  buf[0] = uint8_t((preamble & 0xFF00) >> 8);
  buf[1] = uint8_t((preamble & 0x00FF));
  buf[2] = headermode;
  buf[3] = length;
  buf[4] = crc;
  // standard IQ setting (no inversion)
  buf[5] = 0x00; 
  // unused params
  buf[6] = 0x00; 
  buf[7] = 0x00; 
  buf[8] = 0x00; 

  executeOpcode(OP_PACKET_PARAMS_6X, buf, 9);
#endif
  

}

void lr11xx::reset(void) {
  if (_reset != -1) {
    pinMode(_reset, OUTPUT);

    // perform reset
    digitalWrite(_reset, LOW);
    delay(2);
    digitalWrite(_reset, HIGH);
    delay(2);

    waitOnBusy();

    /// Clear reset status
    // GetStatus, last 4 are irq status on return
    uint8_t cmd[9] = {6,0,0x1,0x0, 0x0,0x0,0x0,0x0};

    cmdTransfer("->getStat reset", cmd);

    uint8_t cmd2[9] = {6,0,0x1,0x0, 0x0,0x0,0x0,0x0};

    cmdTransfer("->getStat reset2", cmd2);

    // testtest
    // kills the board during RNode start
//    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
//    NRF_CLOCK->TASKS_HFCLKSTART = 1;
//    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
//    {
 //     // wait for start
 //   }

  }
}

// lr1110
void lr11xx::calibrate(uint8_t cal) {

  Serial.println("XXXXXXXXXXXXX  Calibrate            XXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  // all
  //uint8_t buf[7] = {3,0, 0x01, 0xf, 0b111111 };
  //uint8_t buf[7] = {3,0, 0x01, 0xf, 0b1010 };

  //uint8_t buf[7] = {3,0, 0x01, 0xf, 0b100111 };
  //uint8_t buf[7] = {3,0, 0x01, 0xf, 0xff };
  uint8_t buf[7] = {3,0, 0x01, 0xf, cal };

  cmdTransfer("calib", buf);

}

// lr1110
void lr11xx::calibrate_image(long frequency) {
  uint8_t image_freq[2] = {0};
  uint8_t freq_error = 0;

  Serial.println("XXXXXXXXXXXXX  Calibrate IMG            XXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  
  // lr1110
  if (frequency >= 430E6 && frequency <= 440E6) {
    image_freq[0] = 0x6B;
    image_freq[1] = 0x6E;
  }
  else if (frequency >= 470E6 && frequency <= 510E6) {
    image_freq[0] = 0x75;
    image_freq[1] = 0x81;
  }
  else if (frequency >= 779E6 && frequency <= 787E6) {
    image_freq[0] = 0xC1;
    image_freq[1] = 0xC5;
  }
  else if (frequency >= 863E6 && frequency <= 870E6) {
    image_freq[0] = 0xD7;
    image_freq[1] = 0xDB;
  }
  else if (frequency >= 902E6 && frequency <= 928E6) {
    image_freq[0] = 0xE1;
    image_freq[1] = 0xE9;
  } else {
    freq_error = 1;
  }

  if(!freq_error) {
    uint8_t buf[7] = {4,0, 0x01, 0x11, image_freq[0], image_freq[1] };

    cmdTransfer("calib img", buf);

    Serial.print("freq ");
    Serial.print(frequency);
    Serial.print(" ");
    Serial.print(image_freq[0]);
    Serial.print(" ");
    Serial.println(image_freq[1]);
  }
}

void lr11xx::getErrors2(void)
{

    Serial.println("get errors2");
    uint8_t cmd2[8] = {5,0, 0x01, 0x0d, 0x0, 0x0,0x0};
    cmdTransfer("getErr", cmd2);
    Serial.print("Fail errors ");
    Serial.print(cmd2[5]);
    Serial.print(" (");
    Serial.print(cmd2[5] & 0x1);
    Serial.print("), ");
    Serial.println(cmd2[6]);

}



int lr11xx::begin(long frequency)
{

Serial.print("\n Lora begin lr1110\n - - RESET - -  XXXXXXXXXXXXXXXXXXXXXXX");
  
  if (_busy != -1) {
      pinMode(_busy, INPUT);
  }

  reset();

#if 0
  if (_busy != -1) {
      pinMode(_busy, INPUT);
  }
#endif

/// reset clears chip, redo preInit?
// No, resets bluetooth?
#if 0
    if (!preInit()) {
      return false;
    }
#else 
  if (!_preinit_done) {
    Serial.println("lr1110 preinit ");
    if (!preInit()) {
      return false;
    }
  }
#endif

  if (_rxen != -1) {
      pinMode(_rxen, OUTPUT);
  }

  //calibrate(0b1101);
  //calibrate(0b111111);

  doSetup();

  //Serial.println("lr1110 en tcxo ");
  //enableTCXO();

  // todo 
  //loraMode();
  //standby();

// todo
  // Set sync word
  //setSyncWord(SYNC_WORD_6X);

// No for LR1110?
//  #if DIO2_AS_RF_SWITCH
//    // enable dio2 rf switch
//    uint8_t byte = 0x01;
//    executeOpcode(OP_DIO2_RF_CTRL_6X, &byte, 1);
//  #endif

  Serial.print(" .rxAnt. ");
  rxAntEnable();

  Serial.print(" .freq. ");
  setFrequency(frequency);

  #if 1
    getErrors2();
  #else
    Serial.println("get errors");
    uint8_t cmd2[8] = {5,0, 0x01, 0x0d, 0x0, 0x0,0x0};
    cmdTransfer("getErr", cmd2);
    Serial.print("Fail errors ");
    Serial.print(cmd2[5]);
    Serial.print(" (");
    Serial.print(cmd2[5] & 0x1);
    Serial.print("), ");
    Serial.println(cmd2[6]);
  #endif

  /// todo - startup calibrates at 915MHz
  /// if freq is changed, need to CalibImage
  /// if temp change, need to calibrate
///  Serial.println("lr1110 calibrate img");
///  calibrate_image(frequency);
  Serial.println("lr1110 calibrate");
  //calibrate(0b100010);
  calibrate(0b111111);

  #if 1
    getErrors2();
  #else
    Serial.println("get errors");
    uint8_t cmd3[8] = {5,0, 0x01, 0x0d, 0x0, 0x0,0x0};
    cmdTransfer("getErr", cmd3);
    Serial.print("Fail errors ");
    Serial.print(cmd3[5] & 0x1);
    Serial.print(", ");
    Serial.println(cmd3[6]);
  #endif

  // set output power to 2 dBm
  Serial.print(" .Pow. ");
  //setTxPower(2);
  // tested for a while
  //setTxPower(6);
  //setTxPower(10);

  setTxPower(0);

  Serial.print(" .CRC. ");
  enableCrc();

  // todo
  // set LNA boost
  //writeRegister(REG_LNA_6X, 0x96);

  // todo
  // set base addresses
  //uint8_t basebuf[2] = {0};
  //executeOpcode(OP_BUFFER_BASE_ADDR_6X, basebuf, 2);

  setModulationParams(_sf, _bw, _cr, _ldro);
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

Serial.println(".... done ");

  return 1;
}

void lr11xx::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  SPI.end();

  _preinit_done = false;
}

int lr11xx::beginPacket(int implicitHeader)
{
      Serial.println("bbbbb\nbeginpacket\n     bbbbbbb");
  standby();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  _payloadLength = 0;
  packetParamsDirty=1;
  _fifo_tx_addr_ptr = 0;
  // set in endPacket()
  //setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  return 1;
}

// lr1110
int lr11xx::endPacket()
{
      setModulationParams(_sf, _bw, _cr, _ldro);
      setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

      //setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  #if 0
      // put in single TX mode
      uint8_t timeout[3] = {0};
      executeOpcode(OP_TX_6X, timeout, 3);
  #endif

      //Serial.print("start TX sz ");
      //Serial.println(_payloadLength);
      //yield();

      // put in single TX mode
      // SetTX
      uint8_t cmd2[8] = {5,0,0x2,0xa,0x0,0x0,0x0};

      cmdTransfer("->tx", cmd2);

#if 0
      uint8_t buf[2];

      buf[0] = 0x00;
      buf[1] = 0x00;

      executeOpcodeRead(OP_GET_IRQ_STATUS_6X, buf, 2);

      // wait for TX done
      while ((buf[1] & IRQ_TX_DONE_MASK_6X) == 0) {
        buf[0] = 0x00;
        buf[1] = 0x00;
        executeOpcodeRead(OP_GET_IRQ_STATUS_6X, buf, 2);
        yield();
      }
#endif

// sx1262 method, with lr1110 loop read of GetStatus
      // GetStatus, last 4 are irq status on return
      uint8_t cmd[9] = {6,0,0x1,0x0, 0x0,0x0,0x0,0x0};
#if 0

      cmdTransfer("->getStat e1", cmd);
      // Wait TXDone
      while ((cmd[7] & 0b100) == 0) {
        //cmd = {6,0,0x1,0x0, 0x0,0x0,0x0,0x0};
        cmd[2]=0x1; cmd[3]=0; cmd[4]=0; cmd[5]=0; cmd[6]=0; cmd[7]=0;
        cmdTransfer("->getStat e2", cmd, 0);
        yield();
      }
      //Serial.println("TXDone");
      //yield();
#endif

#if 0
    // try NOP, read only version
      // NOPs, last 4 are irq status on return
      uint8_t cmd[9] = {6,0,0x0,0x0, 0x0,0x0,0x0,0x0};

      cmdTransfer("->NOP stat e1", cmd);
      // Wait TXDone
      while ((cmd[7] & 0b100) == 0) {
        //cmd = {6,0,0x1,0x0, 0x0,0x0,0x0,0x0};
        cmd[2]=0x0; cmd[3]=0; cmd[4]=0; cmd[5]=0; cmd[6]=0; cmd[7]=0;
        cmdTransfer("->NOP stat e2", cmd, 0);
        yield();
      }
#endif

      // Rely on INT flag setting
      while (!TXCompl_flag) {
        yield();
      }
      TXCompl_flag=0;


#if 0
      Serial.print("endpacket tx- ");
      Serial.print(cmd[2]);
      Serial.print(" ");
      Serial.print(cmd[3]);
      Serial.print(" - ");
      Serial.print(cmd[4]);
      Serial.print(" ");
      Serial.print(cmd[5]);
      Serial.print(" ");
      Serial.print(cmd[6]);
      Serial.print(" ");
      Serial.println(cmd[7]);

      getErrors();
#endif

      // clear IRQ's
#if 0
      uint8_t mask[2];
      mask[0] = 0x00;
      mask[1] = IRQ_TX_DONE_MASK_6X;
      executeOpcode(OP_CLEAR_IRQ_STATUS_6X, mask, 2);
#endif

      // Clear and read, clr TXDone
      uint8_t cmd3[9] = {6,0,0x1,0x14,0x0,0x0,0x0,0b100};

      cmdTransfer("->clrIrq TX", cmd3);


        cmd[2]=0x1; cmd[3]=0; cmd[4]=0; cmd[5]=0; cmd[6]=0; cmd[7]=0;
        cmdTransfer("->getStat after clr", cmd, 0);

      Serial.print("endpacket tx- ");
      Serial.print(cmd[2]);
      Serial.print(" ");
      Serial.print(cmd[3]);
      Serial.print(" - ");
      Serial.print(cmd[4]);
      Serial.print(" ");
      Serial.print(cmd[5]);
      Serial.print(" ");
      Serial.print(cmd[6]);
      Serial.print(" ");
      Serial.println(cmd[7]);

      getErrors();


      Serial.println("eeeeeee\nendpacket\n     eeeeeee");
  return 1;
}

// lr1110 
// RNode main firmware expects the following status bits
	// Status flags
//	const uint8_t SIG_DETECT = 0x01;
//	const uint8_t SIG_SYNCED = 0x02;
//	const uint8_t RX_ONGOING = 0x04;
uint8_t lr11xx::modemStatus() {

    // GetStatus, last 4 are irq status on return
    uint8_t cmd[9] = {6,0,0x1,0x0, 0x0,0x0,0x0,0x0};
    uint8_t byte = 0x00;
    uint8_t toClr = 0x00;

    cmdTransfer("->Modem-IrqStat", cmd, 0);
    // Preamble detected
    if(cmd[7] & 0b10000) {
      byte = byte | 0x01 | 0x04;
      toClr = 0b10000;
    }
    // Header Valid
    if(cmd[7] & 0b100000) {
      byte = byte | 0x02 | 0x04;
      toClr |= 0b100000;
    }

    // clear active IRQs
    //uint8_t cmd2[9] = {6,0,0x1,0x14, 0x0,0x0,0x0,0b110000};
    if(toClr) {
      uint8_t cmd2[9] = {6,0,0x1,0x14, 0x0,0x0,0x0,toClr};
      cmdTransfer("->Clr Act IRQs", cmd2);
    } else {
      //Serial.println("Clr Irq -NA");
    }

    return byte; 

#if 0
    Serial.println("XXXX\n XXXXX modemStatus\n -----------------------");
    // imitate the register status from the sx1276 / 78
    uint8_t buf[2] = {0};

    executeOpcodeRead(OP_GET_IRQ_STATUS_6X, buf, 2);
    uint8_t clearbuf[2] = {0};
    uint8_t byte = 0x00;

    if ((buf[1] & IRQ_PREAMBLE_DET_MASK_6X) != 0) {
      byte = byte | 0x01 | 0x04;
      // clear register after reading
      clearbuf[1] = IRQ_PREAMBLE_DET_MASK_6X;
    }

    if ((buf[1] & IRQ_HEADER_DET_MASK_6X) != 0) {
      byte = byte | 0x02 | 0x04;
    }

    executeOpcode(OP_CLEAR_IRQ_STATUS_6X, clearbuf, 2);

    return byte; 
#endif
}

// lr1110
uint8_t lr11xx::currentRssiRaw() {
    // GetRssiInst
    uint8_t cmd[7] = {4,0,0x2,0x5, 0x0,0x0};

    cmdTransfer("->Modem-GetRssi", cmd, 0);

    return cmd[5];
}

// lr1110
int ISR_VECT lr11xx::currentRssi() {

    uint8_t byte;
    byte = currentRssiRaw();
    int rssi = -(int(byte)) / 2;
    return rssi;

#if 0
    uint8_t byte = 0;
    executeOpcodeRead(OP_CURRENT_RSSI_6X, &byte, 1);
    int rssi = -(int(byte)) / 2;
    return rssi;
#endif
}

// unused
uint8_t lr11xx::packetRssiRaw() {
    uint8_t buf[3] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_6X, buf, 3);
    return buf[2];
}

// used
int ISR_VECT lr11xx::packetRssi() {
    Serial.println("    ------    packetRssi ----------   XXXXXXXXX TODO");
    #if 0
    // may need more calculations here
    uint8_t buf[3] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_6X, buf, 3);
    int pkt_rssi = -buf[0] / 2;
    return pkt_rssi;
    #else
    return 0;
    #endif
}

// used
uint8_t ISR_VECT lr11xx::packetSnrRaw() {
    Serial.println("    ------    packetSnrRaw ----------   XXXXXXXXX  TODO");
    #if 0
    uint8_t buf[3] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_6X, buf, 3);
    return buf[1];
    #else
    return 0;
    #endif
}

// unused
float ISR_VECT lr11xx::packetSnr() {
    uint8_t buf[3] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_6X, buf, 3);
    return float(buf[1]) * 0.25;
}

long lr11xx::packetFrequencyError()
{
    // todo: implement this, no idea how to check it on the sx1262
    const float fError = 0.0;
    return static_cast<long>(fError);
}

size_t lr11xx::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t lr11xx::write(const uint8_t *buffer, size_t size)
{
    if ((_payloadLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - _payloadLength;
    }

    // write data
    writeBuffer(buffer, size);
    _payloadLength = _payloadLength + size;
    packetParamsDirty=1;
    return size;
}

// lr1110
int ISR_VECT lr11xx::available() {
  uint8_t size, rxbufstart;

  return available(&size, &rxbufstart);

}

// lr1110
int ISR_VECT lr11xx::available(uint8_t *size, uint8_t *rxbufstart)
{

    // Rx buffer status
    uint8_t cmd[8] = {2,3,0x2,0x3,0x0,0x0,0x0};

    cmdTransfer("->rx buf stat", cmd);
    // returns stat1, PayloadLengthRX, RxStartBufferPointer

    *size = cmd[5];
    *rxbufstart = cmd[6];

    Serial.print("rx avail sz: ");
    Serial.print(*size);
    Serial.print(" buf strt: ");
    Serial.println(*rxbufstart);

    // payloadlen? - packetindex
    // TODO?  read seems to check the modem size against
    // the packetindex on the mcu side?
    return (*size) - _packetIndex;

  #if 0  // sx1262 status
    uint8_t buf[2] = {0};
    executeOpcodeRead(OP_RX_BUFFER_STATUS_6X, buf, 2);
    return buf[0] - _packetIndex;
  #endif
}

//  lr1110
int ISR_VECT lr11xx::read()
{
  uint8_t data_left, size, rxbufstart;

  // TODO? check this in available
  data_left = available(&size, &rxbufstart);
  if (!data_left) {
    return -1;
  }

  // if received new packet
  if (_packetIndex == 0) {
      _fifo_rx_addr_ptr = rxbufstart;

      readBuffer(_packet, size);
  }

  #if 0  // sx1262
  // if received new packet
  if (_packetIndex == 0) {
      uint8_t rxbuf[2] = {0};
      executeOpcodeRead(OP_RX_BUFFER_STATUS_6X, rxbuf, 2);
      int size = rxbuf[0];
      _fifo_rx_addr_ptr = rxbuf[1];

      readBuffer(_packet, size);
  }
  #endif

  uint8_t byte = _packet[_packetIndex];
  _packetIndex++;
  return byte;
}

// lr1110 - not impl - unused
int lr11xx::peek()
{
  Serial.println("XXXX - lr1110 peek called - not implemented - XXXX");
  #if 0
  if (!available()) {
    return -1;
  }

  // if received new packet
  if (_packetIndex == 0) {
      uint8_t rxbuf[2] = {0};
      executeOpcodeRead(OP_RX_BUFFER_STATUS_6X, rxbuf, 2);
      int size = rxbuf[0];
      _fifo_rx_addr_ptr = rxbuf[1];

      readBuffer(_packet, size);
  }

  uint8_t b = _packet[_packetIndex];
  return b;
  #endif

  return 0;
}

// lr1110 unused
void lr11xx::flush()
{
}

// lr1110
void lr11xx::onReceive(void(*callback)(int))
{
  Serial.println("XXX\n onReceive\nXXX");
  _onReceive = callback;

  if (callback) {

      // irq status read
      uint8_t cmd4[9] = {6,0,0x1,0x0, 0x0,0x0,0x0,0x0};

      cmdTransfer("->getStat dio setup", cmd4);

      Serial.print("pre setup- ");
      Serial.print(cmd4[2]);
      Serial.print(" ");
      Serial.print(cmd4[3]);
      Serial.print(" - ");
      Serial.print(cmd4[4]);
      Serial.print(" ");
      Serial.print(cmd4[5]);
      Serial.print(" ");
      Serial.print(cmd4[6]);
      Serial.print(" ");
      Serial.println(cmd4[7]);

    // Clear and read, clearirq
    uint8_t cmd2[9] = {6,0,0x1,0x14,cmd4[4],cmd4[5],cmd4[6],cmd4[7]};

    cmdTransfer("->clrIrq", cmd2);


    // enable RXDone, PreambleDetected, HeaderValid(Lora), 
    // headerCRCErr 0b111 1000
    // use SetDioIrqParams on lr1110
    // add TXDone, timeout
    // 0xb 1100 0000 - 0000 0100 - 1111 1100

    // DIO9 enabled for RXDone, DIO11 no enables
    // irqenable, irq enable

    // Rx buffer status
    // byte mapped wrong!
    //uint8_t cmd[13] = {10,0,0x1,0x13,0x0,0x0,0x0,0x78, 0x0,0x0,0x0,0x0};
    // all
    //uint8_t cmd[13] = {10,0,0x1,0x13,0x0,0xc0,0x4,0xfc, 0x0,0x0,0x0,0x0};

    // remove 0x 1100 1100 (preamble and syncword irq), upper bytes unchanged
    //uint8_t cmd[13] = {10,0,0x1,0x13,0x0,0xc0,0x4,0xcc, 0x0,0x0,0x0,0x0};

    // simple version, rxdone
//    uint8_t cmd[13] = {10,0,0x1,0x13,0x0,0x00,0x0,0x08, 0x0,0x0,0x0,0x0};
    // simple version, rxdone, txdone
    uint8_t cmd[13] = {10,0,0x1,0x13,0x0,0x00,0x0,0b1100, 0x0,0x0,0x0,0x0};

    cmdTransfer("->dioIrqParam", cmd);

    // clr irqs active when we programmed


#if 0
      uint8_t cmd4[9] = {6,0,0x1,0x0, 0x0,0x0,0x0,0x0};

      cmdTransfer("->getStat dio setup", cmd4);

      Serial.print("follow setup- ");
      Serial.print(cmd4[2]);
      Serial.print(" ");
      Serial.print(cmd4[3]);
      Serial.print(" - ");
      Serial.print(cmd4[4]);
      Serial.print(" ");
      Serial.print(cmd4[5]);
      Serial.print(" ");
      Serial.print(cmd4[6]);
      Serial.print(" ");
      Serial.println(cmd4[7]);
#endif



    #if 0
    pinMode(_dio0, INPUT);

    // set preamble and header detection irqs, plus dio0 mask
    uint8_t buf[8];

    // set irq masks, enable all
    buf[0] = 0xFF; 
    buf[1] = 0xFF;

    // set dio0 masks
    buf[2] = 0x00;
    buf[3] = IRQ_RX_DONE_MASK_6X; 

    // set dio1 masks
    buf[4] = 0x00; 
    buf[5] = 0x00;

    // set dio2 masks
    buf[6] = 0x00; 
    buf[7] = 0x00;

    executeOpcode(OP_SET_IRQ_FLAGS_6X, buf, 8);
    #endif

#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    attachInterrupt(digitalPinToInterrupt(_dio0), lr11xx::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

// lr1110 
void lr11xx::receive(int size)
{
    // test
    setModulationParams(_sf, _bw, _cr, _ldro);

    if (size > 0) {
        implicitHeaderMode();

        // tell radio payload length
        _payloadLength = size;
        packetParamsDirty=1;
        setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
    } else {
        explicitHeaderMode();
        _payloadLength = size;
        packetParamsDirty=1;
        setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
    }

    if (_rxen != -1) {
        rxAntEnable();
    }


        // LoRa ResetStats
        uint8_t cmd9[15] = {2,0,0x2,0x0};
        cmdTransfer("->ResetStats", cmd9);

    Serial.print("start rx < - - -  sz: ");
    Serial.println(size);


    // continuous mode
    // Rx mode, no timeout  (setRX)
    // stay until command/continuous mode
    uint8_t cmd[8] = {5,0,0x2,0x9,0xff,0xff,0xff};
    // go to standby after packet reception
    //uint8_t cmd[8] = {5,0,0x2,0x9,0x00,0x00,0x00};

    cmdTransfer("->rx", cmd);

  #if 1
    //getErrors2();
  #else
    Serial.println("get errors");
    uint8_t cmd2[8] = {5,0, 0x01, 0x0d, 0x0, 0x0,0x0};
    cmdTransfer("getErr", cmd2);
    Serial.print("Fail errors ");
    Serial.print(cmd2[5] & 0x1);
    Serial.print(", ");
    Serial.println(cmd2[6]);
  #endif

}

// lr1110 
void lr11xx::standby()
{
  // SetStandby Xosc mode
  //uint8_t cmd[6] = {3,0,0x1,0x1c,0x1};
  // standby RC osc
  uint8_t cmd[6] = {3,0,0x1,0x1c,0x0};

  //cmdTransfer("->stby xosc", cmd);
  cmdTransfer("->stby RC", cmd);
  Serial.println("go to standby  ----          XXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
}


// lr1110 
void lr11xx::sleep()
{
  // sleep mode
  // no retention, no wakeup
  //uint8_t cmd[10] = {7,0,0x1,0x1b,0x0, 0,0,0,0};
  // retention, no wakeup
  uint8_t cmd[10] = {7,0,0x1,0x1b,0x1, 0,0,0,0};

  cmdTransfer("->sleep", cmd);

  Serial.println("go to sleep  ----          XXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
}

void lr11xx::enableTCXO() {
  Serial.println("lr1110 enableTCXO() called, but internally enabled on current boards");
  #if 0
  #if HAS_TCXO
    #if BOARD_MODEL == BOARD_RAK4630 || BOARD_MODEL == BOARD_HELTEC_LORA32_V3 || BOARD_MODEL == BOARD_HELTEC_CAPSULE_V3 || BOARD_HELTEC_WIRELESS_PAPER_1_1
      uint8_t buf[4] = {MODE_TCXO_3_3V_6X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_TBEAM
      uint8_t buf[4] = {MODE_TCXO_1_8V_6X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_RNODE_NG_22
      uint8_t buf[4] = {MODE_TCXO_1_8V_6X, 0x00, 0x00, 0xFF};
    #endif
    executeOpcode(OP_DIO3_TCXO_CTRL_6X, buf, 4);
  #endif
  #endif
}

// TODO: Once enabled, SX1262 needs a complete reset to disable TCXO
void lr11xx::disableTCXO() { }

// lr1110 only
void lr11xx::setTxPower(int level, int outputPin) {
  if (level > 22) { level = 22; }
  else if (level < -17) { level = -17; }

  _txp = level;

  // PA Config
  uint8_t PaSel, RegPASupply, PaDutyCycle, PaHPSel = 0;

  // HP PA
  if(level>15) {
    PaSel = RegPASupply = 1;
    if(level==22) {
      PaDutyCycle = 4;
      PaHPSel = 7;
    } else if(level>=20) {
      PaDutyCycle = 2;
      PaHPSel = 7;
    } else if(level>=17) {
      // optional 1,5 instead of 4,3
      PaDutyCycle = 4;
      PaHPSel = 3;
    } else if(level>=14) {
      PaDutyCycle = 2;
      PaHPSel = 2;
    }

  // LP PA
  } else {
    PaSel = RegPASupply = 0;
    if(level==15) {
      PaDutyCycle = 7;
      PaHPSel = 0;
    } else if(level>=14) {
      PaDutyCycle = 4;
      PaHPSel = 0;
    } else {
      PaDutyCycle = 0;
      PaHPSel = 0;
    }
  }

  uint8_t cmd3[9] = {6,0,0x02, 0x15, PaSel, RegPASupply,
    PaDutyCycle, PaHPSel};

  cmdTransfer("->pa cfg", cmd3);

  //  TX Power

#if 0  /// direct write power
  // convert to LR1110 power register
  uint8_t regPower;
  if(PaSel) {
    // high power
    regPower = ((level - 22) * 225)/-31 +22;
  } else {
    // high efficiency
    regPower = ((level - 14) * 225)/-31 +14;
  }
  #endif

  uint8_t rampTime = 0x02;  // 40uS

  uint8_t cmd4[10] = {4,0,0x02, 0x11, level,rampTime};

  cmdTransfer("->tx pow", cmd4);

  // RXBoosted - enabled, ~2mA more consumption in RX
  // enable
  uint8_t cmd5[6] = {3,0,0x02, 0x27, 1};
  // disable
  //uint8_t cmd5[6] = {3,0,0x02, 0x27, 0};
  cmdTransfer("->rx boost", cmd5);

#if 0
  Serial.print("power req ");
  Serial.print(level);
#endif
  Serial.print("power reg: ");
  Serial.println(level);

  //  SX1262
  #if 0
    // currently no low power mode for SX1262 implemented, assuming PA boost
    
    // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
    // RegTxClampConfig = @address 0x08D8
    writeRegister(0x08D8, readRegister(0x08D8) | (0x0F << 1));

    uint8_t pa_buf[4];

    pa_buf[0] = 0x04; // PADutyCycle needs to be 0x04 to achieve 22dBm output, but can be lowered for better efficiency at lower outputs
    pa_buf[1] = 0x07; // HPMax at 0x07 is maximum supported for SX1262
    pa_buf[2] = 0x00; // DeviceSel 0x00 for SX1262 (0x01 for SX1261)
    pa_buf[3] = 0x01; // PALut always 0x01 (reserved according to datasheet)

    executeOpcode(OP_PA_CONFIG_6X, pa_buf, 4); // set pa_config for high power

    if (level > 22) { level = 22; }
    else if (level < -9) { level = -9; }

    writeRegister(REG_OCP_6X, 0x38); // 160mA limit, overcurrent protection

    uint8_t tx_buf[2];

    tx_buf[0] = level;
    tx_buf[1] = 0x02; // PA ramping time - 40 microseconds
    
    executeOpcode(OP_TX_PARAMS_6X, tx_buf, 2);

#endif
}

uint8_t lr11xx::getTxPower() {
    return _txp;
}

// lr1110 done
void lr11xx::setFrequency(long freq) {
  _frequency = freq;

  Serial.print( "freq - ");
  Serial.println(freq);
  // set freq cmd
  uint8_t cmd4[10] = {6,0,0x02,0x0b, 
    (freq >> 24) & 0xFF, 
    (freq >> 16) & 0xFF, 
    (freq >> 8) & 0xFF, 
    freq & 0xFF};

  cmdTransfer("->freq", cmd4);

  calibrate_image(_frequency);
  Serial.println("lr1110 calibrate");


}

uint32_t lr11xx::getFrequency() {
    // we can't read the frequency on the sx1262 / 80
    uint32_t frequency = _frequency;

    return frequency;
}

void lr11xx::setSpreadingFactor(int sf)
{
  if (sf < 5) {
      sf = 5;
  } else if (sf > 12) {
    sf = 12;
  }

  _sf = sf;

  handleLowDataRate();
  modulationDirty=1;
  //setModulationParams(sf, _bw, _cr, _ldro);
}

long lr11xx::getSignalBandwidth()
{
    int bw = _bw;
    switch (bw) {
        case 0x00: return 7.8E3;
        case 0x01: return 15.6E3;
        case 0x02: return 31.25E3;
        case 0x03: return 62.5E3;
        case 0x04: return 125E3;
        case 0x05: return 250E3;
        case 0x06: return 500E3;
        case 0x08: return 10.4E3;
        case 0x09: return 20.8E3;
        case 0x0A: return 41.7E3;
    }
  return 0;
}

void lr11xx::handleLowDataRate(){
  if ( long( (1<<_sf) / (getSignalBandwidth()/1000)) > 16) {
    _ldro = 0x01;
  } else {
    _ldro = 0x00;
  }
}

void lr11xx::optimizeModemSensitivity(){
    // todo: check if there's anything the sx1262 can do here
}

// lr1110
void lr11xx::setSignalBandwidth(long sbw)
{
  #if 0
  if (sbw <= 7.8E3) {
      _bw = 0x00;
  } else if (sbw <= 10.4E3) {
      _bw = 0x08;
  } else if (sbw <= 15.6E3) {
      _bw = 0x01;
  } else if (sbw <= 20.8E3) {
      _bw = 0x09;
  } else if (sbw <= 31.25E3) {
      _bw = 0x02;
  } else if (sbw <= 41.7E3) {
      _bw = 0x0A;
  } else if (sbw <= 62.5E3) {
      _bw = 0x03;
  } else if (sbw <= 125E3) {
      _bw = 0x04;
  } else if (sbw <= 250E3) {
      _bw = 0x05;
  } else /*if (sbw <= 250E3)*/ {
      _bw = 0x06;
  }
  #endif

  if (sbw <= 62.5E3) {
      _bw = 0x03;
  } else if (sbw <= 125E3) {
      _bw = 0x04;
  } else if (sbw <= 250E3) {
      _bw = 0x05;
  } else /*if (sbw <= 250E3)*/ {
      _bw = 0x06;
  }


  handleLowDataRate();
  modulationDirty=1;
  //setModulationParams(_sf, _bw, _cr, _ldro);

  optimizeModemSensitivity();
}

// lr1110 ok
void lr11xx::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

#if 1
  int cr = denominator - 4;
#else
  // lr1110 has two  interleavers, test with long interleaver
  int cr = denominator;
#endif

  _cr = cr;

  modulationDirty=1;
  //setModulationParams(_sf, _bw, cr, _ldro);
}

void lr11xx::setPreambleLength(long length)
{
  _preambleLength = length;
  packetParamsDirty=1;
  //setPacketParams(length, _implicitHeaderMode, _payloadLength, _crcMode);
}

void lr11xx::setSyncWord(uint16_t sw)
{
  // lr1110 not used?
  Serial.println("XXXXXXXXXXXXXX SyncWord attempted - not impl  XXXXXXXXXXXXXXXXXXXX");
  // TODO: Fix
    // writeRegister(REG_SYNC_WORD_MSB_6X, (sw & 0xFF00) >> 8);
    // writeRegister(REG_SYNC_WORD_LSB_6X, sw & 0x00FF);
    writeRegister(REG_SYNC_WORD_MSB_6X, 0x14);
    writeRegister(REG_SYNC_WORD_LSB_6X, 0x24);
}

void lr11xx::enableCrc()
{
    _crcMode = 1;
    packetParamsDirty=1;
    //setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

void lr11xx::disableCrc()
{
    _crcMode = 0;
    packetParamsDirty=1;
    //setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

// done lr1110
byte lr11xx::random()
{
  // generate byte of random with built in 
  // Random Number Gen (RNG)
  NRF_RNG->EVENTS_VALRDY = 0;
  NRF_RNG->TASKS_START = 1;
  while (NRF_RNG->EVENTS_VALRDY == 0)
  {
    // wait for start
  }
  uint8_t rand1 = NRF_RNG->VALUE;
  NRF_RNG->TASKS_STOP = 1;

    return readRegister(rand1);
    //return readRegister(REG_RANDOM_GEN_6X);
}

void lr11xx::setPins(int ss, int reset, int dio0, int busy /*, int rxen */)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
  _busy = busy;
//  _rxen = rxen;
}

void lr11xx::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void lr11xx::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

void lr11xx::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  packetParamsDirty=1;
  //setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

void lr11xx::implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  packetParamsDirty=1;
  //setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

void lr11xx::dioStatInternal(uint8_t *stat1, uint8_t *int2, uint8_t *int3, uint8_t *int4)
{

  //Serial.print("handleDio  --------------- ");

    // getStatus
    uint8_t cmd[9] = {2,4,0x1,0x0, 0x0,0x0,0x0,0x0};
    uint8_t clr[9] = {6,0,0x1,0x14,0x0,0x0,0x0,0x0};
    int doClr = 0;

    cmdTransfer("->getStat irq", cmd);

#if 1
  Serial.print(cmd[2]);
  Serial.print(" ");
  Serial.print(cmd[3]);
  Serial.print(" - ");
  Serial.print(cmd[4]);
  Serial.print(" ");
  Serial.print(cmd[5]);
  Serial.print(" ");
  Serial.print(cmd[6]);
  Serial.print(" ");
  Serial.println(cmd[7]);
#endif

  if(stat1) *stat1 = cmd[2];
  if(int2) *int2 = cmd[5];
  if(int3) *int3 = cmd[6];
  if(int4) *int4 = cmd[7];
}


void lr11xx::dioStat(uint8_t *stat1, uint8_t *int2,  uint8_t *int3, uint8_t *int4)
{
  uint8_t l_stat1, l_int2, l_int3, l_int4;
  dioStatInternal(&l_stat1, &l_int2, &l_int3, &l_int4);

  // Test for CMD_DAT, then IRQ stat was not sent
  if(l_stat1 && 0b0110)
  {
    dioStatInternal(&l_stat1, &l_int2, &l_int3, &l_int4);
  }

  if(stat1) *stat1 = l_stat1;
  if(int2) *int2 = l_int2;
  if(int3) *int3 = l_int3;
  if(int4) *int4 = l_int4;

}

uint8_t ISR_VECT lr11xx::dumpRx(void)
{
        uint8_t cmd2[8] = {2,3,0x2,0x3};

        cmdTransfer("->getRxBuf", cmd2);
//        int packetLength = cmd2[6];
        int packetLength = cmd2[5];

        Serial.print("stat1.1 ");
        Serial.print(cmd2[2]);
        Serial.print(" stat2 ");
        Serial.print(cmd2[3]);
        Serial.print(" stat1.2 ");
        Serial.print(cmd2[4]);
        Serial.print(" pack len= ");
        Serial.print(packetLength);
        Serial.print(" start= ");
        Serial.println(cmd2[6]);

        uint8_t stat2 = cmd2[3];
        return stat2;
}


// lr1110 - todo 2 item - ignores error, packet length
void ISR_VECT lr11xx::handleDio0Rise()
{
  uint8_t l_stat1, l_int4;
    uint8_t cmd[9] = {2,4,0x1,0x0, 0x0,0x0,0x0,0x0};
    uint8_t clr[9] = {6,0,0x1,0x14,0x0,0x0,0x0,0x0};
    int doClr = 0;

//  TEST TEST
  dumpRx();

//  TEST TEST




  //dioStat();
  dioStat(&cmd[2], &cmd[5], &cmd[6], &cmd[7]);

#if 0
  Serial.print("handleDio  --------------- ");

    // getStatus
    uint8_t cmd[9] = {2,4,0x1,0x0, 0x0,0x0,0x0,0x0};
    uint8_t clr[9] = {6,0,0x1,0x14,0x0,0x0,0x0,0x0};
    int doClr = 0;

    cmdTransfer("->getStat irq", cmd);

  Serial.print(cmd[2]);
  Serial.print(" ");
  Serial.print(cmd[3]);
  Serial.print(" - ");
  Serial.print(cmd[4]);
  Serial.print(" ");
  Serial.print(cmd[5]);
  Serial.print(" ");
  Serial.print(cmd[6]);
  Serial.print(" ");
  Serial.println(cmd[7]);
#endif

    // Clear and read, clr RXDone
//    uint8_t cmd[9] = {6,0,0x1,0x14,0x0,0x0,0x0,0x8};

//    cmdTransfer("->clrIrq", cmd);

#if 0
    uint8_t buf[2];

    buf[0] = 0x00;
    buf[1] = 0x00;

    executeOpcodeRead(OP_GET_IRQ_STATUS_6X, buf, 2);

    executeOpcode(OP_CLEAR_IRQ_STATUS_6X, buf, 2);
#endif

    // if TXdone
    if (cmd[7] & 0b100) {
      clr[7] = 0b100;
      doClr = 1;
      TXCompl_flag=1;
    }


    int packetLength = 0;

    // if RXdone
    if (cmd[7] & 0b1000) {
      clr[7] = 0b1000;
      doClr = 1;

        Serial.println("RxDone ------------------------");

  Serial.print(cmd[4]);
  Serial.print(" ");
  Serial.print(cmd[5]);
  Serial.print(" ");
  Serial.print(cmd[6]);
  Serial.print(" ");
  Serial.println(cmd[7]);


      // if no CRC header error
      if ((cmd[7] & 0b1000000) == 0) {


#if 0  // todo - check CRC Error
    if ((buf[1] & IRQ_PAYLOAD_CRC_ERROR_MASK_6X) == 0) {
#endif
        // received a packet
        _packetIndex = 0;

#if 0
        // read packet length
        uint8_t rxbuf[2] = {0};
        executeOpcodeRead(OP_RX_BUFFER_STATUS_6X, rxbuf, 2);
        int packetLength = rxbuf[0];
#endif


/////////////////// -- need this for size
#if 0
        uint8_t cmd2[8] = {2,3,0x2,0x3};

        cmdTransfer("->getRxBuf", cmd2);
//        int packetLength = cmd2[6];
        packetLength = cmd2[5];

        Serial.print("stat1.1 ");
        Serial.print(cmd2[2]);
        Serial.print(" stat2 ");
        Serial.print(cmd2[3]);
        Serial.print(" stat1.2 ");
        Serial.print(cmd2[4]);
        Serial.print(" pack len= ");
        Serial.print(packetLength);
        Serial.print(" start= ");
        Serial.println(cmd2[6]);

        #if 0
        packetLength = dumpRx();
        #else
        dumpRx();
        packetLength = 0;    // TODO = =TEST TEST
        #endif
        dumpRx();
#endif



#if 0
        // ReadBuffer8
        uint8_t cmd4[13] = {4,6,0x1,0xa, 0, 6};
        cmdTransfer("->getRxData", cmd4);
        Serial.print("RX buf > ");
        for (int i=7; i< 6+7; i++) {
          Serial.print(cmd4[i]);
          Serial.print(" ");
        }
        Serial.print("< > ");

        for (int i=7+cmd2[6]; i< 6+7+cmd2[6]; i++) {
          Serial.print(cmd4[i]);
          Serial.print(" ");
        }
        Serial.println("<");
#endif

        // LoRa GetStats
        uint8_t cmd9[15] = {2,9,0x2,0x1};
        cmdTransfer("->getStats", cmd9);

        Serial.print("Lora stats, pkts ");
        Serial.print( (cmd9[5] << 8) | cmd[6] );
        Serial.print(" err pkts, crc ");
        Serial.print( (cmd9[7] << 8) | cmd[8] );
        Serial.print(" hdr ");
        Serial.print( (cmd9[9] << 8) | cmd[10] );
        Serial.print(" fls sync ");
        Serial.println( (cmd9[11] << 8) | cmd[12] );


        if (_onReceive && packetLength) {
          Serial.println("call onRec");
          //delay(50);
            _onReceive(packetLength);
          Serial.println("call onRec");

//        Serial.print("pack len= ");
//        Serial.println(packetLength);
        } else {
          Serial.println("RXDone - missing handler or 0 packet len");
        }

        if (!packetLength) {
            // continuous mode
            // Rx mode, no timeout  (setRX)
            uint8_t cmd[8] = {5,0,0x2,0x9,0xff,0xff,0xff};

            cmdTransfer("->rx", cmd);
            Serial.println("Restart RX");
        }

  #if 0
    }
  #endif

      } else {
        // clear CRC header err
        clr[7] = 0b1000000;
        doClr = 1;

        Serial.println("  xxxxxxxx   RXDone - CRC Header Err      EEEEEEEEEEEEEEE");
      }
    }

    if(cmd[7] && 0b10000 )
    {
        clr[7] |= 0b10000;
        doClr = 1;
    //    Serial.println("Preamble");
    }
    if(cmd[7] && 0b100000 )
    {
        clr[7] |= 0b100000;
        doClr = 1;
    //    Serial.println("Header");

    }

    if (doClr) 
    {
      cmdTransfer("->clear irqs", clr, 0);
      #if 0
      Serial.print("handleDio  --------------- clr ");
      Serial.print(clr[2]);
      Serial.print(" ");
      Serial.print(clr[3]);
      Serial.print(" - ");
      Serial.print(clr[6]);
      Serial.print(" ");
      Serial.println(clr[7]);
      #endif
    }

    // else {
    //   Serial.println("CRCE");
    //   Serial.println(buf[0]);
    //   Serial.println(buf[1]);
    // }
}

void ISR_VECT lr11xx::onDio0Rise()
{
    lr11xx_modem.handleDio0Rise();
}

lr11xx lr11xx_modem;

#endif