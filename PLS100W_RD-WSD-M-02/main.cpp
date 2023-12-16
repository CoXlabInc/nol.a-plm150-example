#include <cox.h>
#include <algorithm>
#include <deque>
#include <LoRaMacKR920.hpp>
#include <stm32wl/STM32WLxxFlashPage.hpp>
#include <ConfigConsole.hpp>
#include <ConfigBlob.hpp>
#include <ConfigBool.hpp>
#include <ConfigNumber.hpp>
#include <RS485.hpp>

STM32WLxxFlashPage configMemory(0x803F800);

static ConfigConsole config(Serial);
static ConfigBlob configDevEui("deveui", configMemory, 0, 8);
static ConfigBlob configAppKey("appkey", configMemory, 8, 16);
static ConfigNumber<uint16_t> configPeriod("period", configMemory, 24, 1, 65534);
static ConfigBool configLoRaPublic("lorapublic", configMemory, 26);
static ConfigNumber<int8_t> configAntGain("antgain", configMemory, 27, -128, 127);

// #define SENSOR_TEST_MODE

char keyBuf[128];
Timer timerKeyInput;

LoRaMacKR920 LoRaWAN(SubGHzRadio, 24);

struct timeval tLastTimesync = { .tv_sec = 0, .tv_usec = 0 };
static Timer timeoutTimer;
static Timer watchdogTimer;

std::string message = "";

RS485 rs485(Serial2, PLM100::GPIO7, PLM100::GPIO7);
std::deque<uint8_t> rxq;

static uint16_t crc16_update(uint16_t crc, uint8_t a) {
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i) {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }

  return crc;
}

static void sense(void* data){
  printf("Request [");
  
  uint8_t request[8];
  request[0] = 0x01; /* slave ID */
  request[1] = 0x03; /* read holding registers */
  request[2] = 0x00; /* start address (H) */
  request[3] = 0x00; /* start address (L) */
  request[4] = 0x00; /* number of registers (H) */
  request[5] = 0x02; /* number of registers (L) */

  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < 6; i++) {
    crc = crc16_update(crc, request[i]);
    printf(" %02X", request[i]);
  }

  request[6] = (uint8_t) ((crc >> 0) & 0xFF);
  request[7] = (uint8_t) ((crc >> 8) & 0xFF);

  printf(" %02X %02X ]\n", request[6], request[7]);
  
  rs485.beginTransmission();
  rs485.write(request, sizeof(request));
  rs485.endTransmission();

  timeoutTimer.onFired(sense);
  timeoutTimer.startOneShot(1000);
}

static void report() {
  LoRaMacFrame *f = new LoRaMacFrame(255);

  f->port = 1;
  f->type = LoRaMacFrame::CONFIRMED;

  strncpy((char *) f->buf, message.c_str(), 255);
  f->len = strlen((char *) f->buf);

  error_t err = LoRaWAN.send(f);
  printf("* Sending periodic report (%p:%s (%u byte)): %d\n", f, f->buf, f->len, err);
  if (err != ERROR_SUCCESS) {
    delete f;

    timeoutTimer.onFired(sense);
    timeoutTimer.startOneShot(configPeriod.getValue() * 1000);
  }
}

static void eventLoRaWANJoin(
  LoRaMac &,
  bool joined,
  const uint8_t *joinedDevEui,
  const uint8_t *joinedAppEui,
  const uint8_t *joinedAppKey,
  const uint8_t *joinedNwkSKey,
  const uint8_t *joinedAppSKey,
  uint32_t joinedDevAddr,
  const RadioPacket &frame,
  uint32_t
) {
  if (joined) {
    printf("* Joined to the network!\n");
    postTask(sense);

    watchdogTimer.onFired([](void *) {
                            printf("* Watchdog!\n");
                            reboot();
                          });
    watchdogTimer.startOneShot(configPeriod.getValue() * 10 * 1000);
  } else {
    printf("* Join failed. Retry to join\n");
    LoRaWAN.beginJoining(configDevEui.getValue().data(), configAppKey.getValue().data());
  }
}

static void eventLoRaWANSendDone(LoRaMac &, LoRaMacFrame *frame) {
  printf("* Send done(%d): [%p] destined for port[%u], Freq:%lu Hz, Power:%d dBm, # of Tx:%u, ",
         frame->result, frame, frame->port, frame->freq, frame->power, frame->numTrials);
  if (frame->modulation == Radio::MOD_LORA) {
    const char *strBW[] = { "Unknown", "125kHz", "250kHz", "500kHz", "Unexpected value" };
    if (frame->meta.LoRa.bw > 3) {
      frame->meta.LoRa.bw = (Radio::LoRaBW_t) 4;
    }
    printf("LoRa, SF:%u, BW:%s, ", frame->meta.LoRa.sf, strBW[frame->meta.LoRa.bw]);
  } else if (frame->modulation == Radio::MOD_FSK) {
    printf("FSK, ");
  } else {
    printf("Unkndown modulation, ");
  }
  if (frame->type == LoRaMacFrame::UNCONFIRMED) {
    printf("UNCONFIRMED");
  } else if (frame->type == LoRaMacFrame::CONFIRMED) {
    printf("CONFIRMED");
  } else if (frame->type == LoRaMacFrame::MULTICAST) {
    printf("MULTICAST (error)");
  } else if (frame->type == LoRaMacFrame::PROPRIETARY) {
    printf("PROPRIETARY");
  } else {
    printf("unknown type");
  }
  printf(" frame\n");

  if ( frame->result == RadioPacket::TOO_BIG){
    printf("- TOO BIG\n");
  }
  delete frame;

  timeoutTimer.onFired(sense);
  timeoutTimer.startOneShot(configPeriod.getValue() * 1000);
}

static void eventLoRaWANReceive(LoRaMac &lw, const LoRaMacFrame *frame) {
  printf("* Received:");
  for (uint8_t i = 0; i < frame->len; i++) {
    printf(" %02X", frame->buf[i]);
  }
  printf(" (");
  frame->printTo(Serial);
  printf(")\n");

  if ((frame->type == LoRaMacFrame::CONFIRMED || lw.framePending) &&
      lw.getNumPendingSendFrames() == 0) {
    // If there is no pending send frames, send an empty frame to ack or pull more frames.
    LoRaMacFrame *emptyFrame = new LoRaMacFrame(0);
    if (emptyFrame) {
      error_t err = LoRaWAN.send(emptyFrame);
      if (err != ERROR_SUCCESS) {
        delete emptyFrame;
      }
    }
  }

  watchdogTimer.startOneShot(configPeriod.getValue() * 10 * 1000);
}

static void readFromBus(void *) {
  while (rs485.available() > 0) {
    uint8_t c = rs485.read();

    rxq.push_back(c);

    while (rxq.size() >= 9) {
      if (rxq[0] != 0x01 /* slave ID */) {
        rxq.pop_front();
        continue;
      }

      if (rxq[1] != 0x03 /* read holding registers */) {
        rxq.pop_front();
        continue;
      }

      if (rxq[2] != 0x04 /* data length */) {
        rxq.pop_front();
        continue;
      }

      uint16_t crc = 0xFFFF;
      for (uint8_t i = 0; i < 7; i++) {
        crc = crc16_update(crc, rxq[i]);
      }

      if (rxq[7] != ((crc >> 0) & 0xFF) ||
          rxq[8] != ((crc >> 8) & 0xFF)) {
        rxq.pop_front();
        continue;
      }

      timeoutTimer.stop();

      uint16_t speed = ((uint16_t) rxq[3] << 8) + rxq[4];
      message = "\"wind_speed\":";
      message += std::to_string(speed / 10);
      message += '.';
      message += std::to_string(speed % 10);

      uint16_t direction = ((uint16_t) rxq[5] << 8) + rxq[6];
      message += ",\"wind_direction\":";
      message += std::to_string(direction);

      rxq.clear();
      report();
    }
  }
}

static void sendTimeSync(void *) {
  error_t err = LoRaWAN.requestDeviceTime(&System);
  printf("* First, Request DeviceTime: %d\n", err);

  /* Send the empty packet to sync device time */
  LoRaMacFrame *f = new LoRaMacFrame(0);
  LoRaWAN.send(f);
}

static void onConfigDone() {
  pinMode(PLM100::GPIO10, OUTPUT);
  digitalWrite(PLM100::GPIO10, HIGH);
  
  Serial.stopListening();

  rs485.begin(9600);
  rs485.onReceive(readFromBus, nullptr);
  rs485.receive();

  LoRaWAN.begin();

  LoRaWAN.onSendDone(eventLoRaWANSendDone);
  LoRaWAN.onReceive(eventLoRaWANReceive);
  LoRaWAN.onJoin(eventLoRaWANJoin);
  LoRaWAN.setPublicNetwork(configLoRaPublic.getValue());
  LoRaWAN.beginJoining(configDevEui.getValue().data(),
                       (const uint8_t *) "\x00\x00\x00\x00\x00\x00\x00\x00",
                       configAppKey.getValue().data());
  printf("* Beginning LoRaWAN...\n");

  LoRaWAN.onMACCommandReceived =
    [](LoRaMac &mac,
       LoRaMac::MacCmd_t cid,
       const uint8_t *reqData,
       int16_t reqLen,
       const uint8_t *ansData,
       int16_t ansLen) {
      printf("* MAC command received:0x%02x (len req:%d, ans:%d)\n", (uint8_t) cid, reqLen, ansLen);
      if (cid == LoRaMac::MAC_CMD_DEV_TIME) {
        if (reqLen > 0) {
          printf(" * Req:");
          for (int16_t i = 0; i < reqLen; i++) {
            printf(" %02x", reqData[i]);
          }
          printf("\n");
        } else if (reqLen < 0) {
          printf(" * Req not issued\n");
        }
        if (ansLen > 0) {
          printf(" * Ans:");
          for (int16_t i = 0; i < ansLen; i++) {
            printf(" %02x", ansData[i]);
          }
          printf("\n");

          struct tm tUtc;
          System.getUTC(tUtc);
          printf(" * Timesync result: %d-%d-%d %02d:%02d:%02d\n",
                 tUtc.tm_year + 1900, tUtc.tm_mon + 1, tUtc.tm_mday,
                 tUtc.tm_hour, tUtc.tm_min, tUtc.tm_sec);
          System.getTimeSinceBoot(&tLastTimesync);
        } else if (ansLen < 0) {
          printf(" * No ans\n");
          postTask(sendTimeSync);
        }
      }
    };
}

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** [PLS100W+PLM100WL] Wind speed & direction sensor (RD-WSD-M02) + LoRaWAN ***\n");
  Serial.printf("- DevEui: %02X%02X%02X%02X%02X%02X%02X%02X\n",
                configDevEui.getValue()[0], configDevEui.getValue()[1],
                configDevEui.getValue()[2], configDevEui.getValue()[3],
                configDevEui.getValue()[4], configDevEui.getValue()[5],
                configDevEui.getValue()[6], configDevEui.getValue()[7]);
  config.addItem(&configDevEui);
  config.addItem(&configAppKey);
  config.addItem(&configPeriod);
  config.addItem(&configLoRaPublic);
  config.addItem(&configAntGain);

  if (config.allItemsAreValid()) {
    config.begin(3, onConfigDone);
  } else {
    config.begin(0, onConfigDone);
  }
}
