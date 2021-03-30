#include <cox.h>

Timer sendTimer;
RadioPacket *frame = NULL;
uint32_t sent = 0;
struct timeval tSent;

char buf[20];
int8_t modem;
Radio::LoRaSF_t sf = Radio::SF7;
Radio::LoRaCR_t cr = Radio::CR_4_5;
Radio::LoRaBW_t bw = Radio::BW_125kHz;
int8_t txPower = 14;
bool iqInverted = false;
bool publicNetwork = false;
uint32_t freq = 921700000;
bool packetMode = true;

static void sendTask(void *args) {
  if (frame != NULL) {
    printf("Tx in progress...\n");
    return;
  }

  frame = new RadioPacket(125);
  if (!frame) {
    printf("Not enough memory\n");
    return;
  }

  for (uint8_t n = 2; n < frame->len; n++)
    frame->buf[n] = n;

  frame->buf[0] = (sent >> 8);
  frame->buf[1] = (sent & 0xff);

  SubGHzRadio.wakeup();
  if (SubGHzRadio.transmit(frame) == ERROR_SUCCESS) {
    gettimeofday(&tSent, nullptr);

    printf("[%lu.%06lu] %lu Tx started...\n", (uint32_t) tSent.tv_sec, tSent.tv_usec, sent);
    sent++;
  } else {
    delete frame;
    frame = nullptr;
    printf("Tx error\n");

    postTask(sendTask);
  }
}

static void eventOnTxDone(void *ctx, bool success, GPIOInterruptInfo_t *intrInfo) {
  printf("[%lu.%06lu] Tx", (uint32_t) intrInfo->timeEnteredISR.tv_sec, intrInfo->timeEnteredISR.tv_usec);
  if (success) {
    struct timeval tDuration;
    timersub(&intrInfo->timeEnteredISR, &tSent, &tDuration);
    printf("SUCCESS! (duration: %lu.%06lu)\n", (uint32_t) tDuration.tv_sec, tDuration.tv_usec);
  } else {
    printf("FAIL!\n");
  }
  delete frame;
  frame = nullptr;
  SubGHzRadio.sleep();
  postTask(sendTask);
}

static void eventKeyStroke(SerialPort &) {
  reboot();
}

static void appStart() {
  // Serial.stopInput(); // to receive key stroke not string
  // Serial.onReceive(eventKeyStroke);
  Serial.stopListening();

  /* All parameters are specified. */

  // if (modem == 0) {
    SubGHzRadio.setRadio(sf, bw, cr, true, iqInverted);
    SubGHzRadio.setPublicNetwork(publicNetwork);
  // } else {
  //   SubGHzRadio.setModemFsk();
  //   SubGHzRadio.setDataRate(50000);
  //   SubGHzRadio.setBandwidth(50000);
  //   SubGHzRadio.setAfcBandwidth(83333);
  //   SubGHzRadio.setFdev(25000);
  // }

  SubGHzRadio.setChannel(freq);
  SubGHzRadio.setTxPower(txPower);

  if (packetMode) {
    SubGHzRadio.onTxDone = eventOnTxDone;
    // SubGHzRadio.wakeup();

    // sendTimer.onFired(sendTask, NULL);
    // sendTimer.startPeriodic(6000);
    postTask(sendTask);
  } else {
    SubGHzRadio.transmitCW();
  }
}

static void inputPacketMode(SerialPort &);
static void askPacketMode() {
  printf("Select mode (0: CW, 1: packet) [%d]:", packetMode);
  Serial.onReceive(inputPacketMode);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputPacketMode(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "1") == 0) {
    printf("* Packet mode selected.\n");
  } else if (strcmp(buf, "0") == 0) {
    printf("* Continuous wave mode selected.\n");
    packetMode = false;
  } else {
    printf("* Unknown mode\n");
    askPacketMode();
    return;
  }

  appStart();
}

static void inputFrequency(SerialPort &);
static void askFrequency() {
  printf("Enter frequency in unit of Hz [%lu]:", freq);
  Serial.onReceive(inputFrequency);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputFrequency(SerialPort &) {
  if (strlen(buf) > 0) {
    uint32_t f = (uint32_t) strtoul(buf, NULL, 0);
    if (f == 0) {
      printf("* Invalid frequency.\n");
      askFrequency();
      return;
    }

    freq = f;
  }

  printf("* Frequency: %lu\n", freq);
  askPacketMode();
}

static void inputSyncword(SerialPort &);
static void askSyncword() {
  printf("Use public network [yN]:");
  Serial.onReceive(inputSyncword);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputSyncword(SerialPort &) {
  if (strlen(buf) == 0) {
    publicNetwork = false;
  } else {
    if (strcmp(buf, "y") == 0 || strcmp(buf, "Y") == 0) {
      publicNetwork = true;
    } else if (strcmp(buf, "n") == 0 || strcmp(buf, "N") == 0) {
      publicNetwork = false;
    } else {
      printf("* Invalid syncword.\n");
      askSyncword();
      return;
    }

    publicNetwork = true;
  }

  printf("* Public network: %s\n", publicNetwork ? "true" : "false");
  askFrequency();
}

static void inputIQ(SerialPort &);
static void askIQ() {
  printf("Set IQ (1:normal, 2:inverted) [1]:");
  Serial.onReceive(inputIQ);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputIQ(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "1") == 0) {
    printf("* Normal selected.\n");
    iqInverted = false;
  } else if (strcmp(buf, "2") == 0) {
    printf("* inverted selected.\n");
    iqInverted = true;
  } else {
    printf("* Unknown IQ mode\n");
    askIQ();
    return;
  }
  askSyncword();
}

static void inputTxPower(SerialPort &);
static void askTxPower() {
  printf("Set Tx power (-1 ~ 20) [14]:");
  Serial.onReceive(inputTxPower);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputTxPower(SerialPort &) {
  if (strlen(buf) != 0) {
    txPower = (uint8_t) strtol(buf, NULL, 0);
  }

  printf("* %d dBm selected.\n", txPower);

  if (txPower < -1 || txPower > 20) {
    printf("* Unknown Tx power.\n");
    askTxPower();
    return;
  }

  askIQ();
}

static void inputBW(SerialPort &);
static void askBW() {
  printf("Set bandwidth (1:125kHz, 2:250kHz, 3:500kHz) [1]:");
  Serial.onReceive(inputBW);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputBW(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "1") == 0) {
    printf("* 125kHz selected.\n");
  } else if (strcmp(buf, "2") == 0) {
    printf("* 250kHz selected.\n");
    bw = Radio::BW_250kHz;
  } else if (strcmp(buf, "3") == 0) {
    printf("* 500kHz selected.\n");
    bw = Radio::BW_500kHz;
  } else {
    printf("* Unknown SF\n");
    askBW();
    return;
  }
  askTxPower();
}

static void inputCR(SerialPort &);
static void askCR() {
  printf("Set coding rate (1:4/5, 2:4/6, 3:4/7, 4:4/8) [1]:");
  Serial.onReceive(inputCR);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputCR(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "1") == 0) {
    printf("* (4/5) selected.\n");
  } else if (strcmp(buf, "2") == 0) {
    printf("* (4/6) selected.\n");
    cr = Radio::CR_4_6;
  } else if (strcmp(buf, "3") == 0) {
    printf("* (4/7) selected.\n");
    cr = Radio::CR_4_7;
  } else if (strcmp(buf, "4") == 0) {
    printf("* (4/8) selected.\n");
    cr = Radio::CR_4_8;
  } else {
    printf("* Unknown coding rate\n");
    askCR();
    return;
  }
  askBW();
}

static void inputSF(SerialPort &);
static void askSF() {
  printf("Set SF (7, 8, 9, 10, 11, 12) [7]:");
  Serial.onReceive(inputSF);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputSF(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "7") == 0) {
    printf("* SF7 selected.\n");
  } else if (strcmp(buf, "8") == 0) {
    printf("* SF8 selected.\n");
    sf = Radio::SF8;
  } else if (strcmp(buf, "9") == 0) {
    printf("* SF9 selected.\n");
    sf = Radio::SF9;
  } else if (strcmp(buf, "10") == 0) {
    printf("* SF10 selected.\n");
    sf = Radio::SF10;
  } else if (strcmp(buf, "11") == 0) {
    printf("* SF11 selected.\n");
    sf = Radio::SF11;
  } else if (strcmp(buf, "12") == 0) {
    printf("* SF12 selected.\n");
    sf = Radio::SF12;
  } else {
    printf("* Unknown SF (%s)\n", buf);
    askSF();
    return;
  }
  askCR();
}

static void inputModem(SerialPort &);
static void askModem() {
  printf("Select modem (1: LoRa, 2:FSK) [1]:");
  Serial.onReceive(inputModem);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputModem(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "1") == 0) {
    printf("* LoRa selected.\n");
    askSF();
  } else if (strcmp(buf, "2") == 0) {
    printf("* FSK selected.\n");
    modem = 1;
    appStart();
  } else {
    printf("* Unknown modem\n");
    askModem();
  }
}


void setup() {
  Serial.begin(115200);
  printf("*** [PLM150] Sub GHz Radio Tx Control Example ***\n");

  SubGHzRadio.begin();
  SubGHzRadio.sleep();

#if 1
  Serial.listen();
  askModem();
#else
  modem = 0;
  txPower = 14;
  cr = Radio::CR_4_8;
  sf = Radio::SF7;
  bw = Radio::BW_125kHz;
  iq = true;
  syncword = 0x12;
  appStart();
#endif
}
