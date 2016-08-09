#include <cox.h>

Timer tRSSI;
uint32_t tRxStarted;
int16_t rssiRxStarted;
char buf[20];
int8_t modem;
int8_t sf;
int8_t cr;
int8_t bw;
bool iq;
uint8_t syncword;

static void printRxDone(void *args) {
  RadioPacket *rxFrame = (RadioPacket *) args;

  printf("Rx is done!: RSSI:%d dB, CRC:%s, Length:%u, (",
         rxFrame->rssi,
         rxFrame->crc_ok ? "OK" : "FAIL",
         rxFrame->len);
  uint16_t i;
  for (i = 0; i < rxFrame->len; i++)
    printf("%02X ", rxFrame->buf[i]);
  printf("\b)\n");

  delete rxFrame;
}

static void eventOnRxDone(void *ctx) {
  RadioPacket *rxFrame = new RadioPacket(125);
  if (!rxFrame) {
    printf("Out of memory to read frame\n");
    SX1276.flushBuffer();
    return;
  }

  SX1276.readFrame(rxFrame);
  postTask(printRxDone, (void *) rxFrame);
  SX1276.cca();
}

static void eventOnChannelBusy(void *ctx) {
  printf("Channel Busy!!\n");
  SX1276.cca();
}

static void printRxStarted(void *args) {
  printf("[%lu us] Rx is started... (%d dB)\n", tRxStarted, rssiRxStarted);
}

static void eventOnRxStarted(void *ctx) {
  tRxStarted = micros();
  rssiRxStarted = SX1276.getRssi();
  postTask(printRxStarted, NULL);
}

static void taskRSSI(void *args) {
  ledToggle(0);
  printf("[%lu us] RSSI: %d dB\n", micros(), SX1276.getRssi());
}

static void eventKeyboardInput(SerialPort &) {
  reboot();
}

static void appStart() {
  Serial.onReceive(eventKeyboardInput);

  /* All parameters are specified. */
  SX1276.begin();

  if (modem == 0) {
    SX1276.setModemLoRa();
    SX1276.setDataRate(sf);
    SX1276.setCodingRate(cr);
    SX1276.setBandwidth(bw);
    SX1276.setIQMode(iq);
  } else {
    SX1276.setModemFsk();
    SX1276.setDataRate(50000);
    SX1276.setBandwidth(50000);
    SX1276.setAfcBandwidth(83333);
    SX1276.setFdev(25000);
  }

  SX1276.setChannel(917300000);
  SX1276.onRxStarted(eventOnRxStarted, NULL);
  SX1276.onRxDone(eventOnRxDone, NULL);
  SX1276.onChannelBusy(eventOnChannelBusy, NULL);
  SX1276.wakeup();
  SX1276.cca();

  tRSSI.onFired(taskRSSI, NULL);
  tRSSI.startPeriodic(1000);
}

static void inputSyncword(SerialPort &);
static void askSyncword() {
  printf("Enter syncword [0x12]:");
  Serial.onReceive(inputSyncword);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputSyncword(SerialPort &) {
  if (strlen(buf) == 0) {
    syncword = 0x12;
  } else {
    uint8_t sw = (uint8_t) strtoul(buf, NULL, 0);
    if (sw == 0) {
      printf("* Invalid syncword.\n");
      askSyncword();
      return;
    }

    syncword = sw;
  }

  printf("* Syncword: 0x%02X\n", syncword);
  appStart();
}

static void inputIQ(SerialPort &);
static void askIQ() {
  printf("Set IQ (0:normal, 1:inverted) [0]:");
  Serial.onReceive(inputIQ);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputIQ(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "0") == 0) {
    printf("* Normal selected.\n");
    iq = true;
  } else if (strcmp(buf, "1") == 0) {
    printf("* inverted selected.\n");
    iq = false;
  } else {
    printf("* Unknown IQ mode\n");
    askIQ();
    return;
  }
  askSyncword();
}

static void inputBW(SerialPort &);
static void askBW() {
  printf("Set bandwidth (0:125kHz, 1:250kHz, 2:500kHz) [0]:");
  Serial.onReceive(inputBW);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputBW(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "0") == 0) {
    printf("* 125kHz selected.\n");
    bw = 0;
  } else if (strcmp(buf, "1") == 0) {
    printf("* 250kHz selected.\n");
    bw = 1;
  } else if (strcmp(buf, "2") == 0) {
    printf("* 500kHz selected.\n");
    bw = 2;
  } else {
    printf("* Unknown SF\n");
    askBW();
    return;
  }
  askIQ();
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
    cr = 1;
  } else if (strcmp(buf, "2") == 0) {
    printf("* (4/6) selected.\n");
    cr = 2;
  } else if (strcmp(buf, "3") == 0) {
    printf("* (4/7) selected.\n");
    cr = 3;
  } else if (strcmp(buf, "4") == 0) {
    printf("* (4/8) selected.\n");
    cr = 4;
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
    sf = 7;
  } else if (strcmp(buf, "8") == 0) {
    printf("* SF8 selected.\n");
    sf = 8;
  } else if (strcmp(buf, "9") == 0) {
    printf("* SF9 selected.\n");
    sf = 9;
  } else if (strcmp(buf, "10") == 0) {
    printf("* SF10 selected.\n");
    sf = 10;
  } else if (strcmp(buf, "11") == 0) {
    printf("* SF11 selected.\n");
    sf = 11;
  } else if (strcmp(buf, "12") == 0) {
    printf("* SF12 selected.\n");
    sf = 12;
  } else {
    printf("* Unknown SF\n");
    askSF();
    return;
  }
  askCR();
}

static void inputModem(SerialPort &);
static void askModem() {
  printf("Select modem (0: LoRa, 1:FSK) [0]:");
  Serial.onReceive(inputModem);
  Serial.inputKeyboard(buf, sizeof(buf));
}

static void inputModem(SerialPort &) {
  if (strlen(buf) == 0 || strcmp(buf, "0") == 0) {
    printf("* LoRa selected.\n");
    modem = 0;
    askSF();
  } else if (strcmp(buf, "1") == 0) {
    printf("* FSK selected.\n");
    modem = 1;
    appStart();
  } else {
    printf("* Unknown modem\n");
    askModem();
  }
}

void setup(void) {
  Serial.begin(115200);
  printf("\n*** SX1276 Low-Level Rx Control Example ***\n");

#if 1
  Serial.listen();
  askModem();
#else
  modem = 0;
  sf = 12;
  cr = 4;
  bw = 0;
  iq = true;
  syncword = 0x12;
  appStart();
#endif
}
