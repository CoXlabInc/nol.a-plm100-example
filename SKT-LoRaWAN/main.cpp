#include <cox.h>
#include <algorithm>

#define INTERVAL_SEND 20000
#define OVER_THE_AIR_ACTIVATION 1
#define INIT_CLASS_C 0
#define SKIP_PSEUDOJOIN 1

//! [How to declare LoRaMac for SKT]
#include <LoRaMacKR920SKT.hpp>

LoRaMacKR920SKT LoRaWAN(SX1276, 10);
//! [How to declare LoRaMac for SKT]

Timer timerSend;

#if (OVER_THE_AIR_ACTIVATION == 1)
static uint8_t devEui[8];
static const uint8_t appEui[] = "\x00\x00\x00\x00\x00\x00\x00\x00";
static uint8_t appKey[16];

char keyBuf[128];
Timer timerKeyInput;
#else

static const uint8_t NwkSKey[] = "\xa4\x88\x55\xad\xe9\xf8\xf4\x6f\xa0\x94\xb1\x98\x36\xc3\xc0\x86";
static const uint8_t AppSKey[] = "\x7a\x56\x2a\x75\xd7\xa3\xbd\x89\xa3\xde\x53\xe1\xcf\x7f\x1c\xc7";
static uint32_t DevAddr = 0x06e632e8;
#endif //OVER_THE_AIR_ACTIVATION

static void taskPeriodicSend(void *) {
  error_t err = LoRaWAN.requestLinkCheck();
  printf("* Request LinkCheck: %d\n", err);

  err = LoRaWAN.requestDeviceTime();
  printf("* Request DeviceTime: %d\n", err);

  printf(
    "* Max payload length for DR%u: %u - %u = %u\n",
    LoRaWAN.getCurrentDatarateIndex(),
    LoRaWAN.getMaxPayload(LoRaWAN.getCurrentDatarateIndex()),
    LoRaWAN.getPendingMacCommandsLength(),
    LoRaWAN.getMaxPayload(LoRaWAN.getCurrentDatarateIndex()) - LoRaWAN.getPendingMacCommandsLength()
  );

  LoRaMacFrame *f = new LoRaMacFrame(255);
  if (!f) {
    printf("* Out of memory\n");
    return;
  }

  f->port = 1;
  f->type = LoRaMacFrame::CONFIRMED;
  f->len = sprintf((char *) f->buf, "\"Now\":%lu", System.getDateTime());

  /* Uncomment below line to specify frequency. */
  // f->freq = 922500000;

  /* Uncomment below lines to specify parameters manually. */
  // LoRaWAN.useADR = false;
  // f->modulation = Radio::MOD_LORA;
  // f->meta.LoRa.bw = Radio::BW_125kHz;
  // f->meta.LoRa.sf = Radio::SF12;
  // f->power = 1; /* Index 1 => MaxEIRP - 2 dBm */

  /* Uncomment below line to specify number of trials. */
  // f->numTrials = 1;

  err = LoRaWAN.send(f);
  printf("* Sending periodic report (%s (%u byte)): %d\n", f->buf, f->len, err);
  if (err != ERROR_SUCCESS) {
    delete f;
    timerSend.startOneShot(INTERVAL_SEND);
  }
}

#if (OVER_THE_AIR_ACTIVATION == 1)
//! [How to use onJoin callback for SKT]
static void eventLoRaWANJoin(
  LoRaMac &,
  bool joined,
  const uint8_t *joinedDevEui,
  const uint8_t *joinedAppEui,
  const uint8_t *joinedAppKey,
  const uint8_t *joinedNwkSKey,
  const uint8_t *joinedAppSKey,
  uint32_t joinedDevAddr,
  const RadioPacket &,
  uint32_t airTime
) {
  Serial.printf("* Tx time of JoinRequest: %lu usec.\n", airTime);

  if (joinedNwkSKey && joinedAppSKey) {
    /* RealAppKey Joining */
    if (joined) {
      Serial.println("* RealAppKey joining done!");
      postTask(taskPeriodicSend, NULL);
    } else {
      Serial.println("* RealAppKey joining failed. Retry to join.");
      LoRaWAN.beginJoining(NULL, NULL, NULL);
    }
  } else {
    /* PseudoAppKey Joining */
    if (joined) {
      Serial.println("* PseudoAppKey joining done! Let's do RealAppKey joining!");
      LoRaWAN.beginJoining(NULL, NULL, NULL);
    } else {
      Serial.println("* PseudoAppKey joining failed. Retry to join.");
      LoRaWAN.beginJoining(devEui, appEui, appKey);
    }
  }
}
//! [How to use onJoin callback for SKT]
#endif //OVER_THE_AIR_ACTIVATION

static void eventLoRaWANSendDone(LoRaMac &lw, LoRaMacFrame *frame) {
  Serial.printf("* Send done(%d): ");
  frame->printTo(Serial);
  Serial.println();
  delete frame;

  timerSend.startOneShot(INTERVAL_SEND);
}

static void eventLoRaWANReceive(LoRaMac &lw, const LoRaMacFrame *frame) {
  static uint32_t fCntDownPrev = 0;

  Serial.printf("* Received a frame:");
  for (uint8_t i = 0; i < frame->len; i++) {
    Serial.printf(" %02X", frame->buf[i]);
  }
  Serial.printf(" (");
  frame->printTo(Serial);
  Serial.printf(")\n");

  if (
    (frame->type == LoRaMacFrame::CONFIRMED || lw.framePending) &&
    lw.getNumPendingSendFrames() == 0
  ) {
    // If there is no pending send frames, send an empty frame to ack or pull more frames.
    LoRaMacFrame *emptyFrame = new LoRaMacFrame(0);
    if (emptyFrame) {
      error_t err = LoRaWAN.send(emptyFrame);
      if (err != ERROR_SUCCESS) {
        delete emptyFrame;
      }
    }
  }
}
//! [How to use onReceive callback]

//! [How to use onJoinRequested callback]
static void eventLoRaWANJoinRequested(
  LoRaMac &, uint32_t frequencyHz, const LoRaMac::DatarateParams_t &dr
) {
  printf("* JoinRequested(Frequency: %lu Hz, Modulation: ", frequencyHz);
  if (dr.mod == Radio::MOD_FSK) {
    printf("FSK\n");
  } else if (dr.mod == Radio::MOD_LORA) {
    const char *strLoRaBW[] = { "UNKNOWN", "125kHz", "250kHz", "500kHz" };
    printf("LoRa, SF:%u, BW:%s)\n", dr.param.LoRa.sf, strLoRaBW[dr.param.LoRa.bw]);
  }
}
//! [How to use onJoinRequested callback]

//! [eventLoRaWANLinkADRReqReceived]
static void eventLoRaWANLinkADRReqReceived(LoRaMac &l, const uint8_t *payload) {
  printf("* LoRaWAN LinkADRReq received: [");
  for (uint8_t i = 0; i < 4; i++) {
    printf(" %02X", payload[i]);
  }
  printf(" ]\n");
}
//! [eventLoRaWANLinkADRReqReceived]

//! [eventLoRaWANLinkADRAnsSent]
static void printChannelInformation(LoRaMac &lw) {
  //! [getChannel]
  printf("* Frequency List\n");
  for (uint8_t i = 0; i < lw.MaxNumChannels; i++) {
    const LoRaMac::ChannelParams_t *p = lw.getChannel(i);
    if (p) {
      printf(" - [%u] %lu Hz\n", i, p->Frequency);
    }
  }
  //! [getChannel]

  //! [getDatarate]
  const LoRaMac::DatarateParams_t *dr = lw.getDatarate(lw.getCurrentDatarateIndex());
  printf("* Default DR: %u", lw.getCurrentDatarateIndex());
  if (dr->mod == Radio::MOD_LORA) {
    const char *strBW[] = {
      "Unknown", "125kHz", "250kHz", "500kHz", "Unexpected value"
    };
    printf("(LoRa SF%u BW:", dr->param.LoRa.sf);
    switch (dr->param.LoRa.bw) {
    case Radio::BW_125kHz: printf("125kHz\n"); break;
    case Radio::BW_250kHz: printf("250kHz\n"); break;
    case Radio::BW_500kHz: printf("500kHz\n"); break;
    default: printf("(unexpected:%u)\n", dr->param.LoRa.bw); break;
    }
  } else if (dr->mod == Radio::MOD_FSK) {
    printf("(FSK)\n");
  } else {
    printf("(Unknown modulation)\n");
  }
  //! [getDatarate]

  //! [getTxPower]
  int8_t power = lw.getTxPower(lw.getCurrentTxPowerIndex());
  printf("* Default Tx: ");
  if (power == -127) {
    printf("unexpected value\n");
  } else {
    printf("%d dBm\n", power);
  }
  //! [getTxPower]

  printf(
    " - # of repetitions of unconfirmed uplink frames: %u\n",
    lw.getNumRepetitions()
  );
}

static void eventLoRaWANLinkADRAnsSent(LoRaMac &lw, uint8_t status) {
  printf("* LoRaWAN LinkADRAns sent with status 0x%02X.\n", status);
  printChannelInformation(lw);
}
//! [eventLoRaWANLinkADRAnsSent]

//! [eventLoRaWANDutyCycleReqReceived]
static void eventLoRaWANDutyCycleReqReceived(
  LoRaMac &lw, const uint8_t *payload
) {
  printf("* LoRaWAN DutyCycleReq received: [");
  for (uint8_t i = 0; i < 1; i++) {
    printf(" %02X", payload[i]);
  }
  printf(" ]\n");
}
//! [eventLoRaWANDutyCycleReqReceived]

//! [eventLoRaWANDutyCycleAnsSent]
static void eventLoRaWANDutyCycleAnsSent(LoRaMac &lw) {
  printf(
    "* LoRaWAN DutyCycleAns sent. Current MaxDCycle is %u.\n",
    lw.getMaxDutyCycle()
  );
}
//! [eventLoRaWANDutyCycleAnsSent]

//! [eventLoRaWANRxParamSetupReqReceived]
static void eventLoRaWANRxParamSetupReqReceived(
  LoRaMac &lw,
  const uint8_t *payload
) {
  printf("* LoRaWAN RxParamSetupReq received: [");
  for (uint8_t i = 0; i < 4; i++) {
    printf(" %02X", payload[i]);
  }
  printf(" ]\n");
}
//! [eventLoRaWANRxParamSetupReqReceived]

//! [eventLoRaWANRxParamSetupAnsSent]
static void eventLoRaWANRxParamSetupAnsSent(LoRaMac &lw, uint8_t status) {
  printf(
    "* LoRaWAN RxParamSetupAns sent with status 0x%02X. "
    "Current Rx1Offset is %u, and Rx2 channel is (DR%u, %lu Hz).\n",
    status,
    lw.getRx1DrOffset(),
    lw.getRx2Datarate(),
    lw.getRx2Frequency()
  );
}
//! [eventLoRaWANRxParamSetupAnsSent]

static void eventLoRaWANDevStatusReqReceived(LoRaMac &lw) {
  printf("* LoRaWAN DevStatusReq received.\n");
}

//! [eventLoRaWANDevStatusAnsSent]
static void eventLoRaWANDevStatusAnsSent(
  LoRaMac &lw,
  uint8_t bat,
  uint8_t margin
) {
  printf("* LoRaWAN DevStatusAns sent. (");
  if (bat == 0) {
    printf("Powered by external power source. ");
  } else if (bat == 255) {
    printf("Battery level cannot be measured. ");
  } else {
    printf("Battery: %lu %%. ", map(bat, 1, 254, 0, 100));
  }

  if (bitRead(margin, 5) == 1) {
    margin |= bit(7) | bit(6);
  }

  printf(" SNR: %d)\n", (int8_t) margin);
}
//! [eventLoRaWANDevStatusAnsSent]

//! [eventLoRaWANNewChannelReqReceived]
static void eventLoRaWANNewChannelReqReceived(
  LoRaMac &lw, const uint8_t *payload
) {
  printf("* LoRaWAN NewChannelReq received [");
  for (uint8_t i = 0; i < 5; i++) {
    printf(" %02X", payload[i]);
  }
  printf(" ]\n");
}
//! [eventLoRaWANNewChannelReqReceived]

//! [eventLoRaWANNewChannelAnsSent]
static void eventLoRaWANNewChannelAnsSent(LoRaMac &lw, uint8_t status) {
  printf(
    "* LoRaWAN NewChannelAns sent with "
    "(Datarate range %s and channel frequency %s).\n",
    (bitRead(status, 1) == 1) ? "OK" : "NOT OK",
    (bitRead(status, 0) == 1) ? "OK" : "NOT OK"
  );

  printf("* Frequency List\n");
  for (uint8_t i = 0; i < lw.MaxNumChannels; i++) {
    const LoRaMac::ChannelParams_t *p = lw.getChannel(i);
    if (p) {
      printf(" - [%u] %lu Hz\n", i, p->Frequency);
    }
  }
}
//! [eventLoRaWANNewChannelAnsSent]

//! [eventLoRaWANRxTimingSetupReqReceived]
static void eventLoRaWANRxTimingSetupReqReceived(
  LoRaMac &lw,
  const uint8_t *payload
) {
  printf("* LoRaWAN RxTimingSetupReq received:  [");
  for (uint8_t i = 0; i < 1; i++) {
    printf(" %02X", payload[i]);
  }
  printf(" ]\n");
}
//! [eventLoRaWANRxTimingSetupReqReceived]

//! [eventLoRaWANRxTimingSetupAnsSent]
static void eventLoRaWANRxTimingSetupAnsSent(LoRaMac &lw) {
  printf(
    "* LoRaWAN RxTimingSetupAns sent. "
    "Current Rx1 delay is %u msec, and Rx2 delay is %u msec.\n",
    lw.getRx1Delay(),
    lw.getRx2Delay()
  );
}
//! [eventLoRaWANRxTimingSetupAnsSent]

//! [How to use onLinkChecked callback]
static void eventLoRaWANLinkChecked(
  LoRaMac &lw,
  uint8_t demodMargin,
  uint8_t numGateways
) {
  if (numGateways > 0) {
    printf(
      "* LoRaWAN link checked. Demodulation margin: %u dB, # of gateways: %u\n",
      demodMargin, numGateways
    );
  } else {
    printf("* LoRaWAN link check failed.\n");
  }
}
//! [How to use onLinkChecked callback]

//! [How to use onDeviceTimeAnswered callback]
static void eventLoRaWANDeviceTimeAnswered(
  LoRaMac &lw,
  bool success,
  uint32_t tSeconds,
  uint8_t tFracSeconds
) {
  if (success) {
    struct tm tLocal, tUtc;
    System.getDateTime(tLocal);
    System.getUTC(tUtc);
    printf(
      "* LoRaWAN DeviceTime answered: (%lu + %u/256) GPS epoch.\n"
      "- Adjusted local time: %u-%u-%u %02u:%02u:%02u\n"
      "- Adjusted UTC time: %u-%u-%u %02u:%02u:%02u\n",
      tSeconds, tFracSeconds,
      tLocal.tm_year + 1900, tLocal.tm_mon + 1, tLocal.tm_mday,
      tLocal.tm_hour, tLocal.tm_min, tLocal.tm_sec,
      tUtc.tm_year + 1900, tUtc.tm_mon + 1, tUtc.tm_mday,
      tUtc.tm_hour, tUtc.tm_min, tUtc.tm_sec
    );
  } else {
    printf("* LoRaWAN DeviceTime not answered\n");
  }
}
//! [How to use onDeviceTimeAnswered callback]

//! [How to use onDeviceModeConfigured callback]
static void eventLoRaWANDeviceModeConfigured(LoRaMac &lw, bool success, LoRaMac::DeviceClass_t newClass) {
  if (success) {
    printf("* LoRaWAN DeviceMode configured to ");
    if (newClass == LoRaMac::CLASS_A) {
      printf("A");
    } else if (newClass == LoRaMac::CLASS_C) {
      printf("C");
    } else {
      printf("unknown");
    }
    printf(" by the network.\n");
  } else {
    printf("* LoRaWAN network server is not responsing DeviceModeConf.\n");
  }
}
//! [How to use onDeviceModeConfigured callback]

static void eventLoRaWANDeviceModeIndSent(LoRaMac &lw, LoRaMac::DeviceClass_t newClass) {
  printf("* LoRaWAN DeviceModeInd sent. [0x%02X]\n", (uint8_t) newClass);
}

static void eventButtonPressed() {
  printf("* Button pressed:\n");

  error_t err;
  if (LoRaWAN.getDeviceClass() == LoRaMac::CLASS_A) {
    printf("- Change class A to C:");
    err = LoRaWAN.setDeviceClass(LoRaMac::CLASS_C, true, eventLoRaWANDeviceModeIndSent);
  } else {
    printf("- Change class C to A:");
    err = LoRaWAN.setDeviceClass(LoRaMac::CLASS_A, true, eventLoRaWANDeviceModeIndSent);
  }

  printf("%d\n", err);
}

#if (OVER_THE_AIR_ACTIVATION == 1)

static void taskBeginJoin(void *) {
  Serial.stopListening();

#if (SKIP_PSEUDOJOIN == 0)
  Serial.println("* Let's start PseudoAppKey join!");
  //! [SKT PseudoAppKey joining]
  LoRaWAN.setNetworkJoined(false);
  LoRaWAN.beginJoining(devEui, appEui, appKey);
  //! [SKT PseudoAppKey joining]
#else
  Serial.println("* Let's start RealAppKey join!");
  //! [SKT RealAppKey joining]
  LoRaWAN.setNetworkJoined(true);
  LoRaWAN.beginJoining(devEui, appEui, appKey);
  //! [SKT RealAppKey joining]
#endif
}

static void eventAppKeyInput(SerialPort &) {
  uint8_t numOctets = strlen(keyBuf);
  if (numOctets == 32) {
    numOctets /= 2;
    char strOctet[3];

    for (uint8_t j = 0; j < numOctets; j++) {
      strOctet[0] = keyBuf[2 * j];
      strOctet[1] = keyBuf[2 * j + 1];
      strOctet[2] = '\0';

      appKey[j] = strtoul(strOctet, NULL, 16);
    }

    printf("* New AppKey:");
    for (uint8_t j = 0; j < numOctets; j++) {
      printf(" %02X", appKey[j]);
    }
    printf(" (%u byte)\n", numOctets);
    ConfigMemory.write(appKey, 0, numOctets);
    postTask(taskBeginJoin, NULL);
    return;
  } else {
    printf("* HEX string length MUST be 32-byte.");
  }

  Serial.inputKeyboard(keyBuf, sizeof(keyBuf) - 1);
}

static void eventKeyInput(SerialPort &) {
  timerKeyInput.stop();
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.onReceive(eventAppKeyInput);
  Serial.inputKeyboard(keyBuf, sizeof(keyBuf) - 1);
  Serial.println(
    "* Enter a new appKey as a hexadecimal string "
    "[ex. 00112233445566778899aabbccddeeff]"
  );
}

#endif //OVER_THE_AIR_ACTIVATION

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** [PLM100] LoRaWAN Class A&C Example ***\n");

  // Button to switch between class A and C.
  pinMode(D9, INPUT);
  attachInterrupt(D9, eventButtonPressed, FALLING);

  System.setTimeDiff(9 * 60);  // KST

  timerSend.onFired(taskPeriodicSend, NULL);

  LoRaWAN.begin();

  //! [How to set onSendDone callback]
  LoRaWAN.onSendDone(eventLoRaWANSendDone);
  //! [How to set onSendDone callback]

  //! [How to set onReceive callback]
  LoRaWAN.onReceive(eventLoRaWANReceive);
  //! [How to set onReceive callback]

#if (OVER_THE_AIR_ACTIVATION == 1)
  //! [How to set onJoin callback]
  LoRaWAN.onJoin(eventLoRaWANJoin);
  //! [How to set onJoin callback]
#endif
  //! [How to set onJoinRequested callback]
  LoRaWAN.onJoinRequested(eventLoRaWANJoinRequested);
  //! [How to set onJoinRequested callback]

  //! [How to set onLinkChecked callback]
  LoRaWAN.onLinkChecked(eventLoRaWANLinkChecked);
  //! [How to set onLinkChecked callback]

  //! [How to set onDeviceTimeAnswered callback]
  LoRaWAN.onDeviceTimeAnswered(eventLoRaWANDeviceTimeAnswered, &System);
  //! [How to set onDeviceTimeAnswered callback]

  LoRaWAN.onDeviceModeConfigured(eventLoRaWANDeviceModeConfigured);

  LoRaWAN.onLinkADRReqReceived(eventLoRaWANLinkADRReqReceived);
  LoRaWAN.onLinkADRAnsSent(eventLoRaWANLinkADRAnsSent);
  LoRaWAN.onDutyCycleReqReceived(eventLoRaWANDutyCycleReqReceived);
  LoRaWAN.onDutyCycleAnsSent(eventLoRaWANDutyCycleAnsSent);
  LoRaWAN.onRxParamSetupReqReceived(eventLoRaWANRxParamSetupReqReceived);
  LoRaWAN.onRxParamSetupAnsSent(eventLoRaWANRxParamSetupAnsSent);
  LoRaWAN.onDevStatusReqReceived(eventLoRaWANDevStatusReqReceived);
  LoRaWAN.onDevStatusAnsSent(eventLoRaWANDevStatusAnsSent);
  LoRaWAN.onNewChannelReqReceived(eventLoRaWANNewChannelReqReceived);
  LoRaWAN.onNewChannelAnsSent(eventLoRaWANNewChannelAnsSent);
  LoRaWAN.onRxTimingSetupReqReceived(eventLoRaWANRxTimingSetupReqReceived);
  LoRaWAN.onRxTimingSetupAnsSent(eventLoRaWANRxTimingSetupAnsSent);

  LoRaWAN.setPublicNetwork(false);

  printChannelInformation(LoRaWAN);

#if (OVER_THE_AIR_ACTIVATION == 0)
  printf("ABP!\n");
  LoRaWAN.setABP(NwkSKey, AppSKey, DevAddr);
  LoRaWAN.setNetworkJoined(true);
  LoRaWAN.setCurrentDatarateIndex(5);

  postTask(taskPeriodicSend, NULL);
#else
  System.getEUI(devEui);
  Serial.printf(
    "* DevEUI: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
    devEui[0], devEui[1], devEui[2], devEui[3],
    devEui[4], devEui[5], devEui[6], devEui[7]
  );

  ConfigMemory.read(appKey, 0, 16);
  Serial.printf(
    "* AppKey: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
    appKey[0], appKey[1], appKey[2], appKey[3],
    appKey[4], appKey[5], appKey[6], appKey[7],
    appKey[8], appKey[9], appKey[10], appKey[11],
    appKey[12], appKey[13], appKey[14], appKey[15]
  );

  if (memcmp(appKey, "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 16) == 0) {
    /* The appKey is required to be entered by user.*/
    Serial.onReceive(eventAppKeyInput);
    Serial.inputKeyboard(keyBuf, sizeof(keyBuf) - 1);
    Serial.println(
      "* Enter a new appKey as a hexadecimal string "
      "[ex. 00112233445566778899aabbccddeeff]"
    );
  } else {
    Serial.println("* Press any key to enter a new appKey in 3 seconds...");
    timerKeyInput.onFired(taskBeginJoin, NULL);
    timerKeyInput.startOneShot(3000);
    Serial.onReceive(eventKeyInput);
  }

  Serial.listen();

#endif //OVER_THE_AIR_ACTIVATION

  pinMode(D4, OUTPUT);
  digitalWrite(D4, LOW);
}
