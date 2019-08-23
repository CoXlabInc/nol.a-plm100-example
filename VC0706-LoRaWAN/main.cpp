#include <cox.h>
#include <LoRaMacKR920.hpp>
#include <dev/Adafruit_VC0706.hpp>
#include <algorithm>

LoRaMacKR920 LoRaWAN(SX1276, 10);
Adafruit_VC0706 cam(Serial2);
Timer timerSnap;
char keyBuf[128];
Timer timerKeyInput;
Timer timerPullDownlink;

static uint8_t devEui[8];
static const uint8_t appEui[] = "\x00\x00\x00\x00\x00\x00\x00\x00";
static uint8_t appKey[16];
static uint16_t imgSize = 0;
static uint32_t imgSent = 0;

enum {
  STATE_IDLE,
  STATE_START_FRAG,
  STATE_SEND_FRAGS,
} state = STATE_IDLE;

static void takePicture(void *) {
  cam.resumeVideo();
  printf("- Snapping...\n");

  if (!cam.takePicture()) {
    printf("- Failed to snap!\n");
    timerSnap.startOneShot(5000);
    return;
  }

  imgSize = cam.frameLength();
  printf("- Picture taken! (%u byte)\n", imgSize);
  imgSent = 0;

  error_t err = LoRaWAN.requestLinkCheck();
  printf("- Request LinkCheck: %d\n", err);

  err = LoRaWAN.requestDeviceTime();
  printf("- Request DeviceTime: %d\n", err);

  printf(
    "- Max payload length for DR%u: %u - %u = %u\n",
    LoRaWAN.getCurrentDatarateIndex(),
    LoRaWAN.getMaxPayload(LoRaWAN.getCurrentDatarateIndex()),
    LoRaWAN.getPendingMacCommandsLength(),
    LoRaWAN.getMaxPayload(LoRaWAN.getCurrentDatarateIndex()) - LoRaWAN.getPendingMacCommandsLength()
  );

  LoRaMacFrame *f = new LoRaMacFrame(255);
  if (!f) {
    printf("- Out of memory\n");
    timerSnap.startOneShot(5000);
    return;
  }

  f->port = 223;
  f->type = LoRaMacFrame::CONFIRMED;
  f->len = sprintf(
    (char *) f->buf,
    "\"type\":\"image/jpeg\","
    "\"name\":\"snap\","
    "\"size\":%u,"
    "\"fPort\":1",
    imgSize
  );

  err = LoRaWAN.send(f);
  printf("- Sending frag start [ %s ] (%u byte): %d\n", (char *) f->buf, f->len, err);
  if (err != ERROR_SUCCESS) {
    delete f;
    timerSnap.startOneShot(5000);
    return;
  }

  state = STATE_START_FRAG;
}

static void taskBeginJoin(void *) {
  Serial.stopListening();
  printf("- Let's start join!\n");
  LoRaWAN.beginJoining(devEui, appEui, appKey);
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

    printf("- New AppKey:");
    for (uint8_t j = 0; j < numOctets; j++) {
      printf(" %02X", appKey[j]);
    }
    printf(" (%u byte)\n", numOctets);
    ConfigMemory.write(appKey, 0, numOctets);
    postTask(taskBeginJoin, NULL);
    return;
  } else {
    printf("- HEX string length MUST be 32-byte.\n");
  }

  Serial.inputKeyboard(keyBuf, sizeof(keyBuf) - 1);
}

void setup() {
  Serial.begin(115200);
  printf("\n*** [PLM100] LoRaWAN VC0706 Serial Camera ***\n");

  if (cam.begin()) {
    printf("- Camera found.\n");
  } else {
    printf("- Camera not found\n");
    return;
  }

  char *reply = cam.getVersion();
  if (reply) {
    printf("- Camera ver:%s\n", reply);
  } else {
    printf("- Camera not responding version.\n");
    return;
  }

  cam.setImageSize(VC0706_160x120);
  printf("- Camera image size:");
  uint8_t imageSize = cam.getImageSize();
  switch(imageSize) {
    case VC0706_640x480: printf("640x480\n"); break;
    case VC0706_320x240: printf("320x240\n"); break;
    case VC0706_160x120: printf("160x120\n"); break;
    default: printf("Unknown(%u)\n", imageSize); return;
  }

  LoRaWAN.begin([]() -> uint8_t {
    return map(System.getSupplyVoltage(), 2900, 3300, 1, 254); // measured battery level
  });

  LoRaWAN.onSendDone([](LoRaMac &lw, LoRaMacFrame *frame) {
    printf("- Send done(%d): ", frame->result);
    frame->printTo(Serial);
    Serial.println();

    delete frame;
    if (state == STATE_START_FRAG) {

      LoRaMacFrame *emptyFrame = new LoRaMacFrame(0);
      if (emptyFrame) {
        error_t err = LoRaWAN.send(emptyFrame);
        if (err != ERROR_SUCCESS) {
          printf("- Error on sending empty frame (%d)\n", err);
          delete emptyFrame;
          state = STATE_IDLE;
          timerSnap.startOneShot(5000);
        }
      }
    }

  // while (jpgSize > 0) {
  //   uint8_t bytesToRead = std::min(64, (int) jpgSize);
  //   const uint8_t *buf = cam.readPicture(bytesToRead);
  //   for (uint8_t i = 0; i < bytesToRead; i++) {
  //     printf(" %02X", buf[i]);
  //   }
  //   jpgSize -= bytesToRead;
  //   System.feedWatchdog();
  // }

    //
    //
    // switch(frame->modulation) {
    //   case Radio::MOD_LORA: {
    //     const char *strBW[] = { "Unknown", "125kHz", "250kHz", "500kHz", "Unexpected value" };
    //     if (frame->meta.LoRa.bw > 3) {
    //       frame->meta.LoRa.bw = (Radio::LoRaBW_t) 4;
    //     }
    //     printf("LoRa, SF:%u, BW:%s, ", frame->meta.LoRa.sf, strBW[frame->meta.LoRa.bw]);
    //     break;
    //   }
    //   case Radio::MOD_FSK: printf("FSK, "); break;
    //   default: printf("Unkndown modulation, "); break;
    // }
    //
    // switch (frame->type) {
    //   case LoRaMacFrame::UNCONFIRMED: printf("UNCONFIRMED"); break;
    //   case LoRaMacFrame::CONFIRMED: printf("CONFIRMED"); break;
    //   case LoRaMacFrame::MULTICAST: printf("MULTICAST (error)"); break;
    //   case LoRaMacFrame::PROPRIETARY: printf("PROPRIETARY"); break;
    //   default: printf("unknown type"); break;
    // }
    // printf(" frame\n");
    //
    // for (uint8_t t = 0; t < frame->numTrials; t++) {
    //   const char *strTxResult[] = {
    //     "not started",
    //     "success",
    //     "no ack",
    //     "air busy",
    //     "Tx timeout",
    //   };
    //   printf("- [%u] %s\n", t, strTxResult[min(frame->txResult[t], 4)]);
    // }
  });

  LoRaWAN.onReceive([](LoRaMac &lw, const LoRaMacFrame *frame) {
    printf("- Received a frame:");
    for (uint8_t i = 0; i < frame->len; i++) {
      printf(" %02X", frame->buf[i]);
    }
    printf(" (");
    frame->printTo(Serial);
    printf(")\n");

    if (frame->port == 223) {
      if (state == STATE_START_FRAG) {
        if (frame->len == 4) {
          state = STATE_SEND_FRAGS;
          uint32_t reassemblyId = (
            ((uint32_t) frame->buf[0] << 24) |
            ((uint32_t) frame->buf[1] << 16) |
            ((uint32_t) frame->buf[2] << 8) |
            ((uint32_t) frame->buf[3] << 0)
          );
          printf("- Reassembly ID: 0x%08lX\n", reassemblyId);
        } else {
          printf("- In START_FRAG state, only waits for a 4-byte reassembly ID.");
        }
      } else {

      }
    }

    if (
      (frame->type == LoRaMacFrame::CONFIRMED || lw.framePending) &&
      lw.getNumPendingSendFrames() == 0
    ) {
      printf("- More frame pending.\n");
      // If there is no pending send frames, send an empty frame to ack or pull more frames.
      LoRaMacFrame *emptyFrame = new LoRaMacFrame(0);
      if (emptyFrame) {
        error_t err = LoRaWAN.send(emptyFrame);
        if (err != ERROR_SUCCESS) {
          delete emptyFrame;
        }
      }
    }
  });

  LoRaWAN.onJoin([](
    LoRaMac &lw,
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
    Serial.printf("- Tx time of JoinRequest: %lu usec.\n", airTime);

    if (joined) {
      Serial.println("- Joining done!");
      postTask(takePicture, nullptr);
    } else {
      Serial.println("- Joining failed. Retry to join.");
      lw.beginJoining(devEui, appEui, appKey);
    }
  });

  LoRaWAN.onLinkChecked([](LoRaMac &lw, uint8_t demodMargin, uint8_t numGateways) {
    if (numGateways > 0) {
      printf(
        "- LoRaWAN link checked. Demodulation margin: %u dB, # of gateways: %u\n",
        demodMargin, numGateways
      );
    } else {
      printf("- LoRaWAN link check failed.\n");
    }
  });

  LoRaWAN.onDeviceTimeAnswered([](LoRaMac &lw, bool success, uint32_t tSeconds, uint8_t tFracSeconds) {
    if (success) {
      struct tm tLocal, tUtc;
      System.getDateTime(tLocal);
      System.getUTC(tUtc);
      printf(
        "- LoRaWAN DeviceTime answered: (%lu + %u/256) GPS epoch.\n"
        "  Adjusted local time: %u-%u-%u %02u:%02u:%02u\n"
        "  Adjusted UTC time: %u-%u-%u %02u:%02u:%02u\n",
        tSeconds, tFracSeconds,
        tLocal.tm_year + 1900, tLocal.tm_mon + 1, tLocal.tm_mday,
        tLocal.tm_hour, tLocal.tm_min, tLocal.tm_sec,
        tUtc.tm_year + 1900, tUtc.tm_mon + 1, tUtc.tm_mday,
        tUtc.tm_hour, tUtc.tm_min, tUtc.tm_sec
      );
    } else {
      printf("- LoRaWAN DeviceTime not answered\n");
    }
  }, &System);

  LoRaWAN.setPublicNetwork(false);

  System.getEUI(devEui);
  Serial.printf(
    "- DevEUI: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
    devEui[0], devEui[1], devEui[2], devEui[3],
    devEui[4], devEui[5], devEui[6], devEui[7]
  );

  ConfigMemory.read(appKey, 0, 16);
  Serial.printf(
    "- AppKey: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
    appKey[0], appKey[1], appKey[2], appKey[3],
    appKey[4], appKey[5], appKey[6], appKey[7],
    appKey[8], appKey[9], appKey[10], appKey[11],
    appKey[12], appKey[13], appKey[14], appKey[15]
  );

  if (memcmp(appKey, "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 16) == 0) {
    /* The appKey is required to be entered by user.*/
    Serial.onReceive(eventAppKeyInput);
    Serial.inputKeyboard(keyBuf, sizeof(keyBuf) - 1);
    printf("- Enter a new appKey as a hexadecimal string [ex. 00112233445566778899aabbccddeeff]\n");
  } else {
    printf("- Press any key to enter a new appKey in 3 seconds...\n");
    timerKeyInput.onFired(taskBeginJoin, NULL);
    timerKeyInput.startOneShot(3000);
    Serial.onReceive([](SerialPort &) {
      timerKeyInput.stop();
      while (Serial.available() > 0) {
        Serial.read();
      }

      Serial.onReceive(eventAppKeyInput);
      Serial.inputKeyboard(keyBuf, sizeof(keyBuf) - 1);
      printf("- Enter a new appKey as a hexadecimal string [ex. 00112233445566778899aabbccddeeff]\n");
    });
  }

  Serial.listen();

  timerSnap.onFired(takePicture, nullptr);
}
