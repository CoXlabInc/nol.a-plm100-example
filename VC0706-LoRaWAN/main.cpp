#include <cox.h>
#include <LoRaMacKR920.hpp>
#include <dev/Adafruit_VC0706.hpp>
#include <algorithm>
#include "LoRaMacFragmentFrame.hpp"

#define DO_NOT_EDIT_APPKEY

LoRaMacKR920 LoRaWAN(SX1276, 10);
Adafruit_VC0706 cam(Serial2);
Timer timerSnap;
char keyBuf[128];
Timer timerKeyInput;
Timer timerTimeout;

static uint8_t devEui[8];
static const uint8_t appEui[] = "\x00\x00\x00\x00\x00\x00\x00\x00";
static uint8_t appKey[16];
static uint16_t imgSize = 0;
static uint8_t reassemblyId[4];
static uint8_t *reassemblyBitmap = nullptr;
static uint8_t reassemblyBitmapLength = 0;

enum {
  STATE_IDLE,
  STATE_START_FRAG,
  STATE_SEND_FRAGS,
  STATE_CHECK_FRAG,
} state = STATE_IDLE;

enum MsgType_t {
  MSG_NEW_REASSEMBLY = 0,
  MSG_REASSEMBLY_DONE = 1,
  MSG_REASSEMBLY_BITMAP = 2,
};

static void initState() {
  timerTimeout.stop();
  if (reassemblyBitmap) {
    delete reassemblyBitmap;
    reassemblyBitmap = nullptr;
  }
  reassemblyBitmapLength = 0;
  state = STATE_IDLE;
  imgSize = 0;
}

static int32_t getNextOffset(uint8_t *maxlen) {
  int32_t start = -1;
  uint16_t len = 0;
  printf("* Finding offset to send next...\n");
  for (uint8_t i = 0; i < reassemblyBitmapLength; i++) {
    bool isLastFrag = false;
    for (uint8_t j = 0; j < 8; j++) {
      uint16_t fragOffset = (i * 8 + j) * 32;
      isLastFrag = (fragOffset + 32 >= imgSize);
      uint8_t fragLen = isLastFrag ? (imgSize - fragOffset) : 32;

      if (bitRead(reassemblyBitmap[i], 7 - j) == 0) {
        if (start < 0) {
          start = fragOffset;
        }

        if (len + fragLen > *maxlen) {
          *maxlen = len;
          return start;
        } else {
          len += fragLen;
          bitSet(reassemblyBitmap[i], 7 - j);
        }
      } else {
        if (start >= 0) {
          break;
        }

        if (isLastFrag) {
          *maxlen = len;
          return start;
        }
      }
    }
  }

  *maxlen = len;
  return start;
}

static void takePicture(void *) {
  cam.resumeVideo();
  printf("- Snapping...\n");

  if (!cam.takePicture()) {
    printf("- Failed to snap!\n");
    postTask(takePicture);
    return;
  }

  imgSize = cam.frameLength();
  printf("- Picture taken! (%u byte)\n", imgSize);
  if (imgSize == 0) {
    printf("- Failed to snap!\n");
    postTask(takePicture);
    return;
  }

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
  timerTimeout.startOneShot(60000);
}

static int32_t sendFragment() {
  // Decide how many 32-byte chunks should be transferred at once.
  printf(
    "- Max payload length for DR%u: %u\n",
    LoRaWAN.getCurrentDatarateIndex(),
    LoRaWAN.getMaxPayload(LoRaWAN.getCurrentDatarateIndex())
  );

  state = STATE_SEND_FRAGS;

  uint8_t bytesToRead = (
    (LoRaWAN.getMaxPayload(LoRaWAN.getCurrentDatarateIndex()) - 1 /* MHDR */ - 6 /* Frag overhead */)
    / 32 * 32
  );
  int32_t nextOffset = getNextOffset(&bytesToRead);
  if (nextOffset < 0) {
    printf("- Nothing to send:");
    for (uint8_t i = 0; i < reassemblyBitmapLength; i++) {
      printf(" %02X", reassemblyBitmap[i]);
    }
    printf("\n");
    return nextOffset;
  }

  printf(
    "- Send fragment (%u~%u / %u)\n",
    (uint16_t) nextOffset, ((uint16_t) nextOffset + bytesToRead), imgSize
  );

  LoRaMacFragmentFrame *f = new LoRaMacFragmentFrame(reassemblyId, (uint16_t) nextOffset, bytesToRead);
  if (!f) {
    printf("- Out of memory\n");
    timerTimeout.stop();
    initState();
    timerSnap.startOneShot(5000);
    return -1;
  }

  f->type = LoRaMacFrame::PROPRIETARY;

  uint8_t readCam = 0;
  while (readCam < bytesToRead) {
    bool verified = false;
    uint8_t r = std::min(64, bytesToRead - readCam);

    for (uint8_t i = 0; i < 2; i++) {
      const uint8_t *buf;
      do {
        printf("- (%u) Reading picture...\n", readCam);
        buf = cam.readPicture(r, nextOffset + readCam);
      } while (!buf);

      if (i == 0) {
        f->setBuffer(readCam, buf, r);
      } else if (i == 1) {
        if (memcmp(f->getBuffer(readCam), buf, r) == 0) {
          verified = true;
        }
      }
    }

    if (verified) {
      readCam += r;
    }
  }

  error_t err = LoRaWAN.send(f);
  printf("- Sending frag #%u: %d\n", ((uint16_t) nextOffset / 32), err);
  if (err != ERROR_SUCCESS) {
    delete f;
    timerTimeout.stop();
    initState();
    timerSnap.startOneShot(5000);
    return -1;
  }

  return nextOffset;
}

static void taskBeginJoin(void *) {
  Serial.stopListening();
  printf("- Let's start join!\n");
  LoRaWAN.beginJoining(devEui, appEui, appKey);
}

#ifndef DO_NOT_EDIT_APPKEY
static void eventAppKeyInput(SerialPort &) {
  uint8_t numOctets = strlen(keyBuf);
  if (numOctets == 32) {
    numOctets /= 2;
    char strOctet[3];

    for (uint8_t j = 0; j < numOctets; j++) {
      strOctet[0] = keyBuf[2 * j];
      strOctet[1] = keyBuf[2 * j + 1];
      strOctet[2] = '\0';

      appKey[j] = strtoul(strOctet, nullptr, 16);
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
#endif //!DO_NOT_EDIT_APPKEY

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

    bool success = (frame->result == RadioPacket::SUCCESS);
    delete frame;

    if (state == STATE_START_FRAG || state == STATE_CHECK_FRAG) {
      if (success) {
        LoRaMacFrame *emptyFrame = new LoRaMacFrame(0);
        if (emptyFrame) {
          error_t err = LoRaWAN.send(emptyFrame);
          if (err != ERROR_SUCCESS) {
            printf("- Error on sending empty frame (%d)\n", err);
            delete emptyFrame;
            timerTimeout.stop();
            initState();
            timerSnap.startOneShot(5000);
          }
        }
      } else {
        printf("- Send failed\n");
        initState();
        postTask(takePicture, nullptr);
      }
    } else if (state == STATE_SEND_FRAGS && frame->type == LoRaMacFrame::PROPRIETARY) {
      if (sendFragment() < 0) {
        printf("- Sending image done.\n");

        LoRaMacFrame *f = new LoRaMacFrame(255);
        if (!f) {
          printf("- Out of memory\n");
          initState();
          timerSnap.startOneShot(5000);
          return;
        }

        f->port = 223;
        f->type = LoRaMacFrame::CONFIRMED;
        f->len = sprintf(
          (char *) f->buf,
          "\"type\":\"fragcheck\","
          "\"reassemblyId\":\"%02x%02x%02x%02x\"",
          reassemblyId[0], reassemblyId[1], reassemblyId[2], reassemblyId[3]
        );

        error_t err = LoRaWAN.send(f);
        printf("- Sending frag check [ %s ] (%u byte): %d\n", (char *) f->buf, f->len, err);
        if (err != ERROR_SUCCESS) {
          delete f;
          initState();
          timerSnap.startOneShot(5000);
          return;
        }

        timerTimeout.stop();
        state = STATE_CHECK_FRAG;
        timerTimeout.startOneShot(60000);
      }
    }
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
        if (frame->len == 1 + 4 && frame->buf[0] == MSG_NEW_REASSEMBLY) {
          timerTimeout.stop();
          reassemblyId[0] = frame->buf[1];
          reassemblyId[1] = frame->buf[2];
          reassemblyId[2] = frame->buf[3];
          reassemblyId[3] = frame->buf[4];
          printf(
            "- Reassembly ID: %02X %02X %02X %02X\n",
            reassemblyId[0], reassemblyId[1], reassemblyId[2], reassemblyId[3]
          );

          reassemblyBitmapLength = (imgSize / (32 * 8)) + ((imgSize % (32 * 8) == 0) ? 0 : 1);
          reassemblyBitmap = new uint8_t[reassemblyBitmapLength];
          if (!reassemblyBitmap) {
            printf("- Not enough memory for reassemblyBitmap\n");
            initState();
            timerSnap.startOneShot(5000);
          }
          memset(reassemblyBitmap, 0, reassemblyBitmapLength);
          sendFragment();
        } else {
          printf("- In START_FRAG state, only waits for MSG_NEW_REASSEMBLY.");
        }
      } else if (
        state == STATE_CHECK_FRAG && frame->len >= 5
        && frame->buf[1] == reassemblyId[0]
        && frame->buf[2] == reassemblyId[1]
        && frame->buf[3] == reassemblyId[2]
        && frame->buf[4] == reassemblyId[3]
      ) {
        if (frame->buf[0] == MSG_REASSEMBLY_DONE && frame->len == 5) {
          printf("- All reassembly done.\n");
          initState();
          postTask(takePicture, nullptr);
        } else if (frame->buf[0] == MSG_REASSEMBLY_BITMAP) {
          if (reassemblyBitmapLength != frame->len - 5) {
            printf("- BITMAP LENGTH is DIFFERENT from my expect!!\n");
            initState();
            timerSnap.startOneShot(5000);
            return;
          }

          memcpy(reassemblyBitmap, &frame->buf[5], reassemblyBitmapLength);
          sendFragment();
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

#ifdef DO_NOT_EDIT_APPKEY
  postTask(taskBeginJoin, nullptr);
#else //!DO_NOT_EDIT_APPKEY
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
#endif //!DO_NOT_EDIT_APPKEY
  timerSnap.onFired(takePicture, nullptr);

  timerTimeout.onFired([](void *) {
    printf("- Reply timeout. Re-snap after 5 seconds.\n");
    initState();
    timerSnap.startOneShot(5000);
  }, nullptr);
}
