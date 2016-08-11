#include <cox.h>

LoRaMac *LoRaWAN;
Timer timerSend;

#define OVER_THE_AIR_ACTIVATION 0

#if (OVER_THE_AIR_ACTIVATION == 1)
static const uint8_t devEui[] = "\xC2\xAE\x00\x00\x00\x00\x80\x00";
static const uint8_t appEui[] = "\x1A\x00\x00\xFE\x00\x00\x00\x00";
static const uint8_t appKey[] = "\x0B\xF2\x80\x34\xED\xCB\x14\xE0\x9E\x1F\x94\xEA\x73\xE8\xEF\x0E";
#else

static uint8_t NwkSKey[] = {
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};
static uint8_t AppSKey[] = {
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};
static uint32_t DevAddr = 0;
#endif //OVER_THE_AIR_ACTIVATION

static void taskPeriodicSend(void *) {
  LoRaMacFrame *f = new LoRaMacFrame(16);
  if (!f) {
    printf("* Out of memory\n");
    return NULL;
  }

  uint16_t pressure = 0;
  int16_t altitudeBar = 0;
  int16_t temperature = 0;
  int32_t latitude = 0, longitude = 0;
  uint16_t altitudeGps = 0xFFFF;
  uint8_t batteryLevel = 0;

  f->port = 1;
  f->type = LoRaMacFrame::UNCONFIRMED;
  f->buf[0] = 0;
  f->buf[1] = (pressure >> 8) & 0xFF;
  f->buf[2] = (pressure >> 0) & 0xFF;
  f->buf[3] = (temperature >> 8) & 0xFF;
  f->buf[4] = (temperature >> 0) & 0xFF;
  f->buf[5] = (altitudeBar >> 8) & 0xFF;
  f->buf[6] = (altitudeBar >> 0) & 0xFF;
  f->buf[7] = batteryLevel;
  f->buf[8] = (latitude >> 16) & 0xFF;
  f->buf[9] = (latitude >> 8) & 0xFF;
  f->buf[10] = (latitude >> 0) & 0xFF;
  f->buf[11] = (longitude >> 16) & 0xFF;
  f->buf[12] = (longitude >> 8) & 0xFF;
  f->buf[13] = (longitude >> 0) & 0xFF;
  f->buf[14] = (altitudeGps >> 8) & 0xFF;
  f->buf[15] = (altitudeGps >> 0) & 0xFF;

  error_t err = LoRaWAN->send(f);
  printf("* Sending periodic report (%p): %d\n", f, err);
  if (err != ERROR_SUCCESS) {
    delete f;
  }
}

static void eventLoRaWANJoin( LoRaMac &,
                              bool joined,
                              const uint8_t *joinedDevEui,
                              const uint8_t *joinedAppEui,
                              const uint8_t *joinedAppKey,
                              const uint8_t *joinedNwkSKey,
                              const uint8_t *joinedAppSKey,
                              uint32_t joinedDevAddr) {
#if (OVER_THE_AIR_ACTIVATION == 1)
  if (joined) {
    // Status is OK, node has joined the network
    printf("%s()-joined\n", __func__);
    timerSend.onFired(taskPeriodicSend, NULL);
    timerSend.startPeriodic(10000);
  } else {
    printf("%s()-join failed. Retry to join\n", __func__);
    LoRaWAN->beginJoining(devEui, appEui, appKey);
  }
#endif
}

static void eventLoRaWANSendDone(LoRaMac &, LoRaMacFrame *frame, error_t result) {
  printf("* Send done(%d): [%p] destined for port[%u] ", result, frame, frame->port);
  if (frame->type == LoRaMacFrame::UNCONFIRMED) {
    printf("UNCONFIRMED");
  } else if (frame->type == LoRaMacFrame::CONFIRMED) {
    printf("CONFIRMED");
  } else if (frame->type == LoRaMacFrame::MULTICAST) {
    printf("MULTICAST");
  } else if (frame->type == LoRaMacFrame::PROPRIETARY) {
    printf("PROPRIETARY");
  } else {
    printf("unknown type");
  }
  printf(" frame\n");
  delete frame;
}

static void eventLoRaWANReceive(LoRaMac &, const LoRaMacFrame *frame) {
  printf("* Received: destined for port[%u] ", frame->port);
  if (frame->type == LoRaMacFrame::UNCONFIRMED) {
    printf("UNCONFIRMED");
  } else if (frame->type == LoRaMacFrame::CONFIRMED) {
    printf("CONFIRMED");
  } else if (frame->type == LoRaMacFrame::MULTICAST) {
    printf("MULTICAST");
  } else if (frame->type == LoRaMacFrame::PROPRIETARY) {
    printf("PROPRIETARY");
  } else {
    printf("unknown type");
  }
  printf(" frame\n");
}

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** LoRaWAN Class A Example ***\n");

  LoRaWAN = LoRaMac::CreateForKR917();

  LoRaWAN->begin(SX1276);
  LoRaWAN->onSendDone(eventLoRaWANSendDone);
  LoRaWAN->onReceive(eventLoRaWANReceive);
  LoRaWAN->onJoin(eventLoRaWANJoin);

#if (OVER_THE_AIR_ACTIVATION == 0)
  printf("ABP!\n");
  LoRaWAN->setABP(NwkSKey, AppSKey, DevAddr);
  LoRaWAN->setNetworkJoined(true);

  timerSend.onFired(taskPeriodicSend, NULL);
  timerSend.startPeriodic(10000);
#else
  printf("Trying to join\n");
  LoRaWAN->beginJoining(devEui, appEui, appKey);
#endif
}
