// -*- indent-tabs-mode:nil; -*-

#include "cox.h"

static void received(IEEE802_15_4Mac &radio, const IEEE802_15_4Frame *frame);
static void receivedProbe(uint16_t panId,
                          const uint8_t *eui64,
                          uint16_t shortId,
                          int16_t rssi,
                          const uint8_t *payload,
                          uint8_t payloadLen,
                          uint32_t channel);

static void ledOnTask(void *);
static void ledOffTask(void *);

uint16_t node_id = 2;
uint8_t node_ext_id[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0, 0};

LPPMac *Lpp;
Timer ledTimer;

void setup(void) {
  Serial.begin(115200);
  printf("\n*** [PLM100] LPP Receiver ***\n");

  SX1276.begin();
  SX1276.setDataRate(7);
  SX1276.setCodingRate(Radio::CR_4_5);
  SX1276.setTxPower(20);
  SX1276.setChannel(917300000);

  node_ext_id[6] = highByte(node_id);
  node_ext_id[7] = lowByte(node_id);

  Lpp = LPPMac::Create();
  Lpp->begin(SX1276, 0x1234, 0xFFFF, node_ext_id);
  Lpp->setProbePeriod(3000);
  Lpp->setListenTimeout(3300);
  Lpp->setTxTimeout(632);
  Lpp->setRxTimeout(465);
  Lpp->setRxWaitTimeout(30);
  Lpp->setRadioAlwaysOn(true);

  Lpp->onReceive(received);
  Lpp->onReceiveProbe(receivedProbe);

  pinMode(GPIO1, OUTPUT);
}

static void ledOnTask(void *) {
  ledTimer.onFired(ledOffTask, NULL);
  ledTimer.startOneShot(10);
  digitalWrite(GPIO1, HIGH);
}

static void ledOffTask(void *) {
  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(990);
  digitalWrite(GPIO1, LOW);
}

static void received(IEEE802_15_4Mac &radio, const IEEE802_15_4Frame *frame) {
  uint8_t i;
  const uint8_t *payload;

  payload = (const uint8_t *) frame->getPayloadPointer();

  // ledToggle(3);
  if (frame->srcAddr.len == 2) {
    printf("RX : %x, \t", frame->srcAddr.id.s16);
  } else if (frame->srcAddr.len == 8) {
    printf("RX : %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x, \t",
           frame->srcAddr.id.s64[0],
           frame->srcAddr.id.s64[1],
           frame->srcAddr.id.s64[2],
           frame->srcAddr.id.s64[3],
           frame->srcAddr.id.s64[4],
           frame->srcAddr.id.s64[5],
           frame->srcAddr.id.s64[6],
           frame->srcAddr.id.s64[7]);
  }
  printf("RSSI(%d),\tSNR(%d)\t%02x %02x~ (length:%u)",
         frame->power,
         frame->meta.LoRa.snr,
         payload[0],
         payload[1],
         frame->getPayloadLength());

  for (i = 0; i < frame->getPayloadLength(); i++)
    printf(" %02X", payload[i]);
  printf("\n");
}

static void receivedProbe(uint16_t panId,
                          const uint8_t *eui64,
                          uint16_t shortId,
                          int16_t rssi,
                          const uint8_t *payload,
                          uint8_t payloadLen,
                          uint32_t channel) {
  printf("* Probe received from PAN:0x%04X, "
        "Node EUI64:%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x, "
        "ID:%x, RSSI:%d",
        panId,
        eui64[0], eui64[1], eui64[2], eui64[3],
        eui64[4], eui64[5], eui64[6], eui64[7],
        shortId, rssi);

  if (payloadLen > 0) {
    uint8_t i;

    printf(", length:%u, ", payloadLen);
    for (i = 0; i < payloadLen; i++) {
      printf("%02X ", payload[i]);
    }
  }

  printf("\n");
}
