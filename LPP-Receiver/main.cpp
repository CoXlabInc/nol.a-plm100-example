#include <cox.h>
#include <LPPMac.hpp>

static void received(IEEE802_15_4Mac &radio, const IEEE802_15_4Frame *frame);
static void receivedProbe(uint16_t panId,
                          const uint8_t *eui64,
                          uint16_t shortId,
                          int16_t rssi,
                          const uint8_t *payload,
                          uint8_t payloadLen,
                          uint32_t channel);

uint16_t node_id = 2;
uint8_t node_ext_id[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0, 0};

LPPMac Lpp;

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

  Lpp.begin(SX1276, 0x1234, 0xFFFF, node_ext_id);
  Lpp.setProbePeriod(3000);
  Lpp.setListenTimeout(3300);
  Lpp.setTxTimeout(632);
  Lpp.setRxTimeout(465);
  Lpp.setRxWaitTimeout(30);
  Lpp.setRadioAlwaysOn(true);

  Lpp.onReceive(received);
  Lpp.onReceiveProbe(receivedProbe);
}

//![Receive]
static void received(IEEE802_15_4Mac &radio, const IEEE802_15_4Frame *frame) {
  IEEE802_15_4Address srcAddr = frame->getSrcAddr();
  if (srcAddr.len == 2) {
    printf("RX : %x, \t", srcAddr.id.s16);
  } else if (srcAddr.len == 8) {
    printf("RX : %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x, \t",
           srcAddr.id.s64[0],
           srcAddr.id.s64[1],
           srcAddr.id.s64[2],
           srcAddr.id.s64[3],
           srcAddr.id.s64[4],
           srcAddr.id.s64[5],
           srcAddr.id.s64[6],
           srcAddr.id.s64[7]);
  }
  printf("RSSI(%d),\tSNR(%d)\t%02x %02x~ (length:%u)",
         frame->power,
         frame->meta.LoRa.snr,
         frame->getPayloadAt(0),
         frame->getPayloadAt(1),
         frame->getPayloadLength());

  for (uint16_t i = 0; i < frame->getPayloadLength(); i++)
    printf(" %02X", frame->getPayloadAt(i));
  printf("\n");
}
//![Receive]

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
