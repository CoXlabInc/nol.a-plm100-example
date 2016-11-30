#include <cox.h>

SerialPort *Serial2;
IPv6Interface *ppp;
Timer ledTimer;
bool booted = false;

LPPMac *Lpp;
NoliterAPI &Noliter = enableNoliterLite();

static void ledOffTask(void *args);

static void ledOnTask(void *args) {
  ledTimer.onFired(ledOffTask, NULL);
  ledTimer.startOneShot(10);
  digitalWrite(GPIO1, HIGH);

  if (!booted) {
    digitalToggle(GPIO2);
  }
}

static void ledOffTask(void *args) {
  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(990);
  digitalWrite(GPIO1, LOW);

  if (booted) {
    digitalWrite(GPIO2, HIGH);
#if 0
    Noliter.send("N0", "\"test\":\"1\"");
#endif
  }
}

static void eventGatewaySetupDone() {
  booted = true;
  printf("Nol.iter setup done.\n");
}

static void ip6_state_changed(IPv6Interface &interface, IPv6Interface::State_t state) {
  printf("IPv6 iface 'ppp0': State changed to %s\n",
         ip6_state_string(state));

  if (state == IPv6Interface::STATE_HOST && !booted) {
    printf("Nol.iter start!\n");
    Noliter.onReady(eventGatewaySetupDone);
    Noliter.setGateway(interface);
  }
}

static void eventLppFrameReceived(IEEE802_15_4Mac &radio,
                                  const IEEE802_15_4Frame *frame) {
  char id[24];
  const uint8_t *payload = (const uint8_t *) frame->getPayloadPointer();

  if (frame->srcAddr.len == 2) {
    sprintf(id, "N%u", frame->srcAddr.id.s16);
  } else if (frame->srcAddr.len == 8) {
    sprintf(id, "N%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x",
            frame->srcAddr.id.s64[0],
            frame->srcAddr.id.s64[1],
            frame->srcAddr.id.s64[2],
            frame->srcAddr.id.s64[3],
            frame->srcAddr.id.s64[4],
            frame->srcAddr.id.s64[5],
            frame->srcAddr.id.s64[6],
            frame->srcAddr.id.s64[7]);
  }
  printf("* LPP RX: %s,RSSI(%d),LQI(%d),%02x %02x~ (length:%u)\n",
          id,
          frame->power,
          frame->lqi,
          payload[0],
          payload[1],
          frame->getPayloadLength());

  if (booted) {
    char *data = (char *) dynamicMalloc(frame->getPayloadLength() + 1);
    if (data) {
      memcpy(data, payload, frame->getPayloadLength());
      data[frame->getPayloadLength()] = '\0';
      Noliter.send(id, data);
      dynamicFree(data);
    } else {
      printf("* Not enough memory\n");
    }
  }
}

void setup(void) {
  Serial.begin(115200);
  printf("\n\n*** [PLM100] Nol.iter Gateway ***\n");

  /* Single interface, no routing entry */
  ip6_init(1, 0);

  // Initialize the PPP interface.
  Serial2 = System.enableSerialUCA0();
  Serial2->begin(115200);
  Serial2->listen();

  ppp = enableIPv6PPPoS(*Serial2);
  if (ppp) {
    ppp->begin();
    ppp->setStateNotifier(ip6_state_changed);
    ip6_start();
  } else {
    printf("* Error on enable PPPoS.\n");
  }

  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(1000);

  SX1276.begin();
  SX1276.setDataRate(7);
  SX1276.setCodingRate(1);
  SX1276.setTxPower(20);
  SX1276.setChannel(917500000);

  Lpp = LPPMac::Create();
  Lpp->begin(SX1276, 0x1234, 0x0001, NULL);
  Lpp->setProbePeriod(3000);
  Lpp->setListenTimeout(3300);
  Lpp->setTxTimeout(632);
  Lpp->setRxTimeout(465);
  Lpp->setRxWaitTimeout(30);
  Lpp->setRadioAlwaysOn(true);

  Lpp->onReceive(eventLppFrameReceived);

  pinMode(GPIO1, OUTPUT);
  digitalWrite(GPIO1, LOW);

  pinMode(GPIO2, OUTPUT);
  digitalWrite(GPIO2, LOW);
}
