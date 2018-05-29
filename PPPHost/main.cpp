#include <cox.h>
#include <IPv6.hpp>
#include <IPv6PPPoS.hpp>
#include <UDP.hpp>

IPv6 ipv6;
IPv6PPPoS ppp(Serial2);
Timer ledTimer;

static void ledOffTask(void *args);

static void ledOnTask(void *args) {
  ledTimer.onFired(ledOffTask, NULL);
  ledTimer.startOneShot(10);
  digitalWrite(GPIO1, HIGH);
}

static void ledOffTask(void *args) {
  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(990);
  digitalWrite(GPIO1, LOW);
}

static void EventDatagramReceived(IPv6Interface &input,
                                   const IPv6Address *srcAddr,
                                   const IPv6Address *dstAddr,
                                   uint16_t srcPort,
                                   uint16_t dstPort,
                                   const uint8_t *msg,
                                   uint16_t len) {
  printf("* Datagram received:");
  for (uint16_t i = 0; i < len; i++) {
    printf(" %02X", msg[i]);
  }
  printf("\n");
}

static void eventIPv6StateChanged(IPv6Interface &ipv6Inf, IPv6Interface::State_t state) {
  printf("IPv6 iface 'ppp0': State changed to %s\n",
         ipv6Inf.GetStateString(state));
}

void setup(void) {
  Serial.begin(115200);
  printf("\n\n*** [PLM100] PPP Host ***\n");

  // Initialize the PPP interface.
  Serial2.begin(115200);
  Serial2.listen();
  ppp.begin();
  ppp.setStateNotifier(eventIPv6StateChanged);
  ipv6.begin();

  IPv6Address myPrefix;
  myPrefix.fromString("fd00::");
  ppp.setPrefix(
    myPrefix,
    64,
    0xFFFFFFFFUL,
    0xFFFFFFFFUL,
    false
  );

  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(1000);

  pinMode(GPIO1, OUTPUT);
  digitalWrite(GPIO1, HIGH);

  Udp.listen(10100, EventDatagramReceived);
}
