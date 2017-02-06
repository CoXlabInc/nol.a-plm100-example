#include <cox.h>

IPv6Interface *ppp;
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
                                   const IP6_ADDRESS *srcAddr,
                                   const IP6_ADDRESS *dstAddr,
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

static void ip6_state_changed(IPv6Interface &ip6_inf, IPv6Interface::State_t state) {
  printf("IPv6 iface 'ppp0': State changed to %s\n",
         ip6_state_string(state));
}

void setup(void) {
  Serial.begin(115200);
  printf("\n\n*** [PLM100] PPP Host ***\n");

  /* Single interface, no routing entry */
  ip6_init(1, 0);

  // Initialize the PPP interface.
  Serial2.begin(115200);
  Serial2.listen();
  ppp = enableIPv6PPPoS(Serial2);
  ppp->begin();
  ppp->setStateNotifier(ip6_state_changed);
  ip6_start();

  IP6_ADDRESS my_prefix;
  ip6_pton("fd00::", &my_prefix);
  ppp->setPrefix(my_prefix,
                  64,
                  0xFFFFFFFFUL,
                  0xFFFFFFFFUL,
                  false);

  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(1000);

  pinMode(GPIO1, OUTPUT);
  digitalWrite(GPIO1, HIGH);

  Udp.listen(10100, EventDatagramReceived);
}
