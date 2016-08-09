#include <cox.h>

SerialPort &Serial2 = enableSerialUCA0();
IPv6Interface *ppp;
Timer ledTimer;

static void ledOffTask(void *args);

static void ledOnTask(void *args) {
  ledTimer.onFired(ledOffTask, NULL);
  ledTimer.startOneShot(10);
  ledOn(0);
}

static void ledOffTask(void *args) {
  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(990);
  ledOff(0);
}

static void ip6_state_changed(IPv6Interface &ip6_inf, IPv6Interface::State_t state) {
  printf("IPv6 iface 'ppp0': State changed to %s\n",
         ip6_state_string(state));
}

void setup(void) {
  Serial.begin(115200);
  printf("\n\n*** PPP Host ***\n");

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
}
