// -*- mode:c++; indent-tabs-mode:nil; -*-

#include <cox.h>

Timer ledTimer;
Timer printTimer;

static void ledOffTask(void *args);

static void ledOnTask(void *args) {
  ledTimer.onFired(ledOffTask, NULL);
  ledTimer.startOneShot(10);
  digitalWrite(GPIO1, HIGH);
  digitalToggle(GPIO2);
}

static void ledOffTask(void *args) {
  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(990);
  digitalWrite(GPIO1, LOW);
}

static const char *weekday[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };
static void printTask(void *args) {
  Serial.write("0123456789\r\n");
  printf("[%lu usec] Timer works!\n", micros());

  struct tm t;
  System.getDateTime(t);
  printf(
    "Now: %u-%u-%u %s %02u:%02u:%02u\n",
    t.tm_year + 1900,
    t.tm_mon + 1,
    t.tm_mday,
    weekday[t.tm_wday],
    t.tm_hour,
    t.tm_min,
    t.tm_sec
  );
  printf("Supply voltage: %ld mV\n", System.getSupplyVoltage());

  Serial2.write("0123456789\r\n");
  Serial2.printf("[%lu usec] Timer works!\n", micros());
}

static void eventDateTimeAlarm() {
  struct tm t;
  System.getDateTime(t);
  printf("* ALarm! Now: %u-%u-%u %s %02u:%02u:%02u\n", t.tm_year, t.tm_mon, t.tm_mday, weekday[t.tm_wday], t.tm_hour, t.tm_min, t.tm_sec);
}

static void buttonPressed() {
  // digitalToggle(GPIO1);
  printf("[%lu usec] Button works!\n", micros());

  // if (Serial.isBegan()) {
  //   printf("* Serial is turned off.\n");
  //   Serial.end();
  // } else {
  //   Serial.begin(115200);
  //   printf("* Serial is turned on.\n");
  // }
  // digitalToggle(GPIO1);
}

static void eventSerialRx(SerialPort &p) {
  while (p.available() > 0) {
    char c = p.read();
    if (c == 'q') {
      reboot();
    } else {
      p.write(c);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.onReceive(eventSerialRx);
  Serial.listen();

  Serial2.begin(9600);
  Serial2.onReceive(eventSerialRx);
  Serial2.listen();

  printf("\n*** [PLM100] Basic Functions ***\n");

  uint8_t eui[8];
  System.getEUI(eui);
  Serial.printf(
    "* EUI-64: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
    eui[0], eui[1], eui[2], eui[3], eui[4], eui[5], eui[6], eui[7]
  );

  System.setTimeDiff(9 * 60); //KST
  struct tm t;
  t.tm_year = 2017 - 1900;
  t.tm_mon = 8 - 1;
  t.tm_mday = 24;
  t.tm_hour = 18;
  t.tm_min = 37;
  t.tm_sec = 50;
  System.setDateTime(t);
  System.onDateTimeAlarm(eventDateTimeAlarm);

  System.setTimeAlarm(18, 38);

  ledTimer.onFired(ledOffTask, NULL);
  ledTimer.startOneShot(1000);

  printTimer.onFired(printTask, NULL);
  printTimer.startPeriodic(1000);

  pinMode(GPIO5, INPUT_PULLUP);
  attachInterrupt(GPIO5, buttonPressed, FALLING);

  pinMode(GPIO1, OUTPUT);
  digitalWrite(GPIO1, HIGH);
  pinMode(GPIO2, OUTPUT);
  digitalWrite(GPIO2, HIGH);
}
