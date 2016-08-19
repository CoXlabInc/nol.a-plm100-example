// -*- mode:c++; indent-tabs-mode:nil; -*-

#include <cox.h>

Timer ledTimer;
Timer printTimer;

SerialPort *Serial2;

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

static void printTask(void *args) {
  printf("[%lu usec] Timer works!\n", micros());

  uint16_t year;
  uint8_t month, day, hour, minute, second;
  System.getDateTime(&year, &month, &day, &hour, &minute, &second);
  printf("Now: %u-%u-%u %02u:%02u:%02u\n", year, month, day, hour, minute, second);
  printf("Supply voltage: %ld mV\n", System.getSupplyVoltage());

  Serial2->printf("[%lu usec] Timer works!\n", micros());
}

static void buttonPressed() {
  ledToggle(0);
  printf("[%lu usec] Button works!\n", micros());

  if (Serial.isBegan()) {
    printf("* Serial is turned off.\n");
    Serial.end();
  } else {
    Serial.begin(115200);
    printf("* Serial is turned on.\n");
  }
  ledToggle(0);
}

void setup() {
  Serial.begin(115200);

  Serial2 = System.enableSerialUCA0();
  Serial2->begin(57600);

  printf("\n*** [PLM100] Basic Functions ***\n");
  System.setDateTime(2016, 7, 22, 10, 0, 0);

  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(1000);

  printTimer.onFired(printTask, NULL);
  printTimer.startPeriodic(1000);

  pinMode(GPIO5, INPUT);
  attachInterrupt(GPIO5, buttonPressed, FALLING);
}
