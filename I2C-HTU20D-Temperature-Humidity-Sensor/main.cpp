#include <cox.h>
#include <HTU20D.hpp>

static void ledOnTask(void *);
static void ledOffTask(void *);
static void sensorTemperatureRequest(void *);
static void sensorTemperatureRead(uint16_t val);
static void sensorHumidityRequest(void *);
static void sensorHumidityRead(uint16_t val);

Timer ledTimer;
Timer sensorTimer;
HTU20D htu20d;

void setup(void) {
  Serial.begin(115200);
  printf("\n*** [PLM100] HTU20D Temperature & Humidity Sensor Test ***\n");

  Wire.begin();
  htu20d.begin(Wire);
  htu20d.onTemperatureReadDone(sensorTemperatureRead);
  htu20d.onHumidityReadDone(sensorHumidityRead);

  ledTimer.onFired(ledOnTask, NULL);
  ledTimer.startOneShot(990);

  postTask(sensorTemperatureRequest, NULL);

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

static void sensorTemperatureRequest(void *) {
  htu20d.readTemperature();
}

static void sensorTemperatureRead(uint16_t val) {
  printf("Temperature: %u.%02u Celcius degree\n", val / 100, val % 100);

  sensorTimer.onFired(sensorHumidityRequest, NULL);
  sensorTimer.startOneShot(1000);
}

static void sensorHumidityRequest(void *) {
  htu20d.readHumidity();
}

static void sensorHumidityRead(uint16_t val) {
  printf("Relative Humidity: %u.%02u %%\n", val / 100, val % 100);

  sensorTimer.onFired(sensorTemperatureRequest, NULL);
  sensorTimer.startOneShot(1000);
}
