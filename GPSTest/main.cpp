#include <cox.h>
#include <EVAM8M.hpp>

EVAM8M gps(Serial2);

static void eventGpsReadDone(
  uint8_t fixQuality,
  uint8_t hour,
  uint8_t minute,
  uint8_t sec,
  uint16_t subsec,
  int32_t latitude,
  int32_t longitude,
  int32_t altitude,
  uint8_t numSatellites
) {
  printf(
    "* [GPS] fixQ:%u, %02u:%02u:%02u.%02u, %c %ld.%ld, %c %ld.%ld, %ld.%ld, # of sat:%u\n",
    fixQuality,
    hour, minute, sec, subsec,
    latitude >= 0 ? 'N' : 'S', latitude / 100000, latitude % 100000,
    longitude >= 0 ? 'E' : 'W', longitude / 100000, longitude % 100000,
    altitude / 10, altitude % 10,
    numSatellites
  );
}

void setup() {
  Serial.begin(115200);
  printf("\n*** [PLM100] GPS Test ***\n");

  gps.begin();
  gps.onReadDone(eventGpsReadDone);
  gps.turnOn();
}
