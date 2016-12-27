#include <cox.h>
#include "pitches.h"

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

uint8_t indexNote = 0;
Timer t;

void taskPlayNote(void *);

void taskIncrementIndex(void *) {
  noTone(GPIO7);

  // to distinguish the notes, set a minimum time between them.
  // the note's duration + 30% seems to work well:
  t.onFired(taskPlayNote, NULL);

  indexNote++;
  if (indexNote >= 8) {
    indexNote = 0;
    t.startOneShot(1000);
  } else {
    t.startOneShot(1000 / noteDurations[indexNote] * 1.3);
  }
}

void taskPlayNote(void *) {
  tone(GPIO7, melody[indexNote]);

  // to calculate the note duration, take one second
  // divided by the note type.
  //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
  t.onFired(taskIncrementIndex, NULL);
  t.startOneShot(1000 / noteDurations[indexNote]);
}

void setup() {
  Serial.begin(115200);
  printf("\n[PLM100] Tone Test\n");

  pinMode(GPIO7, OUTPUT);

  postTask(taskPlayNote, NULL);
}
