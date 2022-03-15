#include <MIDI.h>
#include <Multiplexer4067.h> // Multiplexer CD4067 library >> https://github.com/sumotoy/Multiplexer4067
#include <ResponsiveAnalogRead.h>
#include <Thread.h> // Threads library >> https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h>

#define N_MUX 2
#define s0 2
#define s1 3
#define s2 4
#define s3 5
#define x1 A0
#define x2 A1

MIDI_CREATE_DEFAULT_INSTANCE();

ThreadController cpu;
Thread threadPotentiometers; 
Thread threadChannelMenu;

const int TIMEOUT = 300;

const byte N_BUTTONS = 10;
const byte N_POTS = 26;
const byte varThreshold = 8;
const byte N_BUTTONS_ARDUINO = 0;
const byte N_POTS_ARDUINO = 4;
const byte N_BUTTONS_PER_MUX[N_MUX] = {10};
const byte POT_ARDUINO_PIN[N_POTS_ARDUINO] = {A2, A3, A4, A5};
const byte N_POTS_PER_MUX[N_MUX] = {6, 16};
const byte POT_MUX_PIN[N_MUX][16] = {
  {10, 11, 12, 13, 14, 15},
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
};
const byte BUTTON_MUX_PIN[N_MUX][16] = {
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
  {}
};
unsigned long debounceDelay = 5;
unsigned long lastDebounceTime[N_BUTTONS] = {0};
unsigned long PTime[N_POTS] = {0};
unsigned long timer[N_POTS] = {0};

int reading = 0;
int potMin = 95;
int potMax = 970;
int buttonCState[N_BUTTONS] = {0};
int buttonPState[N_BUTTONS] = {0};
int buttonMuxThreshold = 300;
int potCState[N_POTS] = {0};
int potPState[N_POTS] = {0};
int potVar = 0;
int potMidiCState[N_POTS] = {0};
int potMidiPState[N_POTS] = {0};

boolean potMoving = true;
float snapMultiplier = 0.01;

byte CC_NUMBER = 20;

ResponsiveAnalogRead responsivePot[N_POTS] = {};

Multiplexer4067 mux[N_MUX] = {
  Multiplexer4067(s0, s1, s2, s3, x1),
  Multiplexer4067(s0, s1, s2, s3, x2)
};

void setup() {
  MIDI.begin();
  Serial.begin(115200);
  for (int i = 0; i < N_MUX; i++) {
    mux[i].begin();
  }
  pinMode(x1, INPUT_PULLUP);
  pinMode(x2, INPUT_PULLUP);
  for (int i = 0; i < N_POTS_ARDUINO; i++) {
    pinMode(POT_ARDUINO_PIN[i], INPUT_PULLUP);
  }
  for (int i = 0; i < N_POTS; i++) {
    responsivePot[i] = ResponsiveAnalogRead(0, true, snapMultiplier);
  }
  threadPotentiometers.setInterval(5);
  threadPotentiometers.onRun(ReadPotentiometers);
  cpu.add(&threadPotentiometers);
}

void loop() {
  ReadButtons();
  ReadPotentiometers();

}

void ReadButtons() {
  int nButtonsPerMuxSum = N_BUTTONS_ARDUINO;

  for (int j = 0; j < N_MUX; j++) {
    for (int i = 0; i < N_BUTTONS_PER_MUX[j]; i++) {
      buttonCState[i + nButtonsPerMuxSum] = mux[j].readChannel(BUTTON_MUX_PIN[j][i]);
      // Scale values to 0-1
      if (buttonCState[i + nButtonsPerMuxSum] > buttonMuxThreshold) {
        buttonCState[i + nButtonsPerMuxSum] = HIGH;
      }
      else {
        buttonCState[i + nButtonsPerMuxSum] = LOW;
      }
    }
    nButtonsPerMuxSum += N_BUTTONS_PER_MUX[j];
  }

  for (int i = 0; i < N_BUTTONS; i++) {
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (buttonPState[i] != buttonCState[i]) {
        if (buttonCState[i] == HIGH) {
          MIDI.sendControlChange(i, 127, 1);
        } else {
          MIDI.sendControlChange(i, 0, 1);
        }
        buttonPState[i] = buttonCState[i];
      }
    }
  }
}

int clipValue(int in, int minVal, int maxVal) {
  int out;
  minVal++;
  if (in > maxVal) {
    out = maxVal;
  }
  else if (in < minVal) {
    out = minVal - 1;
  }
  else {
    out = in;
  }
  return out;
}

void ReadPotentiometers() {
  for (int i = 0; i < N_POTS_ARDUINO; i++) {
    reading = analogRead(POT_ARDUINO_PIN[i]);
    responsivePot[i].update(reading);
    potCState[i] = responsivePot[i].getValue();
  }
  int nPotsPerMuxSum = N_POTS_ARDUINO;
  for (int j = 0; j < N_MUX; j++) {
    for (int i = 0; i < N_POTS_PER_MUX[j]; i++) {
      reading = mux[j].readChannel(POT_MUX_PIN[j][i]);
      responsivePot[i + nPotsPerMuxSum].update(reading);
      potCState[i + nPotsPerMuxSum] = responsivePot[i + nPotsPerMuxSum].getValue();
    }
    nPotsPerMuxSum += N_POTS_PER_MUX[j];
  }

  for (int i = 0; i < N_POTS; i++) {
    potCState[i] = clipValue(potCState[i], potMin, potMax);
    potMidiCState[i] = map(potCState[i], potMin, potMax, 0, 127);
    potMidiCState[i] = clipValue(potMidiCState[i], 0, 127);
    potVar = abs(potCState[i] - potPState[i]);
    if (potVar > varThreshold) {
      PTime[i] = millis();
    }
    timer[i] = millis() - PTime[i];
    if (timer[i] < TIMEOUT) {
      potMoving = true;
    }
    else {
      potMoving = false;
    }
    if (potMoving == true) {
      MIDI.sendControlChange(CC_NUMBER + i, potMidiCState[i], 1);
      potPState[i] = potCState[i];
      potMidiPState[i] = potMidiCState[i];

    }
  }
}
