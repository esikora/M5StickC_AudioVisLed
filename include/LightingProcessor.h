#ifndef LIGHTINGPROCESSOR_H
#define LIGHTINGPROCESSOR_H

#include <Arduino.h>
#include <M5StickCPlus.h>
#include <FastLED.h>

class LightingProcessor
{
private:
public:
    LightingProcessor();

    void setupLedStrip();
    void loop();
    void updateLedStrip(int lightness[], bool isBeatHit, String modifier);
};

#endif