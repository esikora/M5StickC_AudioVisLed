#ifndef FFTPROCESSOR_H
#define FFTPROCESSOR_H

#include <Arduino.h>
#include <M5StickCPlus.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>
#include <math.h>

class FFTProcessor
{
private:
public:
    FFTProcessor();

    bool setupI2Smic();
    bool setupSpectrumAnalysis();
    void setup();
    void loop();

    int *getLightness();
    bool getBeatHit();
};

#endif