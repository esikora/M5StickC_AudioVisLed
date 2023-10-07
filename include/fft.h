#ifndef FFT_H
#define FFT_H

#include <Arduino.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>
#include <math.h>

class FFTProcessor
{
public:
    FFTProcessor();
    ~FFTProcessor();

    bool setupI2Smic();
    void I2SLoop();
    bool setupSpectrumAnalysis();
    bool getBeatHit();
};

#endif