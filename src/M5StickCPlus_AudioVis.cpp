/**
    M5StickC_AudioVisLedApp:
    This application has been developed to use an M5StickC device (ESP32)
    as an audio sampling and visualization device. It samples audio data
    from the built-in microphone using i2s. The sampled data is transformed
    into the frequency domain using the arduinoFFT library.
    Copyright (C) 2021 by Ernst Sikora

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <M5StickCPlus.h>
#include <FastLED.h>
#include <arduinoFFT.h>
#include "fft.h"

/* ----- Fastled constants ----- */
const uint8_t kPinLedStrip = 26; // M5StickC grove port, yellow cable
const uint8_t kNumLeds = 138;    // 150;
const uint8_t kLedStripBrightness = 250;
const uint8_t kBassHue = 250;
const uint32_t kMaxMilliamps = 2500;

/* ----- Fastled variables ----- */
// LED strip controller
CRGB ledStrip_[kNumLeds];

const uint8_t numFreqLeds = floor(kNumLeds / 2 / (kFreqBandCount + 2)) * 2;
const uint8_t numBassLeds = numFreqLeds * 2;
const uint8_t numExtraLeds = kNumLeds - (kFreqBandCount * numFreqLeds + numBassLeds);

typedef float fftData_t;
FFTProcessor fft;

void setupLedStrip()
{
    FastLED.addLeds<NEOPIXEL, kPinLedStrip>(ledStrip_, kNumLeds);
    FastLED.clear();
    FastLED.setBrightness(kLedStripBrightness);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, kMaxMilliamps); // Set maximum power consumption to 5 V and 2.5 A
    ledStrip_[0].setHSV(60, 255, 255);
    FastLED.show();

    log_d("Total leds: %i, %i for each band and %i for bass. There are %i extras to be distributed.",
          kNumLeds, numFreqLeds, numBassLeds, numExtraLeds);
}

void setup()
{
    log_d("M5.begin!");
    M5.begin();

    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(BLACK);

    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextColor(WHITE, BLUE);
    M5.Lcd.println("Audio Vis ");

    fft.setupI2Smic();

    fft.setupSpectrumAnalysis();

    setupLedStrip();

    log_d("Setup successfully completed.");

    log_d("portTICK_PERIOD_MS: %d", portTICK_PERIOD_MS);

    delay(1000);
}

uint8_t userTrigger_ = 0;
uint8_t cycleNr_ = 1;
float maxCurrent_ = 0.0f;
uint8_t displayMode = 1;
uint8_t ledScrollOffset = 0; // Allows dipslay leds to roll right or left
uint8_t beatVisIntensity_ = 0;

// FIXME: These should come from fft.cpp
// Constant for normalizing int16 input values to floating point range -1.0 to 1.0
const fftData_t kInt16MaxInv = 1.0f / __INT16_MAX__;
const uint8_t kFreqBandCount = 20;
const float kFreqBandAmp[kFreqBandCount] = {0.15f, 0.3f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.3f, 0.4f, 0.4f, 0.4f, 0.5f, 0.8f, 1, 1, 1, 1, 1, 0.3f};
float magnitudeBand[kFreqBandCount] = {0.0f};
unsigned long timeStartMicros = micros();
fftData_t sensitivityFactor_ = 1;

/*
uint16_t testSignalFreqFactor_ = 0;
*/

void loop()
{
    fft.I2SLoop();
    switch (displayMode)
    {
    case 3: // Scroll entire strip to the right
        ledScrollOffset = (++ledScrollOffset >= kNumLeds) ? 0 : ledScrollOffset;
        break;
    case 4: // Scroll entire strip to the left
        ledScrollOffset = (--ledScrollOffset < 0) ? kNumLeds - 1 : ledScrollOffset;
        break;
    case 5: // Scroll out from the middle
        ledScrollOffset = (++ledScrollOffset >= kNumLeds / 2) ? 0 : ledScrollOffset;
        break;
    case 6: // Scroll in to the middle
        ledScrollOffset = (--ledScrollOffset < 0) ? kNumLeds / 2 - 1 : ledScrollOffset;
        break;
    default:
        ledScrollOffset = 0;
    }

    if (fft.getBeatHit())
    {
        uint8_t beatVisIntensity_ = 255;
    }
    else
    {
        beatVisIntensity_ -= 50;
    }

    // ----- Update the Led strip -----
    uint8_t i = 0;
    uint8_t ledIndex = 0;

    // Show beat detection at the beginning of the strip
    while (i < numBassLeds / 2)
    {
        ledIndex = (i++) + ledScrollOffset;
        ledIndex = (ledIndex >= kNumLeds) ? ledIndex - kNumLeds : ledIndex;
        ledStrip_[ledIndex].setHSV(kBassHue, 255, beatVisIntensity_);
    }

    // Show frequency intensities on the remaining Leds
    const uint8_t colorStart = 30;
    const uint8_t colorEnd = 210;
    const uint8_t colorStep = 3; //(colorEnd - colorStart) / (kNumLeds - numBassLeds * 2) / 2;
    uint8_t color = colorStart;

    for (int k = 0; k < kFreqBandCount; k++)
    {
        uint8_t lightness = min(int(magnitudeBand[k] * kFreqBandAmp[k] * sensitivityFactor_), 255);

        for (int j = 0; j < numFreqLeds / 2; j++)
        {
            ledIndex = kNumLeds - i - 1 + ledScrollOffset;
            ledIndex = (ledIndex >= kNumLeds) ? ledIndex - kNumLeds : ledIndex;
            ledStrip_[ledIndex].setHSV(color, 255, lightness);

            ledIndex = i++ + ledScrollOffset;
            ledIndex = (ledIndex >= kNumLeds) ? ledIndex - kNumLeds : ledIndex;
            ledStrip_[ledIndex].setHSV(color, 255, lightness);

            color += colorStep;
        }

        // If extra leds are odd, give extra 1 to the last band aka the center band.
        if (k == kFreqBandCount - 1)
        {
            if (numExtraLeds % 2 == 1)
            {
                ledIndex = i + ledScrollOffset;
                ledIndex = (ledIndex >= kNumLeds) ? ledIndex - kNumLeds : ledIndex;
                ledStrip_[ledIndex].setHSV(color, 255, lightness);
                i++;
            }
        }
        else
        {

            // Add one more to distribute extra leds.
            if (floor(numExtraLeds / 2) >= kFreqBandCount - k - 1)
            {
                ledIndex = kNumLeds - i - 1 + ledScrollOffset;
                ledIndex = (ledIndex >= kNumLeds) ? ledIndex - kNumLeds : ledIndex;
                ledStrip_[ledIndex].setHSV(color, 255, lightness);

                ledIndex = i + ledScrollOffset;
                ledIndex = (ledIndex >= kNumLeds) ? ledIndex - kNumLeds : ledIndex;
                ledStrip_[ledIndex].setHSV(color, 255, lightness);
                i++;
            }
        }
    }

    // Show beat detection at the end of the strip
    for (i = kNumLeds - (numBassLeds / 2); i < kNumLeds; i++)
    {
        ledIndex = i + ledScrollOffset;
        ledIndex = (ledIndex >= kNumLeds) ? ledIndex - kNumLeds : ledIndex;
        ledStrip_[ledIndex].setHSV(kBassHue, 255, beatVisIntensity_);
    }

    FastLED.show();

    // Determine current consumption from USB
    float vBusCurrent = M5.Axp.GetVBusCurrent();

    if (vBusCurrent > maxCurrent_)
    {
        maxCurrent_ = vBusCurrent;
    }

    // Determine current consumption from battery
    float batCurrent = 0.5f * M5.Axp.GetIdischargeData();

    if (batCurrent > maxCurrent_)
    {
        maxCurrent_ = batCurrent;
    }

    // Show current consumption on display
    if (cycleNr_ == 1)
    {
        int16_t cursorX = M5.Lcd.getCursorX();
        int16_t cursorY = M5.Lcd.getCursorY();

        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.printf("%03.0f mA\n", maxCurrent_);

        M5.Lcd.setTextSize(4);
        M5.Lcd.setTextColor(GREEN, BLACK);
        M5.Lcd.println("");
        M5.Lcd.printf("     %i", displayMode);

        M5.Lcd.setCursor(cursorX, cursorY);
        maxCurrent_ = 0;
    }

    cycleNr_ = (cycleNr_ + 1) % 20;
}
