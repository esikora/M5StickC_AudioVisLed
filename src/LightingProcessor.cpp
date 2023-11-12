#include "LightingProcessor.h"

/* ----- From FFTProcessor ----- */
const uint8_t kFreqBandCount = 20;

/* ----- Fastled constants ----- */
const uint8_t kPinLedStrip = 26; // M5StickC grove port, yellow cable
const uint8_t kNumLeds = 139;
const uint8_t kLedStripBrightness = 255;
const uint32_t kMaxMilliamps = 2500;
uint8_t kBassHue = 250;
uint8_t beatVisIntensity_ = 0;

/* ----- Fastled variables ----- */
// LED strip controller
CRGB ledStrip_[kNumLeds];

const uint8_t numFreqLeds = floor(kNumLeds / 2 / (kFreqBandCount + 2));                     // 100 / 2 / 22 = 2
const uint8_t numBassLeds = floor(kNumLeds / 2) - kFreqBandCount * numFreqLeds;             // 50 - 40 = 10
const uint8_t numExtraLeds = kNumLeds - ((kFreqBandCount * numFreqLeds + numBassLeds) * 2); // 100 - 100 but could be an odd number.

uint8_t userTriggerB_ = 0;

LightingProcessor::LightingProcessor()
{
    // Constructor
}

void LightingProcessor::setupLedStrip()
{
    delay(500);
    FastLED.addLeds<NEOPIXEL, kPinLedStrip>(ledStrip_, kNumLeds);
    FastLED.clear();
    FastLED.setBrightness(kLedStripBrightness);
    FastLED.setMaxPowerInVoltsAndMilliamps(12, kMaxMilliamps); // Set maximum power consumption to 5 V and 2.5 A
    ledStrip_[0].setHSV(60, 255, 255);
    FastLED.show();

    Serial.printf("Total leds: %i, %i for each band and %i for bass. There are %i extras to be distributed.",
                  kNumLeds, numFreqLeds, numBassLeds, numExtraLeds);
}

void LightingProcessor::updateLedStrip(int lightness[], bool isBeatHit, String modifier)
{
    if (!modifier.isEmpty())
    {
        if (modifier.indexOf('-') >= 0)
        {
            String key = modifier.substring(0, modifier.indexOf('-'));
            String value = modifier.substring(modifier.indexOf('-') + 1);
            uint8_t val_i = value.toInt();

            if (key == "solid")
            {
                for (int i = 0; i < kNumLeds; i++)
                {
                    ledStrip_[i].setHSV(val_i, 255, 255);
                }
                FastLED.show();
                return;
            }
        }
    }
    uint8_t ledIndex = 0;
    kBassHue++;

    // Detect magnitude peak
    beatVisIntensity_ = (isBeatHit) ? 250 : (beatVisIntensity_ > 0) ? beatVisIntensity_ -= 25
                                                                    : 0;

    // Show beat detection at the beginning of the strip
    for (int i = 0; i < numBassLeds; i++)
    {
        ledStrip_[ledIndex++].setHSV(kBassHue, 255, beatVisIntensity_);
    }

    // Show frequency intensities on the remaining Leds
    const uint8_t colorStart = 30;
    const uint8_t colorEnd = 210;
    const uint8_t colorStep = 3; //(colorEnd - colorStart) / (kNumLeds - numBassLeds * 2) / 2;
    uint8_t color = colorStart;

    // First half
    for (int k = 0; k < kFreqBandCount; k++)
    {
        for (int j = 0; j < numFreqLeds; j++)
        {
            ledStrip_[ledIndex++].setHSV(color, 255, lightness[k]);
            color += colorStep;
        }

        // If extra leds are odd, give extra 1 to the last band aka the center band.
        if (k == kFreqBandCount - 1)
        {
            if (numExtraLeds % 2 == 1)
            {
                ledStrip_[ledIndex++].setHSV(color, 255, lightness[k]);
                color += colorStep;
            }
        }
    }

    // Second half
    for (int k = kFreqBandCount - 1; k >= 0; k--)
    {
        for (int j = 0; j < numFreqLeds; j++)
        {
            ledStrip_[ledIndex++].setHSV(color, 255, lightness[k]);
            color -= colorStep;
        }
    }

    // Show beat detection at the end of the strip
    for (int i = 0; i < numBassLeds; i++)
    {
        ledStrip_[ledIndex++].setHSV(kBassHue, 255, beatVisIntensity_);
    }

    FastLED.show();

    // If user presses ButtonB, print the current lightness array
    M5.BtnB.read();

    if (userTriggerB_ == 0)
    {
        if (M5.BtnB.isPressed())
        {
            userTriggerB_ = 5;
        }
    }
    else
    {
        if (userTriggerB_ == 1)
        {
            for (uint8_t i = 0; i < kFreqBandCount; i++)
            {
                Serial.printf("LED %i = %i\n", i, lightness[i]);
            }
        }
        userTriggerB_ -= 1;
    }
}