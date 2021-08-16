# M5StickC_AudioVisLed
Audio visualization based on an M5StickC (ESP32):
- Sampling of audio data from built-in microphone using i2s (44100 Hz sample rate)
- Transformation of sampled data into the frequency domain using arduinoFFT (2048 samples FFT, 21.5 Hz frequency resolution)
- Visualization of 20 frequency bands and beat detection using an RGB LED strip with 72 LEDs (configurable)

## Getting Started
#### Development environment
- Visual Studio Code (version 1.59.0)
- PlatformIO IDE for VSCode

#### System
- Device: [M5StickC](https://docs.m5stack.com/#/en/core/m5stickc)
- Platform: espressif32
- Board: m5stick-c
- Framework: arduino

#### Peripherals
- [RGB LED strip](https://docs.m5stack.com/en/unit/neopixel)

#### Libraries used
- M5StickC
- arduinoFFT (develop branch)
- FastLED

## Project Description

A comprehensive description of this project is available at [hackster.io](https://www.hackster.io/esikora/audio-visualization-with-esp32-i2s-mic-and-rgb-led-strip-4a251c).

## License

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

See the [LICENSE](LICENSE) file for details.

Copyright 2021 © Ernst Sikora
