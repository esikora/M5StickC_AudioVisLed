#ifndef FIX_FFT_H
#define FIX_FFT_H

#include <stdint.h>

int32_t fix_fft(int16_t[], int16_t[], uint8_t, uint8_t);

int32_t fix_fftr(int16_t[], uint8_t, uint8_t);

#endif