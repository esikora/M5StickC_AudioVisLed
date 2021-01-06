#include <Arduino.h>

#include <M5StickC.h>
#include <driver/i2s.h>

#include <math.h>

//#include <FastLED.h>

#include <arduinoFFT.h>

/* ----- General constants ----- */

const uint16_t kSampleRate = 44100; // Unit: Hz

/* ----- FFT constants ----- */

typedef float fftData_t;

const uint8_t kFFT_SampleCountLog2 = 11;

const uint16_t kFFT_SampleCount = 1 << kFFT_SampleCountLog2;

const fftData_t kFFT_SampleCountInv = 1.0f / kFFT_SampleCount;

const fftData_t kFFT_SamplingFreq = (fftData_t) kSampleRate;

const uint16_t kFFT_FreqBinCount = kFFT_SampleCount / 2;

const float kFFT_FreqStep = kFFT_SamplingFreq / kFFT_SampleCount;

/* ----- FFT variables ----- */

fftData_t fftDataReal_[kFFT_SampleCount] = {0.0};

fftData_t fftDataImag_[kFFT_SampleCount] = {0.0};

fftData_t magnitudeSpectrumAvg_[kFFT_FreqBinCount] = {0};

ArduinoFFT<fftData_t> fft_ = ArduinoFFT<fftData_t>(fftDataReal_, fftDataImag_, kFFT_SampleCount, kFFT_SamplingFreq); // Create FFT object

/* ----- i2s hardware constants ----- */

const i2s_port_t kI2S_Port = I2S_NUM_0;

const int kI2S_PinClk = 0;

const int kI2S_PinData = 34;

/* ----- i2s constants ----- */

const i2s_bits_per_sample_t kI2S_BitsPerSample = I2S_BITS_PER_SAMPLE_16BIT;

const uint8_t kI2S_BytesPerSample = kI2S_BitsPerSample / 8;

const uint16_t kI2S_ReadSizeBytes = kFFT_SampleCount * kI2S_BytesPerSample;

const uint16_t kI2S_BufferSizeSamples = 1024;

const uint16_t kI2S_BufferSizeBytes = kI2S_BufferSizeSamples * kI2S_BytesPerSample;

const uint16_t kI2S_BufferCount = (3 * kFFT_SampleCount) / (2 * kI2S_BufferSizeSamples);

const uint8_t kI2S_BufferCountPerFFT = kFFT_SampleCount / kI2S_BufferSizeSamples;

const int kI2S_QueueLength = 16;

/* ----- i2s variables ----- */

int16_t micReadBuffer_[kFFT_SampleCount] = {0};

QueueHandle_t pI2S_Queue_ = nullptr;


bool setupI2Smic()
{
    esp_err_t i2sErr;

    // i2s configuration for sampling 16 bit mono audio data
    //
    // Notes related to i2s.c:
    // - 'dma_buf_len', i.e. the number of samples in each DMA buffer, is limited to 1024
    // - 'dma_buf_len' * 'bytes_per_sample' is limted to 4092
    // - 'I2S_CHANNEL_FMT_ONLY_RIGHT' means "mono", i.e. only one channel to be received via i2s
    //   In the M5StickC microphone example 'I2S_CHANNEL_FMT_ALL_RIGHT' is used which means two channels.
    //   Afterwards, i2s_set_clk is called to change the DMA configuration to just one channel.
    //
    i2s_config_t i2sConfig = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = kSampleRate,
        .bits_per_sample = kI2S_BitsPerSample, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = kI2S_BufferCount,
        .dma_buf_len = kI2S_BufferSizeSamples
    };

    i2sErr = i2s_driver_install(kI2S_Port, &i2sConfig, kI2S_QueueLength, &pI2S_Queue_);

    if (i2sErr)
    {
        log_e("Failed to start i2s driver. ESP error: %s (%x)", esp_err_to_name(i2sErr), i2sErr);
        return false;
    }

    if (pI2S_Queue_ == nullptr)
    {
        log_e("Failed to setup i2s event queue.");
        return false;
    }

    // Configure i2s pins for sampling audio data from the built-in microphone of the M5StickC
    i2s_pin_config_t i2sPinConfig = {
        .bck_io_num = I2S_PIN_NO_CHANGE,
        .ws_io_num = kI2S_PinClk,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = kI2S_PinData
    };

    i2sErr = i2s_set_pin(kI2S_Port, &i2sPinConfig);

    if (i2sErr)
    {
        log_e("Failed to set i2s pins. ESP error: %s (%x)", esp_err_to_name(i2sErr), i2sErr);
        return false;
    }

    return true;
}

// Frequency bands
// Source: https://www.teachmeaudio.com/mixing/techniques/audio-spectrum
//
// Sub-bass:       20-60 Hz
// Bass:           60-250 Hz
// Low midrange:   250-500 Hz
// Midrange:       500-2000 Hz
// Upper midrange: 2000-4000 Hz
// Presence:       4000-6000 Hz
// Brilliance:     6000-20000 Hz

const uint8_t kFreqBandCount = 7;

const float kFreqBandStartHz = 20;
const float kFreqBandEndHz[kFreqBandCount] = {60, 250, 500, 2000, 4000, 6000, 20000};

/* Start indices, end indices and normalization factors for each frequency band
    The indices relate to the frequency bins resulting from the FFT */
uint16_t freqBandBinIdxStart_[kFreqBandCount] = { 0 };
uint16_t freqBandBinIdxEnd_[kFreqBandCount] = { 0 };
float freqBandNormFactor_[kFreqBandCount] = { 0.0f };

bool setupSpectrumAnalysis()
{
    bool success = true;

    // Variables for bin indices and bin count belonging to the current frequency band
    uint16_t binIdxStart; // Index of the first frequency bin of the current frequency band
    uint16_t binIdxEnd;   // Index of the last frequency bin of the current frequency band
    int16_t binCount;     // Number of frequency bins within the current frequency band
    
    // Set bin index for the start of the first frequency band
    binIdxStart = ceilf( kFreqBandStartHz / kFFT_FreqStep);

    // Compute values for all frequency bands
    for (uint8_t bandIdx = 0; bandIdx < kFreqBandCount; bandIdx++)
    {
        // Store index of first frequency bin of current band
        if ( binIdxStart < kFFT_FreqBinCount )
        {
            freqBandBinIdxStart_[bandIdx] = binIdxStart;
        }
        else
        {
            freqBandBinIdxStart_[bandIdx] = 0;
            
            success = false;

            log_e("Failed to set start bin index for frequency band no. %d", bandIdx);
        }
        
        // Compute index of last frequency bin of current band
        binIdxEnd = ceilf( kFreqBandEndHz[bandIdx] / kFFT_FreqStep ) - 1;
        
        if ( binIdxEnd < kFFT_FreqBinCount)
        {
            freqBandBinIdxEnd_[bandIdx] = binIdxEnd;
        }
        else
        {
            freqBandBinIdxEnd_[bandIdx] = 0;
            binIdxEnd = kFFT_FreqBinCount - 1;

            success = false;

            log_e("Failed to set end bin index for frequency band no. %d", bandIdx);
        }

        // Compute normalization factor for current band
        binCount = binIdxEnd - binIdxStart + 1;

        if (binCount > 0)
        {
            freqBandNormFactor_[bandIdx] = 1.0f / binCount;
        }
        else
        {
            freqBandNormFactor_[bandIdx] = 0.0f;

            success = false;

            log_e("Failed to normalization factor for frequency band no. %d", bandIdx);
        }

        // Set binIdxStart for next band
        binIdxStart = binIdxEnd + 1;

        log_d("Bins in band %d: %d to %d. Number of bins: %d. Normalization factor: %.3f",
            bandIdx,
            freqBandBinIdxStart_[bandIdx], freqBandBinIdxEnd_[bandIdx],
            binCount,
            freqBandNormFactor_[bandIdx]);
    }

    return success;
}

void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.println("Audio Vis");

    M5.Lcd.setTextSize(1);

    setupI2Smic();

    setupSpectrumAnalysis();

    log_d("Setup successfully completed.");

    log_d("portTICK_PERIOD_MS: %d", portTICK_PERIOD_MS);

    delay(1000);
}


uint16_t slotNr_ = 0;
const uint16_t kSlotCount = 21;

int16_t accumMin_ = INT16_MAX;
int16_t accumMax_ = INT16_MIN;

int32_t accumAvgSum_ = 0;
int16_t accumAvgCount_ = 0;

unsigned long timeReadLastMicros_ = 0;

unsigned long timeCompMaxMicros_ = 0;

/*
const uint16_t kRecChunkCount = 22;
const uint32_t kRecSampleCount = kFFT_SampleCount * kRecChunkCount;

int16_t recBuf_[kRecSampleCount] = {0};
uint32_t recBufPos_ = 0;

bool recActive_ = false;
*/

uint8_t userTrigger_ = 0;

/*
uint16_t testSignalFreqFactor_ = 0;
*/

void loop() {

    esp_err_t i2sErr = ESP_OK;

    size_t i2sBytesRead = 0;

    unsigned long timeBeforeReadMicros = micros();

    // Note: If the I2S DMA buffer is empty, 'i2s_read' blocks the current thread until data becomes available
    i2sErr = i2s_read(kI2S_Port, micReadBuffer_, kI2S_ReadSizeBytes, &i2sBytesRead, 100 / portTICK_PERIOD_MS);

    // Get timestamp after reading
    unsigned long timeAferReadMicros = micros();

    unsigned long timeInRead = timeAferReadMicros - timeBeforeReadMicros;

    // Compute duration since last read
    unsigned long timeBetweenRead = timeAferReadMicros - timeReadLastMicros_;

    // Store timestamp for next computation
    timeReadLastMicros_ = timeAferReadMicros;

    if (i2sErr)
    {
        log_e("i2s_read failure. ESP error: %s (%x)", esp_err_to_name(i2sErr), i2sErr);
    }

    if (i2sBytesRead != kI2S_ReadSizeBytes)
    {
        log_w("i2s_read unexpected number of bytes: %d", i2sBytesRead);
    }

    i2s_event_t i2sEvent = {};
    uint8_t i2sEventRxDoneCount = 0;

    uint8_t i2sMsgCount = uxQueueMessagesWaiting(pI2S_Queue_);
    
    log_v("Number of I2S events waiting in queue: %d", i2sMsgCount);

    for (uint8_t i = 0; i < i2sMsgCount; i++)
    {
        if ( xQueueReceive(pI2S_Queue_, (void*) &i2sEvent, 0) == pdTRUE )
        {
            switch (i2sEvent.type)
            {
                case I2S_EVENT_DMA_ERROR:
                    log_e("I2S_EVENT_DMA_ERROR");
                    break;
                
                case I2S_EVENT_TX_DONE:
                    log_v("I2S_EVENT_TX_DONE");
                    break;

                case I2S_EVENT_RX_DONE:
                    log_v("I2S_EVENT_RX_DONE");
                    i2sEventRxDoneCount += 1;
                    break;

                case I2S_EVENT_MAX:
                    log_w("I2S_EVENT_MAX");
                    break;
            }
        }
    }

    if (i2sEventRxDoneCount > kI2S_BufferCountPerFFT)
    {
        log_w("Frame loss. Number of I2S_EVENT_RX_DONE events is: %d", i2sEventRxDoneCount);    
    }
    else
    {
        if (i2sEventRxDoneCount < kI2S_BufferCountPerFFT)
        {
            log_e("Configuration error? Number of I2S_EVENT_RX_DONE events is: %d", i2sEventRxDoneCount);    
        }
    }

    log_v("Read duration [µs]: %d. Duration since last read [µs]: %d", timeInRead, timeBetweenRead);

    // Store start time of loop() to compute duration later on
    unsigned long timeStartMicros = micros();

    // Compute sum, min and max of the current sample block
    int32_t blockSum = micReadBuffer_[0];
    int16_t blockMin = micReadBuffer_[0];
    int16_t blockMax = micReadBuffer_[0];

    for (uint16_t i = 1; i < kFFT_SampleCount; i++)
    {
        blockSum += micReadBuffer_[i];
        blockMin = min(blockMin, micReadBuffer_[i]);
        blockMax = max(blockMax, micReadBuffer_[i]);
    }

    // Compute average value for the current sample block
    int16_t blockAvg = blockSum / kFFT_SampleCount;

    // Compute accumulated values
    accumAvgSum_ += blockAvg;
    accumAvgCount_ += 1;
    accumMin_ = min(accumMin_, blockMin);
    accumMax_ = max(accumMax_, blockMax);

    /*
    // Increment factor for test signal frequency
    if ( slotNr_ == 0 )
    {
        testSignalFreqFactor_ += 1;

        if ( testSignalFreqFactor_ > kFFT_FreqBinCount )
        {
            testSignalFreqFactor_ = 1;
        }
    }
    */

    // Initialize fft input data
    for (uint16_t i = 0; i < kFFT_SampleCount; i++)
    {
        // Corrected input value: Subtract the block average from each sample in order remove the DC component
        int16_t v = micReadBuffer_[i] - blockAvg;

        // Constant for normalizing int16 input values to floating point range -1.0 to 1.0
        const fftData_t kInt16MaxInv = 1.0f / __INT16_MAX__;

        // Input value in floating point representation
        fftData_t r;

        /*
        // Compute input value for FFT
        r = kInt16MaxInv * v;
        */

        /*
        // Generate test signal
        const float k2Pi = 6.2831853f;
        const float k2PiSampleCountInv = k2Pi * kFFT_SampleCountInv;
        
        r = sinf( k2PiSampleCountInv * (testSignalFreqFactor_ * i) );
        */
        
        // Store value in FFT input array
        fftDataReal_[i] = r;
        fftDataImag_[i] = 0.0f;
    }

    /*
    // Copy data into recording buffer
    if (recActive_)
    {
        if ( recBufPos_ < kRecSampleCount )
        {
            for (uint16_t i = 0; i < kFFT_SampleCount; i++)
            {
                recBuf_[recBufPos_] = micReadBuffer_[i];
                recBufPos_ += 1;
            }
        }
        else {
            if (recBufPos_ == kRecSampleCount)
            {
                Serial.write( (uint8_t*) recBuf_, kRecSampleCount * 2 );
                
                recBufPos_ = 0;
                recActive_ = false;
            }
        }
    }
    */

    //fft_.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft_.compute(FFTDirection::Forward);
    
    // Compute magnitude value for each frequency bin, i.e. only first half of the FFT results
    for (uint16_t i = 0; i < kFFT_FreqBinCount; i++)
    {
        float magValNew = sqrtf( fftDataReal_[i] * fftDataReal_[i] + fftDataImag_[i] * fftDataImag_[i] );
        
        // Update the averaged spectrum using the current values
        const float w1 = 9.0f/128.0f;
        const float w2 = 1 - w1;

        magnitudeSpectrumAvg_[i] = magValNew * w1 + magnitudeSpectrumAvg_[i] * w2;
    }

    // Compute average magnitude for each frequency band
    float magnitudeBandAvg[kFreqBandCount] = { 0.0f };

    for (uint8_t bandIdx = 0; bandIdx < kFreqBandCount; bandIdx++)
    {

        for (uint16_t binIdx = freqBandBinIdxStart_[bandIdx]; binIdx <= freqBandBinIdxEnd_[bandIdx]; binIdx++)
        {
            magnitudeBandAvg[bandIdx] += magnitudeSpectrumAvg_[binIdx];
        }

        magnitudeBandAvg[bandIdx] *= freqBandNormFactor_[bandIdx];
    }

    // If user presses ButtonA, print the current frequency spectrum to serial
    M5.BtnA.read();

    if (userTrigger_ == 0)
    {
        if ( M5.BtnA.isPressed() )
        {
            userTrigger_ = 5;

            /*
            if ( !recActive_ )
                recActive_ = true;
            */
        }
    }
    else {
        if (userTrigger_ == 1)
        {
            /*
            for (uint16_t i = 0; i < kFFT_FreqBinCount; i++)
            {
                Serial.printf("%.1f Hz: %.2f\n", kFFT_FreqStep * i, magnitudeSpectrumAvg_[i]);
            }
            */

            for (uint8_t i = 0; i < kFreqBandCount; i++)
            {
                Serial.printf("to %.0f Hz: %.2f\n", kFreqBandEndHz[i], magnitudeBandAvg[i]);
            }
        }
        userTrigger_ -= 1;
    }

    // Compute duration of processing
    unsigned long timeEndMicros = micros();
    unsigned long timeDeltaMicros = timeEndMicros - timeStartMicros;

    log_d("1: %06.2f  2: %06.2f  3: %06.2f  4: %06.2f  5: %06.2f  6: %06.2f  7: %06.2f  t: %d",
        magnitudeBandAvg[0],
        magnitudeBandAvg[1],
        magnitudeBandAvg[2],
        magnitudeBandAvg[3],
        magnitudeBandAvg[4],
        magnitudeBandAvg[5],
        magnitudeBandAvg[6],
        timeDeltaMicros);

    if (timeDeltaMicros > timeCompMaxMicros_)
    {
        timeCompMaxMicros_ = timeDeltaMicros;
    }

    slotNr_ = (slotNr_ + 1) % kSlotCount;

    if (slotNr_ == 0)
    {
        //M5.Lcd.setCursor(0, 10, 4);
        //M5.Lcd.printf("%08.1f", 0.0);

        /*
        log_d("Min sample value: %d", accumMin_);
        log_d("Max sample value: %d", accumMax_);
        log_d("Average value: %d", accumAvgSum_ / accumAvgCount_);
        log_d("Maximum duration of processing: %d microseconds.", timeCompMaxMicros_);
        */

        /*
        for (uint16_t i = 0; i < kFFT_SampleCount; i+=8)
        {
            Serial.printf("%03d: ", i);

            for (uint8_t j = 0; j < 8; j++)
            {
                Serial.printf("%6d, ", micReadBuffer_[i+j]);
            }

            Serial.println();
        }
        */

        accumAvgSum_ = 0;
        accumAvgCount_ = 0;

        accumMin_ = INT16_MAX;
        accumMax_ = INT16_MIN;
        
        timeCompMaxMicros_ = 0;
    }
}