#include <Arduino.h>

#include <M5StickC.h>
#include <driver/i2s.h>

#include <math.h>

//#include <FastLED.h>

#include <arduinoFFT.h>

//#include <fix_fft.h>

/* ----- General constants ----- */

const uint16_t kSampleRate = 44100; // Unit: Hz

/* ----- FFT constants ----- */

typedef float fftData_t;

const uint8_t kFFT_SampleCountLog2 = 12;

const uint16_t kFFT_SampleCount = 1 << kFFT_SampleCountLog2;

const fftData_t kFFT_SamplingFreq = (fftData_t) kSampleRate;

const fftData_t kFFT_SampleCountInv = 1.0 / kFFT_SampleCount;

const fftData_t kFFT_SampleTime = (fftData_t) kFFT_SampleCount / kSampleRate;

const uint16_t kFFT_FreqBinCount = kFFT_SampleCount / 2;

const float kFFT_FreqStep = kFFT_SamplingFreq / kFFT_SampleCount;

/* ----- FFT variables ----- */

fftData_t fftDataReal_[kFFT_SampleCount] = {0.0};

fftData_t fftDataImag_[kFFT_SampleCount] = {0.0};

fftData_t magnitudeSpectrumAvg_[kFFT_FreqBinCount] = {0};

ArduinoFFT<fftData_t> fft_ = ArduinoFFT<fftData_t>(fftDataReal_, fftDataImag_, kFFT_SampleCount, kFFT_SamplingFreq); // Create FFT object

/*
int16_t fftDataReal_[kFFT_SampleCount] = {0};

int16_t fftDataImag_[kFFT_SampleCount] = {0};

uint16_t magnitudeSpectrumAvg_[kFFT_FreqBinCount] = {0};
*/

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

    // Configure i2s for sampling 16 bit mono audio data
    //
    // Notes:
    // - dma_buf_len : Number of samples in each DMA buffer, limited to 1024. dma_buf_len * 'bytes_per_sample' is limted to 4092.
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

// -----------------------------------------------
// FactoryTest.ino

/*
void MicRecordfft(void *arg)
{
    int16_t *buffptr;
    size_t bytesread;
    uint16_t count_n = 0;
    float adc_data;
    double data = 0;
    uint16_t ydata;

    while (1)
    {
        xSemaphoreTake(start_fft, portMAX_DELAY);
        xSemaphoreGive(start_fft);

        fft_config_t *real_fft_plan = fft_init(512, FFT_REAL, FFT_FORWARD, NULL, NULL);
        i2s_read(I2S_NUM_0, (char *)i2s_readraw_buff, 1024, &bytesread, (100 / portTICK_RATE_MS));
        buffptr = (int16_t *)i2s_readraw_buff;

        for (count_n = 0; count_n < real_fft_plan->size; count_n++)
        {
            adc_data = (float)map(buffptr[count_n], INT16_MIN, INT16_MAX, -1000, 1000);
            real_fft_plan->input[count_n] = adc_data;
        }
        fft_execute(real_fft_plan);

        xSemaphoreTake(xSemaphore, 100 / portTICK_RATE_MS);
        for (count_n = 1; count_n < real_fft_plan->size / 4; count_n++)
        {
            data = sqrt(real_fft_plan->output[2 * count_n] * real_fft_plan->output[2 * count_n] + real_fft_plan->output[2 * count_n + 1] * real_fft_plan->output[2 * count_n + 1]);
            if ((count_n - 1) < 80)
            {
                ydata = map(data, 0, 2000, 0, 256);
                fft_dis_buff[posData][80 - count_n] = ydata;
            }
        }
        posData++;
        if (posData >= 161)
        {
            posData = 0;
        }
        xSemaphoreGive(xSemaphore);
        fft_destroy(real_fft_plan);
    }
}


void mic_record_task (void* arg)
{   
    size_t bytesread;

    while(1)
    {
        i2s_read(kI2S_Port, (char*) micReadBuffer_, kFF, &bytesread, (100 / portTICK_RATE_MS));
        adcBuffer = (int16_t *)BUFFER;
        showSignal();
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void showSignal(){
    int y;
    for (int n = 0; n < 160; n++){
    y = adcBuffer[n] * GAIN_FACTOR;
    y = map(y, INT16_MIN, INT16_MAX, 10, 70);
    M5.Lcd.drawPixel(n, oldy[n],WHITE);
    M5.Lcd.drawPixel(n,y,BLACK);
    oldy[n] = y;
    }
}
*/

/**
 * Computes the integer square root of n.
 * 
 * Based on: https://en.wikipedia.org/wiki/Integer_square_root
 */
uint16_t intSqrt(uint32_t n)
{
     if (n < 2)
        return n;

    int8_t shift = 2;

    while ((n >> shift) != 0)
        shift += 2;

    uint16_t result = 0;

    while (shift >= 0)
    {
        result = result << 1;

        uint16_t largeCand = result + 1;

        uint32_t largeCandSq = largeCand;
        largeCandSq *= largeCand;

        if ( largeCandSq <= (n >> shift) )
            result = largeCand;

        shift -= 2;
    }

    return result;
}


void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.println("Audio Vis");

    M5.Lcd.setTextSize(1);

    setupI2Smic();

    log_d("Setup successfully completed.");

    log_d("portTICK_PERIOD_MS: %d", portTICK_PERIOD_MS);

    delay(1000);
}


uint16_t slotNr = 0;
const uint16_t kSlotCount = 21;

int16_t accumMin = INT16_MAX;
int16_t accumMax = INT16_MIN;

int32_t accumAvgSum = 0;
int16_t accumAvgCount = 0;

unsigned long timeReadLastMicros = 0;

unsigned long timeCompMaxMicros = 0;

/*
const uint16_t kRecChunkCount = 22;
const uint32_t kRecSampleCount = kFFT_SampleCount * kRecChunkCount;

int16_t recBuf_[kRecSampleCount] = {0};
uint32_t recBufPos_ = 0;

bool recActive_ = false;
*/

uint8_t userTrigger_ = 0;

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
    unsigned long timeBetweenRead = timeAferReadMicros - timeReadLastMicros;

    // Store timestamp for next computation
    timeReadLastMicros = timeAferReadMicros;

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

    log_d("Read duration [µs]: %d. Duration since last read [µs]: %d", timeInRead, timeBetweenRead);

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
    accumAvgSum += blockAvg;
    accumAvgCount += 1;
    accumMin = min(accumMin, blockMin);
    accumMax = max(accumMax, blockMax);

    // Initialize fft input data
    for (uint16_t i = 0; i < kFFT_SampleCount; i++)
    {
        // Subtract the block average from each sample in order remove the DC component
        int16_t v = micReadBuffer_[i] - blockAvg;

        fftDataReal_[i] = (fftData_t) v;
        fftDataImag_[i] = 0.0f;

        /*
        fftDataReal_[i] = v;
        fftDataImag_[i] = 0;
        */
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
    
    /*
    // Compute forward FFT. Return value "scale" is 0 for the forward FFT.
    fix_fft(fftDataReal_, fftDataImag_, kFFT_SampleCountLog2, 0); // @FIXME: Replace "0 = forward" by enum value
    */

    // Compute magnitude value for each frequency bin, i.e. only first half of the FFT results
    for (uint16_t i = 0; i < kFFT_FreqBinCount; i++)
    {
        float magValNew = sqrtf( fftDataReal_[i] * fftDataReal_[i] + fftDataImag_[i] * fftDataImag_[i] );

        const float w1 = 9.0f/128.0f;
        const float w2 = 1 - w1;

        magnitudeSpectrumAvg_[i] = magValNew * w1 + magnitudeSpectrumAvg_[i] * w2;

        /*
        int32_t realSq = fftDataReal_[i];
        realSq *= fftDataReal_[i];

        int32_t imagSq = fftDataImag_[i];
        imagSq *= fftDataImag_[i];

        uint32_t sumSq = realSq + imagSq;

        uint32_t magValNew = intSqrt(sumSq);
        
        uint32_t magValOldAvg = magnitudeSpectrumAvg_[i];
        uint32_t magValNewAvg = (magValNew * 9 + magValOldAvg * 119) / 128;

        if ( magValNewAvg <= UINT16_MAX )
        {
            magnitudeSpectrumAvg_[i] = (uint16_t) magValNewAvg;
        }
        else {
            magnitudeSpectrumAvg_[i] = UINT16_MAX;

            log_d("Magnitude value larger than UINT16_MAX: %d", magValNewAvg);
        }
        */
    }

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
            for (uint16_t i = 0; i < kFFT_FreqBinCount; i++)
            {
                /*
                Serial.printf("%.1f Hz: %d\n", kFFT_FreqStep * i, magnitudeSpectrumAvg_[i]);
                */

               Serial.printf("%.1f Hz: %.2f\n", kFFT_FreqStep * i, magnitudeSpectrumAvg_[i]);
            }
        }
        userTrigger_ -= 1;
    }

    // Compute duration of loop
    unsigned long timeEndMicros = micros();
    unsigned long timeDeltaMicros = timeEndMicros - timeStartMicros;

    if (timeDeltaMicros > timeCompMaxMicros)
    {
        timeCompMaxMicros = timeDeltaMicros;
    }

    slotNr = (slotNr + 1) % kSlotCount;

    if (slotNr == 0)
    {
        //M5.Lcd.setCursor(0, 10, 4);
        //M5.Lcd.printf("%08.1f", 0.0);

        log_d("Min sample value: %d", accumMin);
        log_d("Max sample value: %d", accumMax);
        log_d("Average value: %d", accumAvgSum / accumAvgCount);
        log_d("Maximum duration of processing: %d microseconds.", timeCompMaxMicros);

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

        accumAvgSum = 0;
        accumAvgCount = 0;

        accumMin = INT16_MAX;
        accumMax = INT16_MIN;
        
        timeCompMaxMicros = 0;
    }
}