#ifndef BMW_PROTOCOL_H
#define BMW_PROTOCOL_H

#include <Arduino.h>

// Definujeme maximální počet vzorků pro odeslání (preambule + data + tail)
#define MAX_BMW_SAMPLES 200

enum BMWDecoderStep {
    BMWDecoderStepReset = 0,
    BMWDecoderStepCheckPreambula,
    BMWDecoderStepSaveDuration,
    BMWDecoderStepCheckDuration,
};

class BMWProtocol {
public:
    BMWProtocol();
    void reset();
    void feed(bool level, uint32_t duration);
    bool decode(long long int* samples, size_t sampleCount);
    
    void yield(uint64_t data, uint8_t bitCount);

    String getCodeString();

    uint64_t finalData;
    uint8_t finalBitCount;
    uint8_t crcType; 
    bool validCodeFound;

    // Nahrazení vectoru statickým polem
    uint32_t samplesToSend[MAX_BMW_SAMPLES];
    uint16_t samplesCount;

private:
    const uint32_t te_short = 350;
    const uint32_t te_long = 700;
    const uint32_t te_delta = 120;
    const uint8_t min_count_bit = 61;

    BMWDecoderStep decoderStep;
    uint32_t te_last;
    uint16_t header_count;
    uint64_t decodeData;
    uint8_t decodeCountBit;

    uint8_t calculateCRC8(uint8_t* data, size_t len);
    uint16_t calculateCRC16(uint8_t* data, size_t len);
    void addBit(uint8_t bit);
};

#endif