#ifndef HYUNDAI_PROTOCOL_H
#define HYUNDAI_PROTOCOL_H

#include <Arduino.h>

enum HyundaiDecoderStep {
    HyundaiDecoderStepReset = 0,
    HyundaiDecoderStepCheckPreambula,
    HyundaiDecoderStepSaveDuration,
    HyundaiDecoderStepCheckDuration,
};

class HyundaiProtocol {
public:
    HyundaiProtocol();
    void reset();
    bool decode(long long int* samples, size_t sampleCount);
    String getCodeString();

    // Data pro UI
    uint64_t fullKey;
    uint32_t serial;
    uint16_t counter;
    uint8_t button;
    bool validCodeFound;

private:
    const uint32_t te_short = 250;
    const uint32_t te_long = 500;
    const uint32_t te_delta = 100;
    const uint8_t min_bit_count = 61;

    HyundaiDecoderStep decoderStep;
    uint16_t headerCount;
    uint32_t te_last;
    uint64_t decodeData;
    uint8_t decodeBitCount;

    void feed(bool level, uint32_t duration);
    void parseHyundaiStructure();
};

#endif