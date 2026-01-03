#ifndef PEUGEOT_PROTOCOL_H
#define PEUGEOT_PROTOCOL_H

#include <Arduino.h>

enum PeugeotDecoderStep {
    PeugeotDecoderStepReset = 0,
    PeugeotDecoderStepCheckPreamble,
    PeugeotDecoderStepSaveDuration,
    PeugeotDecoderStepCheckDuration,
};

class PeugeotProtocol {
public:
    PeugeotProtocol();
    void reset();
    bool decode(long long int* samples, size_t sampleCount);
    String getCodeString();

    // Parsovaná data
    uint64_t fullKey;
    uint32_t serial;
    uint16_t counter;
    uint8_t button;
    bool validCodeFound;

private:
    const uint32_t te_short = 370;
    const uint32_t te_long = 772;
    const uint32_t te_delta = 152;
    const uint8_t min_bit_count = 66;

    PeugeotDecoderStep decoderStep;
    uint16_t headerCount;
    uint32_t te_last;
    uint64_t decodeData;
    uint8_t decodeBitCount;

    void feed(bool level, uint32_t duration);
    uint8_t reverse8(uint8_t byte);
    bool parsePSAStructure();
};

#endif