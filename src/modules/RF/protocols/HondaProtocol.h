#ifndef HONDA_PROTOCOL_H
#define HONDA_PROTOCOL_H

#include <Arduino.h>

#define MAX_HONDA_SAMPLES 300

enum HondaDecoderStep {
    HondaDecoderStepReset = 0,
    HondaDecoderStepCheckPreamble,
    HondaDecoderStepSaveDuration,
    HondaDecoderStepCheckDuration,
};

class HondaProtocol {
public:
    HondaProtocol();
    void reset();
    bool decode(long long int* samples, size_t sampleCount);
    String getCodeString();

    // Data pro UI
    uint64_t fullKey;
    uint32_t deviceID;
    uint16_t counter;
    uint8_t button;
    bool validCodeFound;

    // Pro odesílání
    uint32_t samplesToSend[MAX_HONDA_SAMPLES];
    uint16_t samplesCount;

private:
    const uint32_t te_short = 432;
    const uint32_t te_long = 864;
    const uint32_t te_delta = 150;

    HondaDecoderStep decoderStep;
    uint16_t headerCount;
    uint32_t te_last;
    uint64_t decodeData;
    uint8_t decodeBitCount;

    void feed(bool level, uint32_t duration);
    void parseHondaStructure();
};

#endif