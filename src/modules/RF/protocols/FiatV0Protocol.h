#ifndef FIAT_V0_PROTOCOL_H
#define FIAT_V0_PROTOCOL_H

#include <Arduino.h>
#include  "help.h"

#define MAX_FIAT_SAMPLES 600 // Fiat má velmi dlouhou preambuli

enum FiatV0DecoderStep {
    FiatV0DecoderStepReset = 0,
    FiatV0DecoderStepPreamble = 1,
    FiatV0DecoderStepData = 2,
};



class FiatV0Protocol {
public:
    FiatV0Protocol();
    void reset();
    void feed(bool level, uint32_t duration);
    bool decode(long long int* samples, size_t sampleCount);
    void yield(uint32_t hop, uint32_t fix, uint8_t endByte);

    String getCodeString();

    // Data pro UI
    uint32_t hop;
    uint32_t fix;
    uint8_t endByte;
    bool validCodeFound;

    uint32_t samplesToSend[MAX_FIAT_SAMPLES];
    uint16_t samplesCount;

private:
    const uint32_t te_short = 200;
    const uint32_t te_long = 400;
    const uint32_t te_delta = 100;
    const uint32_t gap_threshold = 800;

    FiatV0DecoderStep decoderStep;
    ManchesterState manchesterState;
    
    uint16_t preambleCount;
    uint32_t dataLow;
    uint32_t dataHigh;
    uint8_t bitCount;

    bool manchesterAdvance(ManchesterEvent event, bool *data);
    void processBit(bool bit);
};

#endif