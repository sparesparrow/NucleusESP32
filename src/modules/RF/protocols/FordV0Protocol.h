#ifndef FORD_V0_PROTOCOL_H
#define FORD_V0_PROTOCOL_H

#include <Arduino.h>
#include "help.h"

#define MAX_FORD_SAMPLES 400

enum FordV0DecoderStep {
    FordV0DecoderStepReset = 0,
    FordV0DecoderStepPreamble,
    FordV0DecoderStepPreambleCheck,
    FordV0DecoderStepGap,
    FordV0DecoderStepData,
};

class FordV0Protocol {
public:
    FordV0Protocol();
    void reset();
    void feed(bool level, uint32_t duration);
    bool decode(long long int* samples, size_t sampleCount);

    String getCodeString();

    // Data pro UI
    uint64_t key1;
    uint16_t key2;
    uint32_t serial;
    uint8_t button;
    uint32_t count;
    bool validCodeFound;

private:
    const uint32_t te_short = 250;
    const uint32_t te_long = 500;
    const uint32_t te_delta = 100;
    const uint32_t gap_threshold = 3500;

    FordV0DecoderStep decoderStep;
    ManchesterState manchesterState;
    
    uint64_t dataLow;
    uint64_t dataHigh;
    uint8_t bitCount;
    uint16_t headerCount;

    bool manchesterAdvance(ManchesterEvent event, bool *data);
    void addBit(bool bit);
    bool processData();
    void decodeFordStructure(uint64_t k1, uint16_t k2);
};

#endif