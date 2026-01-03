#ifndef CITROEN_PROTOCOL_H
#define CITROEN_PROTOCOL_H

#include <Arduino.h>
#include <vector>

enum CitroenDecoderStep {
    CitroenDecoderStepReset = 0,
    CitroenDecoderStepCheckPreamble,
    CitroenDecoderStepSaveDuration,
    CitroenDecoderStepCheckDuration,
};

class CitroenProtocol {
public:
    CitroenProtocol();
    void reset();
    void feed(bool level, uint32_t duration);
    bool decode(long long int* samples, size_t sampleCount);
    
    // Generování vzorků pro odeslání (Replay/Emulace)
    void yield(uint64_t data, uint8_t bitCount);

    String getCodeString();

    // Veřejná data pro UI/Logování
    uint64_t finalData;
    uint32_t serial;
    uint16_t counter;
    uint8_t button;
    uint8_t finalBitCount;
    bool validCodeFound;

    std::vector<uint32_t> samplesToSend;

private:
    const uint32_t te_short = 370;
    const uint32_t te_long = 772;
    const uint32_t te_delta = 152;
    const uint8_t min_count_bit = 66;

    CitroenDecoderStep decoderStep;
    uint32_t te_last;
    uint16_t header_count;
    uint64_t decodeData;
    uint8_t decodeCountBit;

    // Helpery
    uint8_t reverse8(uint8_t byte);
    void parsePSAStructure();
    void addBit(uint8_t bit);
};

#endif