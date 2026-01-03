#include "CitroenProtocol.h"
#include <cstdio>

#define DURATION_DIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

CitroenProtocol::CitroenProtocol() {
    reset();
}

void CitroenProtocol::reset() {
    decoderStep = CitroenDecoderStepReset;
    te_last = 0;
    header_count = 0;
    decodeData = 0;
    decodeCountBit = 0;
    finalData = 0;
    serial = 0;
    counter = 0;
    button = 0;
    finalBitCount = 0;
    validCodeFound = false;
    samplesToSend.clear();
}

uint8_t CitroenProtocol::reverse8(uint8_t byte) {
    byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4;
    byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2;
    byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1;
    return byte;
}

void CitroenProtocol::parsePSAStructure() {
    uint8_t* b = (uint8_t*)&finalData;

    // Extrakce šifrované části (32 bitů) s reverzí
    uint32_t encrypted = ((uint32_t)reverse8(b[3]) << 24) | 
                         (reverse8(b[2]) << 16) | 
                         (reverse8(b[1] & 0x0F) << 8) | 
                         reverse8(b[0]);

    // Extrakce sériového čísla (28 bitů) s reverzí
    serial = ((uint32_t)reverse8(b[7] & 0xF0) << 20) | 
             (reverse8(b[6]) << 12) | 
             (reverse8(b[5]) << 4) | 
             (reverse8(b[4]) >> 4);

    button = (encrypted >> 28) & 0x0F;
    counter = (encrypted >> 16) & 0xFFFF;
}

void CitroenProtocol::addBit(uint8_t bit) {
    decodeData = (decodeData << 1) | (bit & 1);
    decodeCountBit++;
}

void CitroenProtocol::feed(bool level, uint32_t duration) {
    switch(decoderStep) {
        case CitroenDecoderStepReset:
            if(level && (DURATION_DIFF(duration, te_short) < te_delta)) {
                decoderStep = CitroenDecoderStepCheckPreamble;
                te_last = duration;
                header_count = 0;
            }
            break;

        case CitroenDecoderStepCheckPreamble:
            if(level) {
                if(DURATION_DIFF(duration, te_short) < te_delta) {
                    te_last = duration;
                } else {
                    decoderStep = CitroenDecoderStepReset;
                }
            } else {
                if((DURATION_DIFF(duration, te_short) < te_delta) && 
                   (DURATION_DIFF(te_last, te_short) < te_delta)) {
                    header_count++;
                } else if((DURATION_DIFF(duration, 4400) < 500) && header_count >= 10) {
                    decoderStep = CitroenDecoderStepSaveDuration;
                    decodeData = 0;
                    decodeCountBit = 0;
                } else {
                    decoderStep = CitroenDecoderStepReset;
                }
            }
            break;

        case CitroenDecoderStepSaveDuration:
            if(level) {
                if(duration >= (te_long * 3)) { // End of packet / Long sync
                    if(decodeCountBit >= min_count_bit) {
                        finalData = decodeData;
                        finalBitCount = decodeCountBit;
                        parsePSAStructure();
                        validCodeFound = true;
                    }
                    decoderStep = CitroenDecoderStepReset;
                } else {
                    te_last = duration;
                    decoderStep = CitroenDecoderStepCheckDuration;
                }
            } else {
                decoderStep = CitroenDecoderStepReset;
            }
            break;

        case CitroenDecoderStepCheckDuration:
            if(!level) {
                // PWM dekódování: Short-Long = 0, Long-Short = 1
                if((DURATION_DIFF(te_last, te_short) < te_delta) && 
                   (DURATION_DIFF(duration, te_long) < te_delta)) {
                    addBit(0);
                    decoderStep = CitroenDecoderStepSaveDuration;
                } else if((DURATION_DIFF(te_last, te_long) < te_delta) && 
                          (DURATION_DIFF(duration, te_short) < te_delta)) {
                    addBit(1);
                    decoderStep = CitroenDecoderStepSaveDuration;
                } else {
                    decoderStep = CitroenDecoderStepReset;
                }
            } else {
                decoderStep = CitroenDecoderStepReset;
            }
            break;
    }
}

bool CitroenProtocol::decode(long long int* samples, size_t sampleCount) {
    reset();
    for(size_t i = 0; i < sampleCount; i++) {
        feed(samples[i] > 0, (uint32_t)abs(samples[i]));
        if(validCodeFound) return true;
    }
    return false;
}

void CitroenProtocol::yield(uint64_t data, uint8_t bitCount) {
    samplesToSend.clear();
    // 1. Preamble (12x short pulses)
    for(int i=0; i<12; i++) {
        samplesToSend.push_back(te_short);
        samplesToSend.push_back(te_short);
    }
    // 2. Sync Gap
    samplesToSend.push_back(te_short);
    samplesToSend.push_back(4400);

    // 3. PWM Data
    for(int i = bitCount - 1; i >= 0; i--) {
        if((data >> i) & 1) { // Bit 1: Long High, Short Low
            samplesToSend.push_back(te_long);
            samplesToSend.push_back(te_short);
        } else { // Bit 0: Short High, Long Low
            samplesToSend.push_back(te_short);
            samplesToSend.push_back(te_long);
        }
    }
    // 4. Final sync / Tail
    samplesToSend.push_back(te_short);
    samplesToSend.push_back(te_long * 10);
}

String CitroenProtocol::getCodeString() {
    char buf[160];
    snprintf(buf, sizeof(buf),
        "PSA Citroen %dbit\n"
        "Key:%08X%08X\n"
        "Sn:%07X Btn:%X Cnt:%04X\n"
        "Type:Keeloq PSA",
        finalBitCount,
        (uint32_t)(finalData >> 32), (uint32_t)(finalData & 0xFFFFFFFF),
        serial, button, counter);
    return String(buf);
}