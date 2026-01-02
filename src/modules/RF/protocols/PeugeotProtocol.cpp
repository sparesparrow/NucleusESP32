#include "PeugeotProtocol.h"

#define DURATION_DIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

PeugeotProtocol::PeugeotProtocol() {
    reset();
}

void PeugeotProtocol::reset() {
    decoderStep = PeugeotDecoderStepReset;
    headerCount = 0;
    te_last = 0;
    decodeData = 0;
    decodeBitCount = 0;
    validCodeFound = false;
}

uint8_t PeugeotProtocol::reverse8(uint8_t byte) {
    byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4;
    byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2;
    byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1;
    return byte;
}

bool PeugeotProtocol::parsePSAStructure() {
    uint8_t* b = (uint8_t*)&decodeData;
    
    // Validace preambule v datech (prvních 12 bitů by mělo být 0xFFF)
    if(b[0] != 0xFF || (b[1] & 0xF0) != 0xF0) return false;

    // Encrypted part (32 bit) - vyžaduje reverse
    uint32_t encrypted = ((uint32_t)reverse8(b[3]) << 24) | 
                         (reverse8(b[2]) << 16) | 
                         (reverse8(b[1] & 0x0F) << 8) | 
                         reverse8(b[0]);

    // Serial number (28 bit)
    serial = ((uint32_t)reverse8(b[7] & 0xF0) << 20) | 
             (reverse8(b[6]) << 12) | 
             (reverse8(b[5]) << 4) | 
             (reverse8(b[4]) >> 4);

    button = (encrypted >> 28) & 0x0F;
    counter = (encrypted >> 16) & 0xFFFF;
    fullKey = decodeData;
    
    return true;
}

void PeugeotProtocol::feed(bool level, uint32_t duration) {
    switch(decoderStep) {
        case PeugeotDecoderStepReset:
            if(level && (DURATION_DIFF(duration, te_short) < te_delta)) {
                decoderStep = PeugeotDecoderStepCheckPreamble;
                te_last = duration;
                headerCount = 0;
            }
            break;

        case PeugeotDecoderStepCheckPreamble:
            if(level) {
                if(DURATION_DIFF(duration, te_short) < te_delta) te_last = duration;
                else decoderStep = PeugeotDecoderStepReset;
            } else {
                if(DURATION_DIFF(duration, te_short) < te_delta && DURATION_DIFF(te_last, te_short) < te_delta) {
                    headerCount++;
                } else if((DURATION_DIFF(duration, 4400) < 600) && headerCount >= 10) {
                    decoderStep = PeugeotDecoderStepSaveDuration;
                    decodeData = 0;
                    decodeBitCount = 0;
                } else { decoderStep = PeugeotDecoderStepReset; }
            }
            break;

        case PeugeotDecoderStepSaveDuration:
            if(level) {
                if(duration >= (te_long * 3)) {
                    if(decodeBitCount >= min_bit_count) {
                        if(parsePSAStructure()) validCodeFound = true;
                    }
                    decoderStep = PeugeotDecoderStepReset;
                } else {
                    te_last = duration;
                    decoderStep = PeugeotDecoderStepCheckDuration;
                }
            } else { decoderStep = PeugeotDecoderStepReset; }
            break;

        case PeugeotDecoderStepCheckDuration:
            if(!level) {
                if(DURATION_DIFF(te_last, te_short) < te_delta && DURATION_DIFF(duration, te_long) < te_delta) {
                    decodeData >>= 1; // PWM 0
                    decodeBitCount++;
                    decoderStep = PeugeotDecoderStepSaveDuration;
                } else if(DURATION_DIFF(te_last, te_long) < te_delta && DURATION_DIFF(duration, te_short) < te_delta) {
                    decodeData = (decodeData >> 1) | (1ULL << 65); // PWM 1 (ukládáme od MSB dolů kvůli LSB-first)
                    decodeBitCount++;
                    decoderStep = PeugeotDecoderStepSaveDuration;
                } else { decoderStep = PeugeotDecoderStepReset; }
            } else { decoderStep = PeugeotDecoderStepReset; }
            break;
    }
}

bool PeugeotProtocol::decode(long long int* samples, size_t sampleCount) {
    reset();
    for (size_t i = 0; i < sampleCount; i++) {
        feed(samples[i] > 0, (uint32_t)abs(samples[i]));
        if (validCodeFound) return true;
    }
    return false;
}

String PeugeotProtocol::getCodeString() {
    char buf[128];
    snprintf(buf, sizeof(buf), "Peugeot PSA\nSn:%07X\nBtn:%X Cnt:%04X\nType:KeeLoq AM", 
             serial, button, counter);
    return String(buf);
}