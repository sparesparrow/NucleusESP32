#include "HyundaiProtocol.h"

#define DURATION_DIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

HyundaiProtocol::HyundaiProtocol() {
    reset();
}

void HyundaiProtocol::reset() {
    decoderStep = HyundaiDecoderStepReset;
    headerCount = 0;
    te_last = 0;
    decodeData = 0;
    decodeBitCount = 0;
    validCodeFound = false;
    fullKey = 0; serial = 0; counter = 0; button = 0;
}

void HyundaiProtocol::parseHyundaiStructure() {
    fullKey = decodeData;
    // Hyundai structure:
    // Bits 12-39: Serial (28 bits)
    // Bits 8-11: Button (4 bits)
    // Bits 40-55: Counter (16 bits)
    serial = (uint32_t)((decodeData >> 12) & 0x0FFFFFFF);
    button = (uint32_t)((decodeData >> 8) & 0x0F);
    counter = (uint32_t)((decodeData >> 40) & 0xFFFF);
}

void HyundaiProtocol::feed(bool level, uint32_t duration) {
    switch (decoderStep) {
        case HyundaiDecoderStepReset:
            if (level && DURATION_DIFF(duration, te_short) < te_delta) {
                decoderStep = HyundaiDecoderStepCheckPreambula;
                te_last = duration;
                headerCount = 0;
            }
            break;

        case HyundaiDecoderStepCheckPreambula:
            if (level) {
                if (DURATION_DIFF(duration, te_short) < te_delta || DURATION_DIFF(duration, te_long) < te_delta) {
                    te_last = duration;
                } else {
                    decoderStep = HyundaiDecoderStepReset;
                }
            } else {
                if (DURATION_DIFF(duration, te_short) < te_delta && DURATION_DIFF(te_last, te_short) < te_delta) {
                    headerCount++;
                } else if (DURATION_DIFF(duration, te_long) < te_delta && DURATION_DIFF(te_last, te_long) < te_delta) {
                    if (headerCount > 15) {
                        decoderStep = HyundaiDecoderStepSaveDuration;
                        decodeData = 1; // Start bit
                        decodeBitCount = 1;
                    } else {
                        decoderStep = HyundaiDecoderStepReset;
                    }
                } else {
                    decoderStep = HyundaiDecoderStepReset;
                }
            }
            break;

        case HyundaiDecoderStepSaveDuration:
            if (level) {
                if (duration >= (te_long + te_delta * 2)) {
                    if (decodeBitCount == min_bit_count) {
                        parseHyundaiStructure();
                        validCodeFound = true;
                    }
                    decoderStep = HyundaiDecoderStepReset;
                } else {
                    te_last = duration;
                    decoderStep = HyundaiDecoderStepCheckDuration;
                }
            } else {
                decoderStep = HyundaiDecoderStepReset;
            }
            break;

        case HyundaiDecoderStepCheckDuration:
            if (!level) {
                if (DURATION_DIFF(te_last, te_short) < te_delta && DURATION_DIFF(duration, te_short) < te_delta) {
                    // Short-Short = 0
                    decodeData <<= 1;
                    decodeBitCount++;
                    decoderStep = HyundaiDecoderStepSaveDuration;
                } else if (DURATION_DIFF(te_last, te_long) < te_delta && DURATION_DIFF(duration, te_long) < te_delta) {
                    // Long-Long = 1
                    decodeData = (decodeData << 1) | 1;
                    decodeBitCount++;
                    decoderStep = HyundaiDecoderStepSaveDuration;
                } else {
                    decoderStep = HyundaiDecoderStepReset;
                }
            } else {
                decoderStep = HyundaiDecoderStepReset;
            }
            break;
    }
}

bool HyundaiProtocol::decode(long long int* samples, size_t sampleCount) {
    reset();
    for (size_t i = 0; i < sampleCount; i++) {
        feed(samples[i] > 0, (uint32_t)abs(samples[i]));
        if (validCodeFound) return true;
    }
    return false;
}

String HyundaiProtocol::getCodeString() {
    char buf[128];
    snprintf(buf, sizeof(buf), "Hyundai %dbit\nKey:%08X%08X\nSn:%07X Btn:%X Cnt:%04X", 
             min_bit_count, (uint32_t)(fullKey >> 32), (uint32_t)(fullKey & 0xFFFFFFFF), 
             serial, button, counter);
    return String(buf);
}