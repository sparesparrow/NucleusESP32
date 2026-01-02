#include "HondaProtocol.h"

#define DURATION_DIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

HondaProtocol::HondaProtocol() {
    reset();
}

void HondaProtocol::reset() {
    decoderStep = HondaDecoderStepReset;
    headerCount = 0;
    te_last = 0;
    decodeData = 0;
    decodeBitCount = 0;
    validCodeFound = false;
    fullKey = 0; deviceID = 0; counter = 0; button = 0;
    samplesCount = 0;
}

void HondaProtocol::parseHondaStructure() {
    fullKey = decodeData;
    // Honda structure: 8b Sync | 32b ID | 16b Counter | 8b Button
    uint8_t* b = (uint8_t*)&decodeData;
    
    // Extrakce dle Flipper implementace (pozor na endianitu)
    deviceID = ((uint32_t)b[1] << 24) | (b[2] << 16) | (b[3] << 8) | b[4];
    counter = (b[5] << 8) | b[6];
    button = b[7];
}

void HondaProtocol::feed(bool level, uint32_t duration) {
    switch(decoderStep) {
        case HondaDecoderStepReset:
            if(level && (DURATION_DIFF(duration, te_short) < te_delta)) {
                decoderStep = HondaDecoderStepCheckPreamble;
                te_last = duration;
                headerCount = 0;
            }
            break;

        case HondaDecoderStepCheckPreamble:
            if(level) {
                if(DURATION_DIFF(duration, te_short) < te_delta) te_last = duration;
                else decoderStep = HondaDecoderStepReset;
            } else {
                if((DURATION_DIFF(duration, te_short) < te_delta) && 
                   (DURATION_DIFF(te_last, te_short) < te_delta)) {
                    headerCount++;
                } else if((DURATION_DIFF(duration, te_long) < te_delta * 2) && headerCount >= 10) {
                    decoderStep = HondaDecoderStepSaveDuration;
                    decodeData = 0;
                    decodeBitCount = 0;
                } else {
                    decoderStep = HondaDecoderStepReset;
                }
            }
            break;

        case HondaDecoderStepSaveDuration:
            if(level) {
                if(duration >= (te_long * 3)) { // End of Transmission
                    if(decodeBitCount >= 64) {
                        parseHondaStructure();
                        validCodeFound = true;
                    }
                    decoderStep = HondaDecoderStepReset;
                } else {
                    te_last = duration;
                    decoderStep = HondaDecoderStepCheckDuration;
                }
            } else { decoderStep = HondaDecoderStepReset; }
            break;

        case HondaDecoderStepCheckDuration:
            if(!level) {
                if((DURATION_DIFF(te_last, te_short) < te_delta) && (DURATION_DIFF(duration, te_long) < te_delta)) {
                    // Short-Long = 0
                    decodeData = (decodeData << 1);
                    decodeBitCount++;
                    decoderStep = HondaDecoderStepSaveDuration;
                } else if((DURATION_DIFF(te_last, te_long) < te_delta) && (DURATION_DIFF(duration, te_short) < te_delta)) {
                    // Long-Short = 1
                    decodeData = (decodeData << 1) | 1;
                    decodeBitCount++;
                    decoderStep = HondaDecoderStepSaveDuration;
                } else { decoderStep = HondaDecoderStepReset; }
            } else { decoderStep = HondaDecoderStepReset; }
            break;
    }
}

bool HondaProtocol::decode(long long int* samples, size_t sampleCount) {
    reset();
    for (size_t i = 0; i < sampleCount; i++) {
        feed(samples[i] > 0, (uint32_t)abs(samples[i]));
        if (validCodeFound) return true;
    }
    return false;
}

String HondaProtocol::getCodeString() {
    char buf[150];
    snprintf(buf, sizeof(buf), "Honda 64bit\nID:%08X Btn:%02X\nCnt:%04X\nVuln:CVE-2022-27254", 
             deviceID, button, counter);
    return String(buf);
}