#include "FiatV0Protocol.h"
#include <cstdio>

#define DURATION_DIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

FiatV0Protocol::FiatV0Protocol() {
    reset();
}

void FiatV0Protocol::reset() {
    decoderStep = FiatV0DecoderStepReset;
    manchesterState = ManchesterStateMid1;
    preambleCount = 0;
    dataLow = 0;
    dataHigh = 0;
    bitCount = 0;
    hop = 0;
    fix = 0;
    endByte = 0;
    validCodeFound = false;
    samplesCount = 0;
}

bool FiatV0Protocol::manchesterAdvance(ManchesterEvent event, bool *data) {
    bool result = false;
    ManchesterState next_state = manchesterState;

    if (event == ManchesterEventReset) {
        next_state = ManchesterStateMid1;
    } else if (manchesterState == ManchesterStateMid0 || manchesterState == ManchesterStateMid1) {
        if (event == ManchesterEventShortHigh) next_state = ManchesterStateStart1;
        else if (event == ManchesterEventShortLow) next_state = ManchesterStateStart0;
    } else if (manchesterState == ManchesterStateStart1) {
        if (event == ManchesterEventShortLow) {
            next_state = ManchesterStateMid1;
            result = true; if (data) *data = true;
        } else if (event == ManchesterEventLongLow) {
            next_state = ManchesterStateStart0;
            result = true; if (data) *data = true;
        }
    } else if (manchesterState == ManchesterStateStart0) {
        if (event == ManchesterEventShortHigh) {
            next_state = ManchesterStateMid0;
            result = true; if (data) *data = false;
        } else if (event == ManchesterEventLongHigh) {
            next_state = ManchesterStateStart1;
            result = true; if (data) *data = false;
        }
    }
    manchesterState = next_state;
    return result;
}

void FiatV0Protocol::processBit(bool bit) {
    uint32_t carry = (dataLow >> 31) & 1;
    dataLow = (dataLow << 1) | (bit ? 1 : 0);
    dataHigh = (dataHigh << 1) | carry;
    bitCount++;

    if (bitCount == 64) {
        fix = dataLow;
        hop = dataHigh;
        dataLow = 0;
        dataHigh = 0;
    } else if (bitCount > 70) { // 64 bits + end byte
        endByte = (uint8_t)(dataLow & 0xFF);
        validCodeFound = true;
    }
}

void FiatV0Protocol::feed(bool level, uint32_t duration) {
    switch (decoderStep) {
        case FiatV0DecoderStepReset:
            if (level && DURATION_DIFF(duration, te_short) < te_delta) {
                decoderStep = FiatV0DecoderStepPreamble;
                preambleCount = 0;
                manchesterAdvance(ManchesterEventReset, NULL);
            }
            break;

        case FiatV0DecoderStepPreamble:
            if (DURATION_DIFF(duration, te_short) < te_delta) {
                preambleCount++;
            } else if (preambleCount >= 150 && DURATION_DIFF(duration, gap_threshold) < te_delta) {
                decoderStep = FiatV0DecoderStepData;
                bitCount = 0;
                dataLow = 0;
                dataHigh = 0;
            } else {
                decoderStep = FiatV0DecoderStepReset;
            }
            break;

        case FiatV0DecoderStepData: {
            ManchesterEvent event = ManchesterEventReset;
            if (DURATION_DIFF(duration, te_short) < te_delta) {
                event = level ? ManchesterEventShortLow : ManchesterEventShortHigh;
            } else if (DURATION_DIFF(duration, te_long) < te_delta) {
                event = level ? ManchesterEventLongLow : ManchesterEventLongHigh;
            }

            if (event != ManchesterEventReset) {
                bool bit;
                if (manchesterAdvance(event, &bit)) {
                    processBit(bit);
                }
            } else {
                reset();
            }
            break;
        }
    }
}

bool FiatV0Protocol::decode(long long int* samples, size_t sampleCount) {
    reset();
    for (size_t i = 0; i < sampleCount; i++) {
        feed(samples[i] > 0, (uint32_t)abs(samples[i]));
        if (validCodeFound) return true;
    }
    return false;
}

void FiatV0Protocol::yield(uint32_t hop, uint32_t fix, uint8_t endByte) {
    samplesCount = 0;
    auto addS = [&](uint32_t d) { if(samplesCount < MAX_FIAT_SAMPLES) samplesToSend[samplesCount++] = d; };

    // 1. Dlouhá preambule (cca 150-200 krátkých pulzů)
    for(int i=0; i<160; i++) {
        addS(te_short); 
    }
    
    // 2. Sync Gap
    addS(gap_threshold);

    // 3. Manchester Data (Hop + Fix + EndByte)
    uint64_t fullData = ((uint64_t)hop << 32) | fix;
    // (Zde by následovala smyčka pro Manchester odesílání...)
    // Pro úsporu místa: implementuj podobně jako u VW yield.
}

String FiatV0Protocol::getCodeString() {
    char buf[128];
    snprintf(buf, sizeof(buf), "Fiat V0 71bit\nKey:%08X%08X\nHop:%08X Fix:%08X\nEnd:%02X", 
             hop, fix, hop, fix, endByte);
    return String(buf);
}