#include "FordV0Protocol.h"
#include <cstdio>

#define DURATION_DIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

FordV0Protocol::FordV0Protocol() {
    reset();
}

void FordV0Protocol::reset() {
    decoderStep = FordV0DecoderStepReset;
    manchesterState = ManchesterStateMid1;
    dataLow = 0; dataHigh = 0;
    bitCount = 0; headerCount = 0;
    key1 = 0; key2 = 0;
    serial = 0; button = 0; count = 0;
    validCodeFound = false;
    printf("[DEBUG] Reset decoder\n");
}

bool FordV0Protocol::manchesterAdvance(ManchesterEvent event, bool *data) {
    bool result = false;
    ManchesterState next_state = manchesterState;
    if (event == ManchesterEventReset) {
        next_state = ManchesterStateMid1;
        printf("[DEBUG] Manchester: Reset\n");
    }
    else if (manchesterState == ManchesterStateMid0 || manchesterState == ManchesterStateMid1) {
        if (event == ManchesterEventShortHigh) {
            next_state = ManchesterStateStart1;
            printf("[DEBUG] Manchester: Mid -> Start1 (ShortHigh)\n");
        }
        else if (event == ManchesterEventShortLow) {
            next_state = ManchesterStateStart0;
            printf("[DEBUG] Manchester: Mid -> Start0 (ShortLow)\n");
        }
    }
    else if (manchesterState == ManchesterStateStart1) {
        if (event == ManchesterEventShortLow) {
            next_state = ManchesterStateMid1; result = true; if (data) *data = true;
            printf("[DEBUG] Manchester: Start1 -> Mid1 (ShortLow), bit=1\n");
        }
        else if (event == ManchesterEventLongLow) {
            next_state = ManchesterStateStart0; result = true; if (data) *data = true;
            printf("[DEBUG] Manchester: Start1 -> Start0 (LongLow), bit=1\n");
        }
    }
    else if (manchesterState == ManchesterStateStart0) {
        if (event == ManchesterEventShortHigh) {
            next_state = ManchesterStateMid0; result = true; if (data) *data = false;
            printf("[DEBUG] Manchester: Start0 -> Mid0 (ShortHigh), bit=0\n");
        }
        else if (event == ManchesterEventLongHigh) {
            next_state = ManchesterStateStart1; result = true; if (data) *data = false;
            printf("[DEBUG] Manchester: Start0 -> Start1 (LongHigh), bit=0\n");
        }
    }
    manchesterState = next_state;
    return result;
}

void FordV0Protocol::addBit(bool bit) {
    uint32_t low32 = (uint32_t)dataLow;
    dataLow = (dataLow << 1) | (bit ? 1 : 0);
    dataHigh = (dataHigh << 1) | ((low32 >> 31) & 1);
    bitCount++;
    printf("[DEBUG] Add bit: %d, bitCount: %d, dataLow: %08X, dataHigh: %08X\n", bit, bitCount, dataLow, dataHigh);
}

void FordV0Protocol::decodeFordStructure(uint64_t k1, uint16_t k2) {
    printf("[DEBUG] Decoding Ford structure: key1=%016llX, key2=%04X\n", k1, k2);
    uint8_t buf[13] = {0};
    for (int i = 0; i < 8; ++i) buf[i] = (uint8_t)(k1 >> (56 - i * 8));
    buf[8] = (uint8_t)(k2 >> 8);
    buf[9] = (uint8_t)(k2 & 0xFF);

    uint8_t tmp = buf[8];
    uint8_t parity = 0;
    uint8_t parity_any = (tmp != 0);
    while (tmp) { parity ^= (tmp & 1); tmp >>= 1; }
    buf[11] = parity_any ? parity : 0;

    uint8_t xor_byte = buf[11] ? buf[7] : buf[6];
    uint8_t limit = buf[11] ? 7 : 6;

    for (int idx = 1; idx < limit; ++idx) buf[idx] ^= xor_byte;
    if (buf[11] == 0) buf[7] ^= xor_byte;

    uint8_t orig_b7 = buf[7];
    buf[7] = (orig_b7 & 0xAA) | (buf[6] & 0x55);
    uint8_t mixed = (buf[6] & 0xAA) | (orig_b7 & 0x55);
    buf[6] = mixed;

    uint32_t serial_le = ((uint32_t)buf[1]) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
    serial = __builtin_bswap32(serial_le);
    button = (buf[5] >> 4) & 0x0F;
    count = ((buf[5] & 0x0F) << 16) | (buf[6] << 8) | buf[7];

    printf("[DEBUG] Decoded: serial=%08X, button=%02X, count=%06X\n", serial, button, count);
}

bool FordV0Protocol::processData() {
    if (bitCount == 64) {
        key1 = ~((dataHigh << 32) | dataLow);
        printf("[DEBUG] 64 bits received, key1=%016llX\n", key1);
        dataLow = 0; dataHigh = 0;
        return false;
    }
    if (bitCount == 80) {
        key2 = ~((uint16_t)(dataLow & 0xFFFF));
        printf("[DEBUG] 80 bits received, key2=%04X\n", key2);
        decodeFordStructure(key1, key2);
        return true;
    }
    return false;
}

void FordV0Protocol::feed(bool level, uint32_t duration) {
    printf("[DEBUG] Feed: level=%d, duration=%u\n", level, duration);
    switch (decoderStep) {
        case FordV0DecoderStepReset:
            if (level && DURATION_DIFF(duration, te_short) < te_delta) {
                decoderStep = FordV0DecoderStepPreamble;
                headerCount = 0; bitCount = 0;
                manchesterAdvance(ManchesterEventReset, NULL);
                printf("[DEBUG] Step: Reset -> Preamble\n");
            }
            break;
        case FordV0DecoderStepPreamble:
            if (!level && DURATION_DIFF(duration, te_long) < te_delta) {
                decoderStep = FordV0DecoderStepPreambleCheck;
                printf("[DEBUG] Step: Preamble -> PreambleCheck\n");
            } else { decoderStep = FordV0DecoderStepReset; printf("[DEBUG] Step: Preamble -> Reset (invalid)\n"); }
            break;
        case FordV0DecoderStepPreambleCheck:
            if (level) {
                if (DURATION_DIFF(duration, te_long) < te_delta) {
                    headerCount++; decoderStep = FordV0DecoderStepPreamble;
                    printf("[DEBUG] Step: PreambleCheck -> Preamble, headerCount=%d\n", headerCount);
                } else if (DURATION_DIFF(duration, te_short) < te_delta) {
                    decoderStep = FordV0DecoderStepGap;
                    printf("[DEBUG] Step: PreambleCheck -> Gap\n");
                } else { decoderStep = FordV0DecoderStepReset; printf("[DEBUG] Step: PreambleCheck -> Reset (invalid)\n"); }
            }
            break;
        case FordV0DecoderStepGap:
            if (!level && (DURATION_DIFF(duration, gap_threshold) < 300)) {
                dataLow = 1; dataHigh = 0; bitCount = 1;
                decoderStep = FordV0DecoderStepData;
                printf("[DEBUG] Step: Gap -> Data, bitCount=1\n");
            } else if (!level && duration > gap_threshold) { decoderStep = FordV0DecoderStepReset; printf("[DEBUG] Step: Gap -> Reset (invalid)\n"); }
            break;
        case FordV0DecoderStepData: {
            ManchesterEvent event = ManchesterEventReset;
            if (DURATION_DIFF(duration, te_short) < te_delta) event = level ? ManchesterEventShortLow : ManchesterEventShortHigh;
            else if (DURATION_DIFF(duration, te_long) < te_delta) event = level ? ManchesterEventLongLow : ManchesterEventLongHigh;

            bool bit;
            if (event != ManchesterEventReset && manchesterAdvance(event, &bit)) {
                addBit(bit);
                if (processData()) { validCodeFound = true; decoderStep = FordV0DecoderStepReset; printf("[DEBUG] Step: Data -> Reset (valid code)\n"); }
            } else if (event == ManchesterEventReset) { decoderStep = FordV0DecoderStepReset; printf("[DEBUG] Step: Data -> Reset (invalid)\n"); }
            break;
        }
    }
}

bool FordV0Protocol::decode(long long int* samples, size_t sampleCount) {
    reset();
    for (size_t i = 0; i < sampleCount; i++) {
        feed(samples[i] > 0, (uint32_t)abs(samples[i]));
        if (validCodeFound) {
            printf("[DEBUG] Valid code found at sample %zu\n", i);
            return true;
        }
    }
    return false;
}

String FordV0Protocol::getCodeString() {
    char buf[160];
    snprintf(buf, sizeof(buf), "Ford V0 80bit\nKey:%08X%08X\nSn:%08X Btn:%02X\nCnt:%06X BS:%02X", 
             (uint32_t)(key1 >> 32), (uint32_t)(key1 & 0xFFFFFFFF), serial, button, count, (key2 >> 8) & 0xFF);
    printf("[DEBUG] Code string: %s\n", buf);
    return String(buf);
}