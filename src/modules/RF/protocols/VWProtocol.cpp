#include "VWProtocol.h"
#include <cstdio>

#define DURATION_DIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

// Konstruktor
VWProtocol::VWProtocol() {
    reset();
}

// Resetování stavu
void VWProtocol::reset() {
    decoderStep = VwDecoderStepReset;
    manchesterState = ManchesterStateMid1;
    data_main = 0;
    data_extra = 0;
    decodeCountBit = 0;
    validCodeFound = false;
    samplesToSend.clear();
}

// Manchester stavový automat
bool VWProtocol::manchesterAdvance(ManchesterEvent event, bool *data) {
    bool result = false;
    ManchesterState next_state = ManchesterStateMid1;

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

// Přidání bitu do datových struktur
void VWProtocol::addBit(bool level) {
    if (decodeCountBit >= min_count_bit) return;

    uint8_t bit_index_full = min_count_bit - 1 - decodeCountBit;

    if (bit_index_full >= 72) { // Type (8 bit)
        uint8_t idx = bit_index_full - 64; 
        if (level) data_extra |= (1U << idx);
    } else if (bit_index_full >= 8) { // Key (64 bit)
        uint8_t idx = bit_index_full - 8;
        if (level) data_main |= (1ULL << idx);
    } else { // Check (8 bit)
        if (level) data_extra |= (1U << bit_index_full);
    }

    decodeCountBit++;
    if (decodeCountBit >= min_count_bit) {
        validCodeFound = true;
    }
}

// Hlavní dekódovací logika (krmení pulzy)
void VWProtocol::feed(bool level, uint32_t duration) {
    uint32_t te_med = (te_long + te_short) / 2;

    switch (decoderStep) {
        case VwDecoderStepReset:
            if (DURATION_DIFF(duration, te_short) < te_delta) 
                decoderStep = VwDecoderStepFoundSync;
            break;

        case VwDecoderStepFoundSync:
            if (DURATION_DIFF(duration, te_short) < te_delta) break;
            if (level && DURATION_DIFF(duration, te_long) < te_delta) 
                decoderStep = VwDecoderStepFoundStart1;
            else decoderStep = VwDecoderStepReset;
            break;

        case VwDecoderStepFoundStart1:
            if (!level && DURATION_DIFF(duration, te_short) < te_delta) 
                decoderStep = VwDecoderStepFoundStart2;
            else decoderStep = VwDecoderStepReset;
            break;

        case VwDecoderStepFoundStart2:
            if (level && DURATION_DIFF(duration, te_med) < te_delta) 
                decoderStep = VwDecoderStepFoundStart3;
            else decoderStep = VwDecoderStepReset;
            break;

        case VwDecoderStepFoundStart3:
            if (DURATION_DIFF(duration, te_med) < te_delta) break;
            if (level && DURATION_DIFF(duration, te_short) < te_delta) {
                manchesterAdvance(ManchesterEventReset, NULL);
                manchesterAdvance(ManchesterEventShortHigh, NULL);
                decodeCountBit = 0;
                data_main = 0;
                data_extra = 0;
                decoderStep = VwDecoderStepFoundData;
            } else decoderStep = VwDecoderStepReset;
            break;

        case VwDecoderStepFoundData:
            ManchesterEvent event = ManchesterEventReset;
            if (DURATION_DIFF(duration, te_short) < te_delta)
                event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
            else if (DURATION_DIFF(duration, te_long) < te_delta)
                event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;

            if (event == ManchesterEventReset) reset();
            else {
                bool new_bit;
                if (manchesterAdvance(event, &new_bit)) addBit(new_bit);
            }
            break;
    }
}

// Dekódování celého pole vzorků
bool VWProtocol::decode(long long int* samples, size_t sampleCount) {
    reset();
    for(size_t i = 0; i < sampleCount; i++) {
        feed(samples[i] > 0, (uint32_t)abs(samples[i]));
        if(validCodeFound) return true;
    }
    return false;
}

// Generování vzorků pro odeslání
void VWProtocol::yield(uint8_t type, uint64_t key, uint8_t check) {
    samplesToSend.clear();

    // 1. Preamble (40x short high/low)
    for (int i = 0; i < 40; i++) {
        samplesToSend.push_back(te_short); // High
        samplesToSend.push_back(te_short); // Low
    }

    // 2. Start Frame
    samplesToSend.push_back(te_long);
    samplesToSend.push_back(te_short);
    uint32_t te_med = (te_long + te_short) / 2;
    samplesToSend.push_back(te_med);
    samplesToSend.push_back(te_med);
    samplesToSend.push_back(te_short);

    // 3. Payload Manchester
    uint8_t full_payload[10];
    full_payload[0] = type;
    for (int i = 0; i < 8; i++) {
        full_payload[1 + i] = (key >> (56 - i * 8)) & 0xFF;
    }
    full_payload[9] = check;

    for (int byte_idx = 0; byte_idx < 10; byte_idx++) {
        for (int bit_idx = 7; bit_idx >= 0; bit_idx--) {
            bool bit = (full_payload[byte_idx] >> bit_idx) & 1;
            if (bit) {
                samplesToSend.push_back(te_short); // High
                samplesToSend.push_back(te_short); // Low
            } else {
                samplesToSend.push_back(te_short); // Low
                samplesToSend.push_back(te_short); // High
            }
        }
    }
    samplesToSend.push_back(te_long * 10); 
}

// Převod na text pro UI
const char* VWProtocol::getButtonName(uint8_t btn) {
    switch (btn) {
        case 0x1: return "UNLOCK";
        case 0x2: return "LOCK";
        case 0x4: return "TRUNK";
        case 0x8: return "PANIC";
        default:  return "Unknown";
    }
}

String VWProtocol::getCodeString() {
    uint8_t type = (data_extra >> 8) & 0xFF;
    uint8_t check = data_extra & 0xFF;
    uint8_t btn = (check >> 4) & 0xF;
    uint32_t key_h = (uint32_t)(data_main >> 32);
    uint32_t key_l = (uint32_t)(data_main & 0xFFFFFFFF);

    char buf[256];
    snprintf(buf, sizeof(buf),
        "VW/VAG %dbit\n"
        "Key:%02X%08X%08X%02X\n"
        "Type:%02X Btn:%X %s",
        decodeCountBit, type, key_h, key_l, check, 
        type, btn, getButtonName(btn));

    return String(buf);
}