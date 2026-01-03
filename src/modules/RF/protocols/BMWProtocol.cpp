#include "BMWProtocol.h"
#include <cstdio>

#define DURATION_DIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

BMWProtocol::BMWProtocol() {
    reset();
}

void BMWProtocol::reset() {
    decoderStep = BMWDecoderStepReset;
    te_last = 0;
    header_count = 0;
    decodeData = 0;
    decodeCountBit = 0;
    validCodeFound = false;
    
    // OPRAVA: Místo samplesToSend.clear()
    samplesCount = 0; 
}

uint8_t BMWProtocol::calculateCRC8(uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for(size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
            else crc <<= 1;
        }
    }
    return crc;
}

uint16_t BMWProtocol::calculateCRC16(uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

void BMWProtocol::addBit(uint8_t bit) {
    decodeData = (decodeData << 1) | (bit & 1);
    decodeCountBit++;
}

void BMWProtocol::feed(bool level, uint32_t duration) {
    switch(decoderStep) {
        case BMWDecoderStepReset:
            if(level && (DURATION_DIFF(duration, te_short) < te_delta)) {
                decoderStep = BMWDecoderStepCheckPreambula;
                te_last = duration;
                header_count = 0;
            }
            break;

        case BMWDecoderStepCheckPreambula:
            if(level) {
                if((DURATION_DIFF(duration, te_short) < te_delta) || (DURATION_DIFF(duration, te_long) < te_delta)) {
                    te_last = duration;
                } else {
                    decoderStep = BMWDecoderStepReset;
                }
            } else {
                if((DURATION_DIFF(duration, te_short) < te_delta) && (DURATION_DIFF(te_last, te_short) < te_delta)) {
                    header_count++;
                } else if((DURATION_DIFF(duration, te_long) < te_delta) && (DURATION_DIFF(te_last, te_long) < te_delta)) {
                    if(header_count > 15) {
                        decoderStep = BMWDecoderStepSaveDuration;
                        decodeData = 0;
                        decodeCountBit = 0;
                    } else {
                        decoderStep = BMWDecoderStepReset;
                    }
                } else {
                    decoderStep = BMWDecoderStepReset;
                }
            }
            break;

        case BMWDecoderStepSaveDuration:
            if(level) {
                if(duration >= (te_long + te_delta * 2)) {
                    if(decodeCountBit >= min_count_bit) {
                        // CRC Kontrola
                        uint64_t reversedData = 0; // Pro CRC musíme data občas zarovnat dle bitů
                        uint8_t raw_bytes[8];
                        for(int i=0; i<8; i++) raw_bytes[i] = (decodeData >> (i*8)) & 0xFF;
                        size_t raw_len = (decodeCountBit + 7) / 8;

                        uint8_t c8 = calculateCRC8(raw_bytes, raw_len - 1);
                        if(c8 == raw_bytes[raw_len - 1]) {
                            crcType = 8;
                        } else {
                            uint16_t c16 = calculateCRC16(raw_bytes, raw_len - 2);
                            uint16_t rx_c16 = (raw_bytes[raw_len - 2] << 8) | raw_bytes[raw_len - 1];
                            if(c16 == rx_c16) crcType = 16;
                            else crcType = 0;
                        }

                        if(crcType != 0) {
                            finalData = decodeData;
                            finalBitCount = decodeCountBit;
                            validCodeFound = true;
                        }
                    }
                    decoderStep = BMWDecoderStepReset;
                } else {
                    te_last = duration;
                    decoderStep = BMWDecoderStepCheckDuration;
                }
            } else {
                decoderStep = BMWDecoderStepReset;
            }
            break;

        case BMWDecoderStepCheckDuration:
            if(!level) {
                if((DURATION_DIFF(te_last, te_short) < te_delta) && (DURATION_DIFF(duration, te_short) < te_delta)) {
                    addBit(0);
                    decoderStep = BMWDecoderStepSaveDuration;
                } else if((DURATION_DIFF(te_last, te_long) < te_delta) && (DURATION_DIFF(duration, te_long) < te_delta)) {
                    addBit(1);
                    decoderStep = BMWDecoderStepSaveDuration;
                } else {
                    decoderStep = BMWDecoderStepReset;
                }
            } else {
                decoderStep = BMWDecoderStepReset;
            }
            break;
    }
}

bool BMWProtocol::decode(long long int* samples, size_t sampleCount) {
    reset();
    for(size_t i = 0; i < sampleCount; i++) {
        feed(samples[i] > 0, (uint32_t)abs(samples[i]));
        if(validCodeFound) return true;
    }
    return false;
}

void BMWProtocol::yield(uint64_t data, uint8_t bitCount) {
    // OPRAVA: Místo samplesToSend.clear()
    samplesCount = 0;

    // Pomocná lambda pro bezpečné plnění pole
    auto addSample = [&](uint32_t duration) {
        if (samplesCount < MAX_BMW_SAMPLES) { // MAX_BMW_SAMPLES je 200
            samplesToSend[samplesCount++] = duration;
        }
    };

    // 1. Preamble
    for(int i = 0; i < 20; i++) {
        addSample(te_short); // High
        addSample(te_short); // Low
    }

    // 2. Sync
    addSample(te_long);
    addSample(te_long);

    // 3. Data
    for(int i = bitCount - 1; i >= 0; i--) {
        if((data >> i) & 1) {
            addSample(te_long);
            addSample(te_long);
        } else {
            addSample(te_short);
            addSample(te_short);
        }
    }

    // 4. Tail
    addSample(te_short * 20); 
}

String BMWProtocol::getCodeString() {
    uint32_t serial = (uint32_t)((finalData >> 12) & 0x0FFFFFFF);
    uint8_t btn = (finalData >> 8) & 0x0F;
    uint16_t cnt = (finalData >> 40) & 0xFFFF;

    char buf[128];
    snprintf(buf, sizeof(buf),
        "BMW %dbit (CRC:%d)\nKey:%08X%08X\nSn:%07X Btn:%X Cnt:%04X",
        finalBitCount, crcType, 
        (uint32_t)(finalData >> 32), (uint32_t)(finalData & 0xFFFFFFFF),
        serial, btn, cnt);
    return String(buf);
}