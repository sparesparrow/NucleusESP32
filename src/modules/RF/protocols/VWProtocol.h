#ifndef VW_PROTOCOL_H
#define VW_PROTOCOL_H

#include <Arduino.h>
#include <vector>

/**
 * VW (VAG) Protocol implementation for ESP32.
 * Based on Flipper Zero subghz protocols.
 */

// Manchester stavy pro dekodér
#include "help.h"

enum VwDecoderStep {
    VwDecoderStepReset = 0,
    VwDecoderStepFoundSync,
    VwDecoderStepFoundStart1,
    VwDecoderStepFoundStart2,
    VwDecoderStepFoundStart3,
    VwDecoderStepFoundData,
};

class VWProtocol {
public:
    VWProtocol();

    // Hlavní metody pro zpracování
    void reset();
    void feed(bool level, uint32_t duration);
    bool decode(long long int* samples, size_t sampleCount);
    
    /**
     * Vygeneruje surové pulzy pro odeslání.
     * @param type Typ zprávy (byte 0)
     * @param key Hlavní 64-bitový klíč
     * @param check Kontrolní součet/tlačítko (byte 9)
     */
    void yield(uint8_t type, uint64_t key, uint8_t check);

    // Formátování výstupu
    String getCodeString();

    // Data přístupná pro externí zpracování
    uint64_t data_main;      // 64 bitů (hlavní část)
    uint16_t data_extra;     // 16 bitů (Type + Check)
    uint8_t decodeCountBit;
    bool validCodeFound;

    // Seznam vygenerovaných vzorků (v mikrosekundách)
    std::vector<uint32_t> samplesToSend;

private:
    // Časové konstanty protokolu (v mikrosekundách)
    const uint32_t te_short = 500;
    const uint32_t te_long = 1000;
    const uint32_t te_delta = 120;
    const uint8_t min_count_bit = 80;

    VwDecoderStep decoderStep;
    ManchesterState manchesterState;

    // Pomocné vnitřní metody
    bool manchesterAdvance(ManchesterEvent event, bool *data);
    void addBit(bool level);
    const char* getButtonName(uint8_t btn);
};

#endif // VW_PROTOCOL_H