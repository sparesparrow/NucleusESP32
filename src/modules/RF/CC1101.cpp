#include "CC1101.h"
#include "Arduino.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include "GUI/ScreenManager.h"
#include <sstream>
#include <ctime>
#include <vector>
#include <string> 
#include <iostream>
#include <numeric>
#include <algorithm>
#include "GUI/events.h"
#include "SPI.h"
#include "modules/ETC/SDcard.h"
#include <esp_timer.h>
#include <esp_attr.h>
#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "soc/gpio_struct.h"
#include <driver/timer.h>
#include <typeinfo>



float start_freq = 433;
float stop_freq = 434;
float freq = start_freq;
long compare_freq;
float mark_freq;
int rssi;
int mark_rssi = -100;
uint32_t localSampleCount = 0;
uint16_t shortPulseAvg = 0;
uint16_t longPulseAvg = 0;
uint32_t pauseAVG = 0;


SDcard& SD_RF = SDcard::getInstance();




ScreenManager& screenMgr1 = ScreenManager::getInstance();

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


using namespace std;
int receiverGPIO;

String rawString;

uint16_t  sample[SAMPLE_SIZE];

int error_toleranz = 200;

uint8_t samplecount = 0;

bool CC1101_is_initialized = false;
bool receiverEnabled = false;
bool CC1101_recieve_is_running = false;
bool CC1101_transmit_is_running = false;
bool CC1101_isiddle = true;
bool CC1101_interup_attached = false;
uint32_t actualFreq;

int CC1101_MODULATION = 2;

int smoothcount;
unsigned long samplesmooth[SAMPLE_SIZE];

String fullPath;   

RCSwitch mySwitch;

 float strongestASKFreqs[4] = {0};  // Store the four strongest ASK/OOK frequencies
    int strongestASKRSSI[4] = {-200}; // Initialize with very low RSSI values
    float strongestFSKFreqs[2] = {0}; // Store the two strongest FSK frequencies (F0 and F1)
    int strongestFSKRSSI[2] = {-200}; // Initialize FSK RSSI values

    SignalCollection CC1101_CLASS::allData;


    uint32_t cur_timestamp;
uint8_t  cur_status;
uint32_t last_change_time;
uint32_t pulse_duration;


RCSwitch CC1101_CLASS::getRCSwitch() {
 return mySwitch;
}


 std::vector<int64_t> CC1101_CLASS::samplesToSend;
// Initialize static member
CC1101_CLASS::ReceivedData CC1101_CLASS::receivedData;

void IRAM_ATTR InterruptHandler(void *arg) {
    static volatile uint64_t DRAM_ATTR lastTime = 0;
    const uint64_t time = xthal_get_ccount();
    int64_t  duration = time - lastTime;
    lastTime = time;

    if (gpio_get_level(CC1101_CCGDO2A)) {
        duration = -duration;
    }

    // Simple noise filtering
    if (((duration > 22800) and (duration > 0)) or ((-duration > 22800) and (-duration > 0))) { // 95µs minimum
        noInterrupts();
        if (CC1101_CLASS::receivedData.samples.size() < SAMPLE_SIZE) {
            CC1101_CLASS::receivedData.samples.push_back(duration / 240);
            CC1101_CLASS::receivedData.lastReceiveTime = esp_timer_get_time();
            samplecount++;
        }
        if (duration > 4800000 || duration < -4800000) {
                CC1101_CLASS::receivedData.samples.clear();
        }
        interrupts();
    }
}

bool CC1101_CLASS::decodeNiceFlorSProtocol(const long long int* data, size_t size) {
    if (size == 0) {
        Serial.println("No pulse data provided.");
        return false;
    }

    // Protocol constants for NiceFlorS
    const uint32_t te_short = 500;      // expected short pulse duration (µs)
    const uint32_t te_long  = 1000;     // expected long pulse duration (µs)
    const uint32_t te_delta = 300;      // tolerance (µs)
    const uint32_t min_bits = 52;       // minimum bit count required
    // Header: defined as a LOW pulse with absolute duration >= te_short * 38
    const uint32_t header_threshold = te_short * 38; // ~500*38 = 19000 µs

    // Step 1. Find header: a LOW pulse (data[i] < 0) whose absolute value is at least header_threshold.
    size_t headerIndex = 0;
    bool headerFound = false;
    for (size_t i = 0; i < size; i++) {
        if (data[i] < 0 && (uint32_t)llabs(data[i]) >= header_threshold) {
            headerIndex = i;
            headerFound = true;
            break;
        }
    }
    if (!headerFound) {
        Serial.println("Header not found.");
        return false;
    }

    // Step 2. Verify expected start pattern.
    // Expect two pulses immediately after header: a HIGH pulse then a LOW pulse,
    // each with a duration near 3×te_short (about 1500 µs) within tolerance.
    if (headerIndex + 2 >= size) {
        Serial.println("Not enough pulses after header.");
        return false;
    }
    uint32_t firstHigh = (uint32_t)llabs(data[headerIndex + 1]);
    uint32_t firstLow  = (uint32_t)llabs(data[headerIndex + 2]);
    if (!((firstHigh >= (te_short * 3 - te_delta * 3)) && (firstHigh <= (te_short * 3 + te_delta * 3)) &&
          (firstLow  >= (te_short * 3 - te_delta * 3)) && (firstLow  <= (te_short * 3 + te_delta * 3)))) {
        Serial.println("Start bit pattern not found.");
        return false;
    }

    // Step 3. Decode key bits from pulse pairs following the header pattern.
    // Each bit is encoded as two pulses:
    // • For bit 1: a HIGH pulse near te_long (1000 µs ± te_delta) followed by a LOW pulse near te_short (500 µs ± te_delta).
    // • For bit 0: a HIGH pulse near te_short (500 µs ± te_delta) followed by a LOW pulse near te_long (1000 µs ± te_delta).
    size_t i = headerIndex + 3; // start after header + start pulses
    uint64_t decodedValue = 0;
    int bitCount = 0;

    while (i + 1 < size) {
        long long int firstPulse = data[i];    // expected HIGH pulse
        long long int secondPulse = data[i + 1]; // expected LOW pulse

        // Verify polarity: first pulse should be positive, second negative.
        if (firstPulse <= 0 || secondPulse >= 0) {
            i++; // skip this pulse and try next
            continue;
        }
        uint32_t absFirst = (uint32_t)llabs(firstPulse);
        uint32_t absSecond = (uint32_t)llabs(secondPulse);
        bool bit;

        // If HIGH pulse is near te_long and LOW pulse near te_short → bit 1.
        if ((absFirst >= te_long - te_delta && absFirst <= te_long + te_delta) &&
            (absSecond >= te_short - te_delta && absSecond <= te_short + te_delta)) {
            bit = true;
        }
        // If HIGH pulse is near te_short and LOW pulse near te_long → bit 0.
        else if ((absFirst >= te_short - te_delta && absFirst <= te_short + te_delta) &&
                 (absSecond >= te_long - te_delta && absSecond <= te_long + te_delta)) {
            bit = false;
        }
        else {
            // Unrecognized pulse pair; skip it.
            i += 2;
            continue;
        }

        decodedValue = (decodedValue << 1) | (bit ? 1ULL : 0ULL);
        bitCount++;
        i += 2;
    }

    if (bitCount >= (int)min_bits) {
            lv_obj_t * textareaRC;
    lv_obj_t * container = screenMgr1.getSquareLineContainer();
    if(C1101preset == CUSTOM){
        textareaRC = screenMgr1.text_area_SubGHzCustom;        
    } else {
       // Serial.println("Signal preset");
        textareaRC = screenMgr1.getTextArea();
    }


    
    lv_textarea_add_text(textareaRC, "\n NiceFLorS decoded (" );
    lv_textarea_add_text(textareaRC, String(bitCount).c_str());
    lv_textarea_add_text(textareaRC, "\n bits): 0x");
    lv_textarea_add_text(textareaRC, String(decodedValue).c_str());
        Serial.print("NiceFlorS decoded (");
        Serial.print(bitCount);
        Serial.print(" bits): 0x");
        Serial.println(decodedValue, HEX);
        return true;
    } else {
        Serial.print("Not enough bits decoded: ");
        Serial.println(bitCount);
        return false;
    }
}

bool CC1101_CLASS::decodeNiceFloProtocol(const long long int* data, size_t size) {
    if(size == 0) {
        Serial.println("No pulse data provided.");
        return false;
    }
    
    // Protocol constants for NiceFlo
    const uint32_t te_short = 700;      // expected short pulse duration (µs)
    const uint32_t te_long  = 1400;     // expected long pulse duration (µs)
    const uint32_t te_delta = 200;      // tolerance (µs)
    const uint32_t min_bits = 12;       // minimum bits required
    const uint32_t header_threshold = te_short * 36;  // ~700*36 = 25200 µs
    
    // Step 1: Find header: a LOW pulse (negative) with abs(duration) >= header_threshold.
    size_t headerIndex = 0;
    bool headerFound = false;
    for(size_t i = 0; i < size; i++) {
        if(data[i] < 0 && (uint32_t)llabs(data[i]) >= header_threshold) {
            headerIndex = i;
            headerFound = true;
            break;
        }
    }
    if(!headerFound) {
        Serial.println("Header not found.");
        return false;
    }
    
    // Step 2: Decode pulses after the header.
    size_t i = headerIndex + 1;
    uint64_t decodedValue = 0;
    int bitCount = 0;
    
    // Process pulses in pairs (LOW then HIGH).
    // Also, if a LOW pulse is very long (e.g. >= te_short*4) it is considered a boundary.
    while(i + 1 < size) {
        long long int lowPulse = data[i];      // expected LOW pulse
        long long int highPulse = data[i + 1];   // expected HIGH pulse
        
        // Check polarity: lowPulse should be negative, highPulse positive.
        if(lowPulse >= 0 || highPulse <= 0) {
            i++; // skip unexpected pulse
            continue;
        }
        
        uint32_t absLow = (uint32_t)llabs(lowPulse);
        uint32_t absHigh = (uint32_t)llabs(highPulse);
        
        // If the LOW pulse is very long (e.g. boundary), exit decoding.
        if(absLow >= te_short * 4) {
            break;
        }
        
        bool bit;
        // If LOW is near te_short and HIGH near te_long → bit 0.
        if((absLow >= te_short - te_delta && absLow <= te_short + te_delta) &&
           (absHigh >= te_long - te_delta && absHigh <= te_long + te_delta))
        {
            bit = false;
        }
        // If LOW is near te_long and HIGH near te_short → bit 1.
        else if((absLow >= te_long - te_delta && absLow <= te_long + te_delta) &&
                (absHigh >= te_short - te_delta && absHigh <= te_short + te_delta))
        {
            bit = true;
        }
        else {
            // If the pair does not match, skip it.
            i += 2;
            continue;
        }
        
        decodedValue = (decodedValue << 1) | (bit ? 1ULL : 0ULL);
        bitCount++;
        i += 2;
    }
    
    if(bitCount >= (int)min_bits) {
            lv_obj_t * textareaRC;
    lv_obj_t * container = screenMgr1.getSquareLineContainer();
    if(C1101preset == CUSTOM){
        textareaRC = screenMgr1.text_area_SubGHzCustom;        
    } else {
       // Serial.println("Signal preset");
        textareaRC = screenMgr1.getTextArea();
    }


    
    lv_textarea_add_text(textareaRC, "\n NiceFlo decoded (" );
    lv_textarea_add_text(textareaRC, String(bitCount).c_str());
    lv_textarea_add_text(textareaRC, "\n bits): 0x");
    lv_textarea_add_text(textareaRC, String(decodedValue).c_str());
        Serial.print("NiceFlo decoded (");
        Serial.print(bitCount);
        Serial.print(" bits): 0x");
        Serial.println(decodedValue, HEX);
        return true;
    } else {
        Serial.print("Not enough bits decoded: ");
        Serial.println(bitCount);
        return false;
    }
}

// --- Decoding Function for Came Protocol ---
// This function uses the measured averages to decode the raw pulse durations
// into a bit string. It uses pauseAVG (or a fraction thereof) to determine a frame break.
bool CC1101_CLASS::decodeCameProtocol(const long long int* data, size_t size) {
  String decoded = "";
  // Loop through each sample.
  int count;
  bool trueFlag = false;
  for (size_t i = 0; i < size; i++) {
    long long int d = data[i];
    if(count > 11) {
      trueFlag = true;
    }
    // If a low pulse is long enough (e.g. >80% of pauseAVG), mark it as a frame boundary.
    if (d < 0 && -d > (pauseAVG * 0.8)) {
      decoded += " | ";  // Use a delimiter to indicate a gap between frames.
      count = 0;
      continue;
    }
    
    // For HIGH pulses, decide whether this pulse is "short" or "long"
    if (d > 0) {
      // Compare the absolute difference from shortPulseAvg and longPulseAvg.
      float diffShort = abs(d - shortPulseAvg);
      float diffLong  = abs(d - longPulseAvg);
      if (diffShort < diffLong) {
        decoded += "0"; // Consider a short pulse as logic 0.
        count++;
      } else {
        decoded += "1"; // A long pulse as logic 1.
        count++;
      }
    }

    // Optionally, you can ignore low pulses that are not gaps.
  }
  if(trueFlag){
        lv_obj_t * textareaRC;
    lv_obj_t * container = screenMgr1.getSquareLineContainer();
    if(C1101preset == CUSTOM){
        textareaRC = screenMgr1.text_area_SubGHzCustom;        
    } else {
       // Serial.println("Signal preset");
        textareaRC = screenMgr1.getTextArea();
    }


    
    lv_textarea_add_text(textareaRC, "\n came decoded (" );
    lv_textarea_add_text(textareaRC, String(count).c_str());
    lv_textarea_add_text(textareaRC, "\n bits): 0x");
    lv_textarea_add_text(textareaRC, String(decoded).c_str());
    Serial.println(decoded);
    return true;
  } else {
    return false;
  }
  
}


bool CC1101_CLASS::decodeCameAtomoProtocol(const long long int* data, size_t size) {
    if(size == 0) {
        Serial.println("No pulse data provided.");
        return false;
    }
    
    // Atomo protocol constants
    const uint32_t te_short = 600;             // expected short pulse (µs)
    const uint32_t te_long  = 1200;            // expected long pulse (µs)
    const uint32_t te_delta = 250;             // tolerance (µs)
    const uint32_t min_bits = 62;              // minimum bits to be considered valid
    const uint32_t header_threshold = te_long * 60;  // ~72000 µs header threshold

    // Find header: look for a LOW pulse (negative) whose absolute value is at least header_threshold.
    size_t headerIndex = 0;
    bool headerFound = false;
    for(size_t i = 0; i < size; i++) {
        if(data[i] < 0 && (uint32_t)llabs(data[i]) >= header_threshold) {
            headerIndex = i;
            headerFound = true;
            break;
        }
    }
    if(!headerFound) {
        Serial.println("Header not found.");
        return false;
    }
    
    // Start decoding after the header.
    size_t i = headerIndex + 1;
    uint64_t decodedValue = 0;
    int bitCount = 0;
    
    // Process pulses in pairs: expect a LOW pulse (negative) followed by a HIGH pulse (positive)
    while(i + 1 < size) {
        long long int lowPulse = data[i];
        long long int highPulse = data[i + 1];
        
        // Check polarity: low pulse should be negative, high pulse positive.
        if(lowPulse >= 0 || highPulse <= 0) {
            i++;
            continue;
        }
        
        uint32_t absLow = (uint32_t)llabs(lowPulse);
        uint32_t absHigh = (uint32_t)llabs(highPulse);
        bool bit;
        
        // If LOW is near te_long and HIGH is near te_short → interpret as bit 1.
        if((absLow >= te_long - te_delta && absLow <= te_long + te_delta) &&
           (absHigh >= te_short - te_delta && absHigh <= te_short + te_delta))
        {
            bit = true;
        }
        // If LOW is near te_short and HIGH is near te_long → interpret as bit 0.
        else if((absLow >= te_short - te_delta && absLow <= te_short + te_delta) &&
                (absHigh >= te_long - te_delta && absHigh <= te_long + te_delta))
        {
            bit = false;
        }
        else {
            // Pair does not match expected patterns; skip these two pulses.
            i += 2;
            continue;
        }
        
        decodedValue = (decodedValue << 1) | (bit ? 1ULL : 0ULL);
        bitCount++;
        i += 2;
    }
    
    if(bitCount >= (int)min_bits) {
    lv_obj_t * textareaRC;
    lv_obj_t * container = screenMgr1.getSquareLineContainer();
    if(C1101preset == CUSTOM){
        textareaRC = screenMgr1.text_area_SubGHzCustom;        
    } else {
       // Serial.println("Signal preset");
        textareaRC = screenMgr1.getTextArea();
    }


    
    lv_textarea_add_text(textareaRC, "\n Atomo decoded (" );
    lv_textarea_add_text(textareaRC, String(bitCount).c_str());
    lv_textarea_add_text(textareaRC, "\n bits): 0x");
    lv_textarea_add_text(textareaRC, String(decodedValue).c_str());

        Serial.print("Atomo decoded (");
        Serial.print(bitCount);
        Serial.print(" bits): 0x");
        Serial.println(decodedValue, HEX);
        return true;
    } else {
        Serial.print("Not enough bits decoded: ");
        Serial.println(bitCount);
        return false;
    }
}

bool CC1101_CLASS::decodeCameTweeProtocol(const long long int* data, size_t size) {
    if (size == 0) {
        Serial.println("No pulse data provided.");
        return false;
    }
    
    // Came Twee protocol constants
    const uint32_t te_short = 500;             // expected short pulse duration (µs)
    const uint32_t te_long  = 1000;            // expected long pulse duration (µs)
    const uint32_t te_delta = 250;             // tolerance (µs)
    const uint32_t min_bits = 54;              // minimum number of bits required
    const uint32_t header_threshold = te_long * 51;  // e.g. 1000*51 = 51000 µs

    // Step 1: Search for header (a LOW pulse whose abs(duration) is >= header_threshold)
    size_t headerIndex = 0;
    bool headerFound = false;
    for (size_t i = 0; i < size; i++) {
        if (data[i] < 0 && (uint32_t)llabs(data[i]) >= header_threshold) {
            headerIndex = i;
            headerFound = true;
            break;
        }
    }
    if (!headerFound) {
        Serial.println("Header not found.");
        return false;
    }
    
    // Step 2: Decode pulses after the header.
    size_t i = headerIndex + 1;
    uint64_t decodedValue = 0;
    int bitCount = 0;
    
    // Process pulses in pairs: expect a LOW pulse (negative) followed by a HIGH pulse (positive)
    while (i + 1 < size) {
        long long int lowPulse = data[i];
        long long int highPulse = data[i + 1];
        
        // Ensure correct polarity: lowPulse should be negative and highPulse positive.
        if (lowPulse >= 0 || highPulse <= 0) {
            i++; // skip nonconforming pulse and try next
            continue;
        }
        
        uint32_t absLow = (uint32_t)llabs(lowPulse);
        uint32_t absHigh = (uint32_t)llabs(highPulse);
        bool bit;
        
        // If LOW is near te_long and HIGH near te_short → interpret as bit 1.
        if ((absLow >= te_long - te_delta && absLow <= te_long + te_delta) &&
            (absHigh >= te_short - te_delta && absHigh <= te_short + te_delta))
        {
            bit = true;
        }
        // If LOW is near te_short and HIGH near te_long → interpret as bit 0.
        else if ((absLow >= te_short - te_delta && absLow <= te_short + te_delta) &&
                 (absHigh >= te_long - te_delta && absHigh <= te_long + te_delta))
        {
            bit = false;
        }
        else {
            // This pulse pair does not match; skip the pair.
            i += 2;
            continue;
        }
        
        decodedValue = (decodedValue << 1) | (bit ? 1ULL : 0ULL);
        bitCount++;
        i += 2;
    }
    
    if (bitCount >= (int)min_bits) {
            lv_obj_t * textareaRC;
    lv_obj_t * container = screenMgr1.getSquareLineContainer();
    if(C1101preset == CUSTOM){
        textareaRC = screenMgr1.text_area_SubGHzCustom;        
    } else {
       // Serial.println("Signal preset");
        textareaRC = screenMgr1.getTextArea();
    }


    
    lv_textarea_add_text(textareaRC, "\n CAME TWEE decoded (" );
    lv_textarea_add_text(textareaRC, String(bitCount).c_str());
    lv_textarea_add_text(textareaRC, "\n bits): 0x");
    lv_textarea_add_text(textareaRC, String(decodedValue).c_str());
        Serial.print("Came Twee decoded (");
        Serial.print(bitCount);
        Serial.print(" bits): 0x");
        Serial.println(decodedValue, HEX);
        return true;
    } else {
        Serial.print("Not enough bits decoded: ");
        Serial.println(bitCount);
        return false;
    }
}

bool CC1101_CLASS::init() {
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCLK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
    ELECHOUSE_cc1101.Init();

    if (ELECHOUSE_cc1101.getCC1101()) {
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x07);  // Maximum LNA gain
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, 0x00);  // Fastest AGC
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0, 0x91);  // Normal AGC, 16 samples
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND1, 0xB6);    // RX frontend configuration
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND0, 0x10);    // TX frontend configuration
        
        ELECHOUSE_cc1101.setGDO(CCGDO0A, CCGDO2A);
        ELECHOUSE_cc1101.setSidle();
        CC1101_is_initialized = true;
        return true;
    }
    return false;
}

void CC1101_CLASS::setFrequency(float freq)
{
    CC1101_MHZ = freq;
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);
}

void CC1101_CLASS::setSync(int sync)
{
    CC1101_SYNC = sync;
}

void CC1101_CLASS::setPTK(int ptk)
{
    CC1101_PKT_FORMAT = ptk;
}
/*
+------------------+----------------+
| Threshold (dBm)  | AGCCTRL1 Value |
+------------------+----------------+
| -80              | 0x80           |
| -70              | 0x88           |
| -60              | 0x90           |
| -50              | 0x98           |
| -40              | 0xA0           |
| -30              | 0xA8           |
| -20              | 0xB0           |
| -10              | 0xB8           |
|   0              | 0xC0           |
| +10              | 0xC8           |
| +20              | 0xD0           |
| +30              | 0xD8           |
| +40              | 0xE0           |
| +50              | 0xE8           |
| +60              | 0xF0           |
| +70              | 0xF8           |
| +80              | 0xFF           |
+------------------+----------------+
+------------+--------+-------------------------+
| GDO Pin    | Reg    | Inversion Bit (INV)    |
+------------+--------+-------------------------+
| GDO2       | 0x00   | Bit 6 (1 = Inverted)   |
| GDO1       | 0x01   | Bit 6 (1 = Inverted)   |
| GDO0       | 0x02   | Bit 6 (1 = Inverted)   |
+------------+--------+-------------------------+
| Note: Inversion is controlled by setting Bit 6 
|       of the respective IOCFGx register.
+------------------------------------------------+
*/

void CC1101_CLASS::enableReceiver() {
        Serial.println("CC1101: enableReceiver");
        CC1101_CLASS::allData.empty();
      

     ELECHOUSE_cc1101.SpiStrobe(0x30); // Reset CC1101
     localSampleCount = 0;
     delay(50);


        CC1101_CLASS::init();

    CC1101_CLASS::loadPreset();



    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG2, 0x0D); // Set GP2
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x32); // Async mode
    delay(10);

    ELECHOUSE_cc1101.setRxBW(CC1101_RX_BW);
    ELECHOUSE_cc1101.setDcFilterOff(1);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(115.051);

    delay(10);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_4),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
   // gpio_pulldown_en(GPIO_NUM_4);
        ELECHOUSE_cc1101.SetRx();
        delay(20);
    if(!gpio_get_level(CC1101_CCGDO2A)) {
        delay(10);
        uint8_t iocfg0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_IOCFG2);
        iocfg0 |= (1 << 6); 
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG2, iocfg0);
        delay(20);
    }
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(GPIO_NUM_4, InterruptHandler, NULL);

  //  ELECHOUSE_cc1101.SetRx();
    receiverEnabled = true;

        CC1101_CLASS::receivedData.samples.clear();
        CC1101_CLASS::receivedData.lastReceiveTime = 0;
        CC1101_CLASS::receivedData.sampleCount = 0;
        CC1101_CLASS::receivedData.signals.clear();
        CC1101_CLASS::receivedData.normalizedCount = 0;

        delay(500);

    interrupts();
}


void CC1101_CLASS::emptyReceive() {
       ELECHOUSE_cc1101.SpiStrobe(0x30); // Reset CC1101
     localSampleCount = 0;
     delay(50);


        CC1101_CLASS::init();

    CC1101_CLASS::loadPreset();



    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG2, 0x0D); // Set GP2
        delay(10);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x32); // Async mode
    delay(10);

    ELECHOUSE_cc1101.setRxBW(CC1101_RX_BW);
        delay(10);
    ELECHOUSE_cc1101.setDcFilterOff(1);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(115.051);

    delay(10);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_4),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
    gpio_pulldown_en(GPIO_NUM_4);
        ELECHOUSE_cc1101.SetRx();
        delay(20);
    if(!gpio_get_level(CC1101_CCGDO2A)) {
        delay(10);
        uint8_t iocfg0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_IOCFG2);
        iocfg0 |= (1 << 6); 
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG2, iocfg0);
        delay(20);
    }
    // gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    // gpio_isr_handler_add(GPIO_NUM_4, InterruptHandler, NULL);

    ELECHOUSE_cc1101.SetRx();
}


void CC1101_CLASS::enableReceiverCustom() {
    CC1101TH TH;
    Serial.println("Custom Receiver enabled");
     int BW =  lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox);
     int TRS = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox1);
     int SYNC = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox2);
     int PTK = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox3);
     int MOD = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox4);       
     int DEV = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox5);
     int DRATE = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox6);
      Serial.println(BW);
      Serial.println(TRS);
      Serial.println(SYNC);
      Serial.println(PTK);
      Serial.println(MOD);
      Serial.println(DEV);
      Serial.println(DRATE);

       ELECHOUSE_cc1101.SpiStrobe(0x30); // Reset CC1101
     localSampleCount = 0;
     delay(50);


        CC1101_CLASS::init();

    CC1101_CLASS::loadPreset();

 ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG2, 0x0D); // Set GP2
        delay(10);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x32); // Async mode
    delay(10);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1 ,TH.getRegValue(TRS));
   delay(10);

    delay(10);
    ELECHOUSE_cc1101.setRxBW(BW);
        delay(10);
    ELECHOUSE_cc1101.setDcFilterOff(1);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setModulation(MOD);
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);
    ELECHOUSE_cc1101.setDeviation(DEV);
    ELECHOUSE_cc1101.setDRate(DRATE);

 delay(10);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_4),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
   // gpio_pulldown_en(GPIO_NUM_4);
        ELECHOUSE_cc1101.SetRx();
        delay(20);
    if(!gpio_get_level(CC1101_CCGDO2A)) {
        delay(10);
        uint8_t iocfg0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_IOCFG2);
        iocfg0 |= (1 << 6); 
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG2, iocfg0);
        delay(20);
    }
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(GPIO_NUM_4, InterruptHandler, NULL);

  //  ELECHOUSE_cc1101.SetRx();
    receiverEnabled = true;

        CC1101_CLASS::receivedData.samples.clear();
        CC1101_CLASS::receivedData.lastReceiveTime = 0;
        CC1101_CLASS::receivedData.sampleCount = 0;
        CC1101_CLASS::receivedData.signals.clear();
        CC1101_CLASS::receivedData.normalizedCount = 0;

        delay(500);

    interrupts();

}

void CC1101_CLASS::setCC1101Preset(CC1101_PRESET preset) {
    C1101preset = preset;
}



void CC1101_CLASS::disableReceiver()
{
    detachInterrupt((uint8_t)receiverGPIO);
    ELECHOUSE_cc1101.setSidle();
    CC1101.emptyReceive();
 
}

void CC1101_CLASS::loadPreset() {
    switch (C1101preset) {
    case AM650:
        CC1101_MODULATION = 2;
        CC1101_DRATE = 3.79372;
        CC1101_RX_BW = 650.00;
        CC1101_DEVIATION = 1.58;
        break;
    case AM270:
        CC1101_MODULATION = 2;
        CC1101_DRATE = 3.79372;
        CC1101_RX_BW = 270.833333;
        CC1101_DEVIATION = 1.58;
        break;
    case FM238:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 4.79794;
        CC1101_RX_BW = 270.833333;
        CC1101_DEVIATION = 2.380371;
        break;
    case FM476:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 4.79794;
        CC1101_RX_BW = 270.833333;
        CC1101_DEVIATION = 47.60742;
        break;
            break;
    case FM95:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 4.798;
        CC1101_RX_BW = 270;
        CC1101_DEVIATION = 9.521;
        CC1101_SYNC = 6;
        break;
    case FM15k:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 3.794;
        CC1101_RX_BW = 135;
        CC1101_DEVIATION = 15.869;
        CC1101_SYNC = 7;
        break;
        case FSK12k:
        CC1101_MODULATION = 0;       // Modulation derived from preset (2-FSK)
        CC1101_DRATE = 12.69;        // Derived from preset name (12.69kHz deviation)
        CC1101_RX_BW = 200;          // Receiver bandwidth (200kHz as per preset name)
        CC1101_DEVIATION = 12.69;    // Derived from deviation value in preset name
        // CC1101_SYNC = 0x47;          // Sync word from `Custom_preset_data` (byte 3)
        // CC1101_CONFIG = {            // Full configuration extracted from `Custom_preset_data`
        //     0x02, 0x0D, 0x03, 0x47, 0x08, 0x32, 0x0B, 0x06, 0x15, 0x30,
        //     0x14, 0x00, 0x13, 0x00, 0x12, 0x00, 0x11, 0x32, 0x10, 0xA7,
        //     0x18, 0x18, 0x19, 0x1D, 0x1D, 0x92, 0x1C, 0x00, 0x1B, 0x04,
        //     0x20, 0xFB, 0x22, 0x17, 0x21, 0xB6, 0x00, 0x00, 0x00, 0x12,
        //     0x0E, 0x34, 0x60, 0xC5, 0xC1, 0xC0
        // };
        break;
    
    case FSK25k:
        CC1101_MODULATION = 0;       // Modulation derived from preset (2-FSK)
        CC1101_DRATE = 25.39;        // Derived from preset name (25.39kHz deviation)
        CC1101_RX_BW = 200;          // Receiver bandwidth (200kHz as per preset name)
        CC1101_DEVIATION = 25.39;    // Derived from deviation value in preset name
        CC1101_SYNC = 0x47;          // Sync word from `Custom_preset_data` (byte 3)
        // CC1101_CONFIG = {            // Full configuration extracted from `Custom_preset_data`
        //     0x02, 0x0D, 0x03, 0x47, 0x08, 0x32, 0x0B, 0x06, 0x15, 0x40,
        //     0x14, 0x00, 0x13, 0x00, 0x12, 0x00, 0x11, 0x32, 0x10, 0xA7,
        //     0x18, 0x18, 0x19, 0x1D, 0x1D, 0x92, 0x1C, 0x00, 0x1B, 0x04,
        //     0x20, 0xFB, 0x22, 0x17, 0x21, 0xB6, 0x00, 0x00, 0x00, 0x12,
        //     0x0E, 0x34, 0x60, 0xC5, 0xC1, 0xC0
        // };
        break;
    
    case FSK31k:
        CC1101_MODULATION = 0;       // Modulation derived from preset (2-FSK)
        CC1101_DRATE = 31.73;        // Derived from preset name (31.73kHz deviation)
        CC1101_RX_BW = 200;          // Receiver bandwidth (200kHz as per preset name)
        CC1101_DEVIATION = 31.73;    // Derived from deviation value in preset name
        // CC1101_SYNC = 0x47;          // Sync word from `Custom_preset_data` (byte 3)
        // CC1101_CONFIG = {            // Full configuration extracted from `Custom_preset_data`
        //     0x02, 0x0D, 0x03, 0x47, 0x08, 0x32, 0x0B, 0x06, 0x15, 0x42,
        //     0x14, 0x00, 0x13, 0x00, 0x12, 0x00, 0x11, 0x32, 0x10, 0xA7,
        //     0x18, 0x18, 0x19, 0x1D, 0x1D, 0x92, 0x1C, 0x00, 0x1B, 0x04,
        //     0x20, 0xFB, 0x22, 0x17, 0x21, 0xB6, 0x00, 0x00, 0x00, 0x12,
        //     0x0E, 0x34, 0x60, 0xC5, 0xC1, 0xC0
        // };
        break;
    case PAGER:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 0.625;
        CC1101_RX_BW = 270;
        CC1101_DEVIATION = 5.157;
        CC1101_SYNC = 6;
        break;
    case HND1:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 37.04;
        CC1101_RX_BW = 250;
        CC1101_DEVIATION = 30;
        CC1101_SYNC = 6;
        break;
    case HND2:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 15.357;
        CC1101_RX_BW = 67;
        CC1101_DEVIATION = 15.869;
        CC1101_SYNC = 7;
        break;
    default:
        Serial.println(CC1101_MODULATION);
        break;
    }
    Serial.print("preset loaded");
}

bool CC1101_CLASS::CheckReceived() {
    constexpr unsigned long SIGNAL_TIMEOUT = 25000;

    // Timing windows for protocol detection
    constexpr uint16_t SHORT_PULSE_MIN = 100;
    constexpr uint16_t LONG_PULSE_MAX = 1500;
    constexpr uint16_t TOLERANCE_PERCENT = 20; //TODO User set tolerance

    // Ensure atomic read of samplecount
    localSampleCount = 0;

    noInterrupts();
    localSampleCount = samplecount;
    interrupts();

    if (localSampleCount < 10) return false;

    uint64_t currentTime = esp_timer_get_time();
    uint64_t signalDuration = currentTime - CC1101_CLASS::receivedData.lastReceiveTime;

    // Check pointer validity before accessing receivedData.samples
    if (!CC1101_CLASS::receivedData.samples.empty()) {
        if (CC1101_CLASS::receivedData.samples[0] > 2500 || CC1101_CLASS::receivedData.samples[0] < -2500) {
            CC1101_CLASS::receivedData.samples.erase(CC1101_CLASS::receivedData.samples.begin());
          //  Serial.println(F("First pulse removed"));
            CC1101_CLASS::receivedData.startstate = !CC1101_CLASS::receivedData.startstate;
        }
    }

    if (signalDuration > SIGNAL_TIMEOUT || localSampleCount >= SAMPLE_SIZE) {
        if (!CC1101_CLASS::receivedData.samples.empty() && 
            (CC1101_CLASS::receivedData.samples[0] > 2500 || CC1101_CLASS::receivedData.samples[0] < -2500))
        {
            CC1101_CLASS::receivedData.samples.erase(CC1101_CLASS::receivedData.samples.begin());
        //    Serial.println(F("First pulse removed"));
            CC1101_CLASS::receivedData.startstate = !CC1101_CLASS::receivedData.startstate;
        }

        CC1101_CLASS::receivedData.sampleCount = 0;

        std::vector<uint16_t> shortPulses;
        std::vector<uint16_t> longPulses;
        shortPulseAvg = 0;
        longPulseAvg = 0;
        uint16_t samplesCount = CC1101_CLASS::receivedData.samples.size();
        uint16_t pauseTolerance;

        // - fallowed by- or + by + makes no sense, assume its one pulse/space

        auto &samples = CC1101_CLASS::receivedData.samples;
        for (size_t i = 1; i < samples.size(); ++i) {
            if ((samples[i - 1] < 0) && (samples[i] < 0)) {
                samples[i - 1] += samples[i];
                samples.erase(samples.begin() + i);
                --i;
            }
            if ((samples[i - 1] > 0) && (samples[i] > 0)) {
                samples[i - 1] += samples[i];
                samples.erase(samples.begin() + i);
                --i;
            }

            if ((samples[i - 1] < 0) && (samples[i] > 10000)) {
                samples[i - 1] += -samples[i];
                samples.erase(samples.begin() + i);
                --i;
            }
        }

        for (uint32_t i = 0; i < samplesCount; i++) {
            uint16_t sample = CC1101_CLASS::receivedData.samples[i];
            if ((sample < SHORT_PULSE_MIN && sample > 0) || (sample > -SHORT_PULSE_MIN && sample < 0)) {
        //        Serial.println(F("Short pulse skipped"));
           //     Serial.println(sample);
                continue;
            }
            if ((sample > 2000 && sample > 0) || (sample < -2000 && sample < 0)) {
                // Serial.println(F("Long pulse skipped"));
                // Serial.println(sample);
                continue;

            }

            // First valid pulse becomes reference for short pulse
            if (shortPulses.empty() && longPulses.empty() && sample > 0) {
                shortPulses.push_back(sample);
                 Serial.println("short");
                 Serial.println(sample);
                longPulses.push_back(-CC1101_CLASS::receivedData.samples[i+1]);
                 Serial.println("long");
                 Serial.println(-CC1101_CLASS::receivedData.samples[i+1]);
                continue;
            }

            if (shortPulses.empty() && longPulses.empty() && sample < 0) {
                shortPulses.push_back(-sample);
                 Serial.println("short");
                 Serial.println(-sample);
                longPulses.push_back(CC1101_CLASS::receivedData.samples[i+1]);
                 Serial.println("long");
                 Serial.println(CC1101_CLASS::receivedData.samples[i+1]);
                continue;
            }



            // Calculate averages
            shortPulseAvg = std::accumulate(shortPulses.begin(), shortPulses.end(), 0) / 
                           (shortPulses.size() > 0 ? shortPulses.size() : 1);
        //    Serial.println("shortPulseAvg");
       //     Serial.println(shortPulseAvg);

            longPulseAvg = std::accumulate(longPulses.begin(), longPulses.end(), 0) / 
                          (longPulses.size() > 0 ? longPulses.size() : 1);
     //       Serial.println("longPulseAvg");
//Serial.println(longPulseAvg);


            // Determine tolerance levels
            uint16_t shortTolerance = (shortPulseAvg * TOLERANCE_PERCENT) / 100;
            uint16_t longTolerance = (longPulseAvg * TOLERANCE_PERCENT) / 100;



            if(sample > 0) {
                if (static_cast<int>(sample - shortPulseAvg) <= shortTolerance) {
                        shortPulses.push_back(sample);
                } else if (!longPulses.empty() || 
                        static_cast<int>(sample - longPulseAvg) <= longTolerance) {
                        longPulses.push_back(sample);
                 }
            } else {
                if (static_cast<int>(-sample + shortPulseAvg) <= -shortTolerance) {
                        shortPulses.push_back(-sample);
                } else if (!longPulses.empty() || 
                        static_cast<int>(-sample + longPulseAvg) <= -longTolerance) {
                        longPulses.push_back(-sample);
                 }
            }

        }

        shortPulseAvg = std::accumulate(shortPulses.begin(), shortPulses.end(), 0) / 
                       (shortPulses.size() > 0 ? shortPulses.size() : 1);
        longPulseAvg = std::accumulate(longPulses.begin(), longPulses.end(), 0) / 
                       (longPulses.size() > 0 ? longPulses.size() : 1);

        if (longPulseAvg < shortPulseAvg) {
            std::swap(longPulseAvg, shortPulseAvg);
        }

        if(CC1101_CLASS::receivedData.samples.size() < 1){
        return false;
        }

        std::vector<uint16_t> pauses;

        for (const auto& sample : CC1101_CLASS::receivedData.samples) {
            if(-sample > (4 * longPulseAvg)){
            pauses.push_back(-sample);
            }
        }


        pauseAVG = std::accumulate(pauses.begin(), pauses.end(), 0) / 
                    (pauses.size() > 0 ? pauses.size() : 1);      

        Signal data;
        
        Serial.println(F("Pulse Analysis:"));
        Serial.print(F("Short pulses avg: ")); Serial.println(shortPulseAvg);
        Serial.print(F("Long pulses avg: ")); Serial.println(longPulseAvg);
        Serial.print(F("Pause avg: ")); Serial.println(pauseAVG);

        Serial.println(F("Raw samples: "));
        CC1101_CLASS::receivedData.sampleCount = 0;
        for (const auto& sample : CC1101_CLASS::receivedData.samples) {
            Serial.print(sample);
            data.addSample(sample);
            Serial.print(" ");
            CC1101_CLASS::receivedData.sampleCount++;
        }
        CC1101_CLASS::allData.addSignal(data);

        return true;
    }
    
    return false;
}



void CC1101_CLASS::fskAnalyze() {
    Serial.println(F("ana run"));         

    while (true) { 

        if (CC1101_MODULATION == 2) {
            freq = start_freq;

            while (freq <= stop_freq) {
                ELECHOUSE_cc1101.setMHZ(freq);
                int rssi = ELECHOUSE_cc1101.getRssi();

                if (rssi > -80) { 
 
                    for (int i = 0; i < 4; i++) {
                        if (rssi > strongestASKRSSI[i]) {
      
                            for (int j = 3; j > i; j--) {
                                strongestASKRSSI[j] = strongestASKRSSI[j - 1];
                                strongestASKFreqs[j] = strongestASKFreqs[j - 1];
                            }

                            strongestASKRSSI[i] = rssi;
                            strongestASKFreqs[i] = freq;


                            Serial.println(String("New ASK Frequency: ") + strongestASKFreqs[i] +
                                           " MHz | RSSI: " + strongestASKRSSI[i]);
         
                            break;
                        }
                    }
                }

                freq += 0.10; 
            }
        } else if (CC1101_MODULATION == 0) {
            freq = start_freq;

            while (freq <= stop_freq) {
                ELECHOUSE_cc1101.setMHZ(freq);
                int rssi = ELECHOUSE_cc1101.getRssi();

                if (rssi > -80) { 
                    if (rssi > strongestFSKRSSI[0]) {

                        strongestFSKRSSI[1] = strongestFSKRSSI[0];
                        strongestFSKFreqs[1] = strongestFSKFreqs[0];

                        strongestFSKRSSI[0] = rssi;
                        strongestFSKFreqs[0] = freq;


                        Serial.println(String("New FSK Frequencies: F0 = ") + strongestFSKFreqs[0] +
                                       " MHz | RSSI: " + strongestFSKRSSI[0] +
                                       ", F1 = " + strongestFSKFreqs[1] +
                                       " MHz | RSSI: " + strongestFSKRSSI[1]);
                    } else if (rssi > strongestFSKRSSI[1]) {

                        strongestFSKRSSI[1] = rssi;
                        strongestFSKFreqs[1] = freq;


                        Serial.println(String("Updated FSK F1: ") + strongestFSKFreqs[1] +
                                       " MHz | RSSI: " + strongestFSKRSSI[1]);
                    }
                }

                freq += 0.10; 
            }
        } else {
            Serial.println("Unsupported modulation type");
        }
    }
}

void CC1101_CLASS::enableScanner(float start, float stop) {
    start_freq = start;
    stop_freq = stop;

    if (!CC1101_is_initialized) {
        CC1101_CLASS::init();
    }
    CC1101_CLASS::loadPreset();


        ELECHOUSE_cc1101.setDcFilterOff(1);


    ELECHOUSE_cc1101.setSyncMode(CC1101_SYNC);
    ELECHOUSE_cc1101.setPktFormat(CC1101_PKT_FORMAT);
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(CC1101_DRATE);
    ELECHOUSE_cc1101.setRxBW(CC1101_RX_BW);
    pinMode(CCGDO2A, INPUT);

    // Start scanning on second core
    CC1101_CLASS::startSignalAnalyseTask();
}
void CC1101_CLASS::sendByteSequence(const uint8_t sequence[], const uint16_t pulseWidth, const uint8_t messageLength) {
    uint8_t dataByte;
    uint8_t i; 
    for (i = 0; i <= messageLength; i++) 
    {
        dataByte = sequence[i];
        for (int8_t bit = 7; bit >= 0; bit--)
        { 
            digitalWrite(CC1101_CCGDO0A, (dataByte & (1 << bit)) != 0 ? HIGH : LOW);
            delayMicroseconds(pulseWidth);
        }
    }    
}
 void CC1101_CLASS::signalAnalyseTask(void* pvParameters) {
    //CC1101_CLASS *cc1101 = static_cast<CC1101_CLASS *>(pvParameters);

    // Initialize scanning parameters
    const uint32_t subghz_frequency_list[] = {
        300000000, 303875000, 304250000, 310000000, 315000000, 318000000,  //  300-348 MHz
        390000000, 418000000, 433075000, 433420000, 433920000, 434420000, 434775000, 438900000,  //  387-464 MHz
        868350000, 868000000, 915000000, 925000000  //  779-928 MHz
    };
    int num_frequencies = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
    int rssi = 0;
    int mark_rssi = -100;
    float mark_freq = 0;
 //   long compare_freq = 0;

    Serial.println(F("\r\nScanning predefined frequency list, press any key to stop..."));


    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setRxBW(58);
    ELECHOUSE_cc1101.SetRx();


    for (int i = 0; i < num_frequencies; ++i) {
        float freq = subghz_frequency_list[i] / 1000000.0;  // Convert to MHz
        ELECHOUSE_cc1101.setMHZ(freq);
        rssi = ELECHOUSE_cc1101.getRssi();

        if (rssi > -75) {  
            if (rssi > mark_rssi) {
                mark_rssi = rssi;
                mark_freq = freq;
            }
        }

        // Print progress
        Serial.print(F("Scanning at "));
        Serial.print(freq, 6);
        Serial.print(F(" MHz, RSSI: "));
        Serial.println(rssi);

        if (Serial.available()) { 
            break;
        }
    }

    if (mark_rssi > -75) {  
        Serial.print(F("\r\nSignal found at "));
        Serial.print(F("Freq: "));
        Serial.print(mark_freq);
  
  
      
  Serial.print(F(" MHz, RSSI: "));
  Serial.println(mark_rssi);
} else {
  Serial.println(F("\r\nNo strong signal found."));
}

Serial.println(F("\r\nScanning stopped."));

ELECHOUSE_cc1101.SetRx();
}


void CC1101_CLASS::startSignalAnalyseTask() {
    xTaskCreatePinnedToCore(
        CC1101_CLASS::signalAnalyseTask,  // Function to run
        "SignalAnalyseTask",              // Task name
        8192,                             // Stack size
        this,                             // Task parameter (pass instance)
        1,                                // Priority
        NULL,                             // Task handle
        1                                 // Core 1
    );
}


void CC1101_CLASS::signalAnalyse(){

//Serial.println("Signal AnalyseIN");
     lv_obj_t * textareaRC;
    lv_obj_t * container = screenMgr1.getSquareLineContainer();
    if(C1101preset == CUSTOM){
        textareaRC = screenMgr1.text_area_SubGHzCustom;        
    } else {
       // Serial.println("Signal preset");
        textareaRC = screenMgr1.getTextArea();
    }

    lv_textarea_set_text(textareaRC, "\nNew RAW signal");//, \nCount: ");
  //  lv_textarea_add_text(textareaRC, String(CC1101_CLASS::receivedData.sampleCount).c_str());
    
    lv_textarea_add_text(textareaRC, "\nShort pulses avg: ");
    lv_textarea_add_text(textareaRC, String(shortPulseAvg).c_str());
    lv_textarea_add_text(textareaRC, "\nLong pulses avg: ");
    lv_textarea_add_text(textareaRC, String(longPulseAvg).c_str());

    std::ostringstream  rawString;

Serial.println("");
        // for (int i = 0; i < CC1101_CLASS::receivedData.samples.size(); i++) {
        //     if(CC1101_CLASS::receivedData.samples[i] > 25000){
        //         CC1101_CLASS::receivedData.samples[i] = 25000;
        //     }
        // }
Signal samplesData = CC1101_CLASS::allData.getSignal(CC1101_CLASS::allData.signals.size() - 1);
uint16_t pausetreshold = pauseAVG - (pauseAVG / 4);



pulseTrain pulseTrain;
pulseTrains pulseTrains;

  for (int i = 0; i < samplesData.size(); i++) {
            if(-samplesData.getSample(i) > pausetreshold){
                pulseTrains.addPulseTrain(pulseTrain);
                pulseTrain.clear();
                continue;
            }
            samplesData.getSample(i) > 0 ? pulseTrain.addPulse(samplesData.getSample(i)) : pulseTrain.addPulse(-samplesData.getSample(i));
    }

        
    rawString << "RAW_Data " <<  ": ";

    for (int i = 0; i < samplesData.size(); i++) {
            rawString << samplesData.getSample(i);
            Serial.print(CC1101_CLASS::receivedData.samples[i]);
            Serial.print(", ");
        }


Serial.println("");

lv_textarea_add_text(textareaRC, "\nCapture Complete.");
delay(20);

Serial.println("number of trains: ");
Serial.print(pulseTrains.getSize());
Serial.println("\n");



for(int i = 0; i < pulseTrains.getSize(); i++) {
    for (int j = 0; j < pulseTrains.getPulseTrain(i).getSize(); j++) {
        Serial.print(pulseTrains.getPulseTrain(i).getPulse(j));
        Serial.print(", ");
    }
    Serial.println("--\n");
    if(decode(pulseTrains.getPulseTrainPointer(i), pulseTrains.getPulseTrain(i).getSize())){
        break;
    }
}






// if (C1101preset != CUSTOM) {
// //Serial.println("draw");
// lv_obj_set_scroll_dir(container, LV_DIR_HOR);
// lv_obj_add_flag(container, LV_OBJ_FLAG_SCROLLABLE);

// lv_obj_t *chart = lv_chart_create(container);
// lv_obj_set_size(chart, 1000, 100);  
// lv_chart_set_type(chart, LV_CHART_TYPE_SCATTER);  


// lv_chart_series_t *ser1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);

// lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -10, 40);  

// size_t num_elements = sizeof(pulseTrainVec.data()) / sizeof(pulseTrainVec.data()[0]);
// std::vector<unsigned long> filtered_values;

// bool found_first = false;
// size_t i_sec = 0;
// size_t num_elements_f = 0;

// for (size_t i = 0; i < num_elements; ++i) {
//     if (pulseTrainVec.data()[i] > 1000) {
//         if (found_first) break;         
//         found_first = true;
//         i_sec = i;                       
//         continue;                      
//     }
    
//     if (found_first) {    
//         if (pulseTrainVec.data()[i] > 1000) break;          
//         if (i - i_sec > 20) break;                 

//         filtered_values.push_back(pulseTrainVec.data()[i] / 100);
//         Serial.println(pulseTrainVec.data()[i] / 100);
//         num_elements_f++;
//         continue;
//     }
    
    
// }


// int x_pos = 5;           
// int high_value = 25;     
// int low_value = 5;       
// int pulse_width;    

// lv_chart_set_point_count(chart, 100);  
// Serial.println("yes");

// for (int i = 0; i < num_elements_f; i++) {  
// pulse_width = filtered_values[i];



//     lv_chart_set_next_value2(chart, ser1, x_pos, high_value);
//     lv_chart_set_next_value2(chart, ser1, x_pos + pulse_width, high_value);
    

//     x_pos += pulse_width;


//     lv_chart_set_next_value2(chart, ser1, x_pos, low_value);
//     lv_chart_set_next_value2(chart, ser1, x_pos + pulse_width, low_value);


//     x_pos += pulse_width;

// }

// lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS); 


// lv_obj_scroll_to_x(container, 0, LV_ANIM_OFF);

// lv_chart_refresh(chart);

    

// // }
    //  FlipperSubFile subFile;
    //      bool level = true;
// for (size_t i = 0; i < CC1101_CLASS::allData.size(); ++i) {
//     rawString << "RAW_Data " << i + 1 << ": ";
//     const auto& signal = CC1101_CLASS::allData.getSignal(i); 

//     for (unsigned long sample : signal.samples) { 
//         rawString << sample;

//         if (level) {
//             rawString << " -"; 
//         } else {
//             rawString << " "; 
//         }

//         level = !level; 
//     }

//     rawString << "\n"; 
// }
    CC1101_CLASS::disableReceiver();
    SD_RF.restartSD();

if (!SD_RF.directoryExists("/recordedRF/")) {
    SD_RF.createDirectory("/recordedRF/");
}

String filename = CC1101_CLASS::generateFilename(CC1101_MHZ, CC1101_MODULATION, CC1101_RX_BW);
String fullPath = "/recordedRF/" + filename;
FlipperSubFile subFile;
File32* outputFilePtr = SD_RF.createOrOpenFile(fullPath.c_str(), O_WRITE | O_CREAT);
if (outputFilePtr) {
    File32& outputFile = *outputFilePtr; 
    std::vector<uint8_t> customPresetData;
if (C1101preset == CUSTOM) {
    customPresetData.insert(customPresetData.end(), {
        CC1101_MDMCFG4, ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG4),
        CC1101_MDMCFG3, ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG3),
        CC1101_MDMCFG2, ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG2),
        CC1101_DEVIATN, ELECHOUSE_cc1101.SpiReadReg(CC1101_DEVIATN),
        CC1101_FREND0,  ELECHOUSE_cc1101.SpiReadReg(CC1101_FREND0),
        0x00, 0x00
    });

    std::array<uint8_t, 8> paTable;
    ELECHOUSE_cc1101.SpiReadBurstReg(0x3E, paTable.data(), paTable.size());
    customPresetData.insert(customPresetData.end(), paTable.begin(), paTable.end());
}
subFile.generateRaw(outputFile, C1101preset, customPresetData, rawString, CC1101_MHZ);
SD_RF.closeFile(outputFilePtr);

// CC1101_CLASS::enableReceiver();
// }
// //}
}
}


bool CC1101_CLASS::decode(pulseTrain *pulseTrain, size_t length) {
        if (CC1101_CLASS::receivedData.samples.size() == 0) {
        Serial.println("No pulses to decode.");
        return false;
    }

    if(CC1101.decodeCameAtomoProtocol(CC1101_CLASS::receivedData.samples.data(), CC1101_CLASS::receivedData.samples.size())) {
        return true;
    }
    if(CC1101.decodeCameTweeProtocol(CC1101_CLASS::receivedData.samples.data(), CC1101_CLASS::receivedData.samples.size())) {
        return true;
    }
    if(CC1101.decodeCameProtocol(CC1101_CLASS::receivedData.samples.data(), CC1101_CLASS::receivedData.samples.size())) {
        return true;
    }  
    if(CC1101.decodeNiceFlorSProtocol(CC1101_CLASS::receivedData.samples.data(), CC1101_CLASS::receivedData.samples.size())) {
        return true;
    }
    if(CC1101.decodeNiceFloProtocol(CC1101_CLASS::receivedData.samples.data(), CC1101_CLASS::receivedData.samples.size())) {
        return true;
    }

return false;


  
}


#include <driver/timer.h>
#include <soc/rtc.h>

    std::vector<uint32_t> samplesToSend;   // Holds the durations of high/low states




void CC1101_CLASS::sendRaw() {

            detachInterrupt(CC1101_CCGDO0A);
            detachInterrupt(CC1101_CCGDO2A);

            CC1101_CLASS::initRaw();
            Serial.print(F("\r\nReplaying RAW data from the buffer...\r\n"));

            Signal samplesData = CC1101_CLASS::allData.getSignal(CC1101_CLASS::allData.signals.size() - 1);
            CC1101_CLASS::levelFlag = samplesData.samples[0] > 0; 
            Serial.println(F("Transmitting"));
            Serial.println(samplesData.samples[0]);
            levelFlag ? Serial.println(F("true")) : Serial.println(F("False"));

            CC1101_CLASS::samplesToSend.clear();
            for (size_t j = 0; j < samplesData.samples.size(); ++j) {
                if (samplesData.samples[j] > 0){
                CC1101_CLASS::samplesToSend.push_back(samplesData.samples[j]);
                } else {
                    CC1101_CLASS::samplesToSend.push_back(-samplesData.samples[j]);
                }

                if (samplesData.samples[j] > 20000 || samplesData.samples[j] < -20000) {
                CC1101_CLASS::samplesToSend.clear();
                }
            }

            for (size_t i = 0; i < CC1101_CLASS::samplesToSend.size(); ++i){
                Serial.print(CC1101_CLASS::samplesToSend[i]);
                Serial.print(", ");
            }

            Serial.println("do send");

            for (size_t i = 0; i < CC1101_CLASS::samplesToSend.size(); i++) {

                
                gpio_set_level(CC1101_CCGDO0A, CC1101_CLASS::levelFlag);
                CC1101_CLASS::levelFlag = !CC1101_CLASS::levelFlag;
                delayMicroseconds(CC1101_CLASS::samplesToSend[i]); 
                
            }
            Serial.println(F("Transmission complete"));
            CC1101_CLASS::disableTransmit();

}

void CC1101_CLASS::enableRCSwitch(){
     if (ELECHOUSE_cc1101.getCC1101()){       // Check the CC1101 Spi connection.
  Serial.println("Connection OK");
  }else{
  Serial.println("Connection Error");
  }

//CC1101 Settings:                (Settings with "//" are optional!)
  ELECHOUSE_cc1101.Init();            // must be set to initialize the cc1101!
//ELECHOUSE_cc1101.setRxBW(812.50);  // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
//ELECHOUSE_cc1101.setPA(10);       // set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12)   Default is max!
  ELECHOUSE_cc1101.setMHZ(CC1101_FREQ); // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    mySwitch.setReceiveTolerance(20);
    mySwitch.enableReceive(CC1101_CCGDO2A);  // Receiver on interrupt 0 => that is pin #2

  ELECHOUSE_cc1101.SetRx();  // set Receive on
}



void CC1101_CLASS::initRaw() {
  Serial.print("Init CC1101 raw");

    if(!CC1101_is_initialized) {
        CC1101_CLASS::init();
        CC1101_is_initialized = true;
    } else {
        ELECHOUSE_cc1101.setSidle();
    }

    ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
    ELECHOUSE_cc1101.setGDO0(CC1101_CCGDO0A);         // set lib internal gdo pin (gdo0). Gdo2 not use for this example.
    ELECHOUSE_cc1101.setCCMode(0);          // set config for internal transmission mode. value 0 is for RAW recording/replaying
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    ELECHOUSE_cc1101.setDRate(99.97);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.SetTx();
    
    gpio_set_direction(CC1101_CCGDO0A, GPIO_MODE_OUTPUT); 
}
   
String CC1101_CLASS::generateFilename(float frequency, int modulation, float bandwidth)
{
    char filenameBuffer[32];

    sprintf(filenameBuffer, "%d_%s_%s.sub", static_cast<int>(frequency * 100), presetToString(C1101preset),
            generateRandomString(8).c_str());

    return String(filenameBuffer);
}

String CC1101_CLASS::generateRandomString(int length)
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    const std::string characters = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

    std::stringstream ss;
    for (int i = 0; i < length; ++i) {
        int randomIndex = std::rand() % characters.size();
        char randomChar = characters[randomIndex];
        ss << randomChar;
    }

    return String(ss.str().c_str());
}

void CC1101_CLASS::sendSamples(int timings[], int timingsLength, bool levelFlag)
{
    CC1101_CLASS::initRaw();
    

    
    Serial.print(F("\r\nReplaying RAW data from the buffer...1\r\n"));

    Serial.print("Transmitting\n");
    Serial.print(timingsLength);

    Serial.print("\n----------------\n");

    for (size_t i = 0; i < timingsLength; i++) {

                
        gpio_set_level(CC1101_CCGDO0A, levelFlag);
        levelFlag = !levelFlag;
        delayMicroseconds(timings[i]); 
                
    }




    Serial.print("Transmitted\n");

    digitalWrite(CC1101_CCGDO0A, LOW); 

    Serial.print(F("\r\nReplaying RAW data complete.\r\n\r\n"));
    ELECHOUSE_cc1101.setSidle(); 
    ELECHOUSE_cc1101.goSleep();  
    disableTransmit();
  //  SDInit();
}

void CC1101_CLASS::enableTransmit()
{
    digitalWrite(CC1101_CS, LOW);
    pinMode(CC1101_CCGDO0A, OUTPUT); 
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);               // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION); // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(CC1101_DRATE); // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
    ELECHOUSE_cc1101.setDcFilterOff(1);
    ELECHOUSE_cc1101.setSyncMode(0);
    ELECHOUSE_cc1101.setPktFormat(CC1101_PKT_FORMAT); // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX.
                                                      // 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
                                                      // 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX.
                                                      // 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
                                                      // ELECHOUSE_cc1101.setSyncMode(3);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.

    ELECHOUSE_cc1101.setPA(12);
    gpio_set_pull_mode(CC1101_CCGDO0A, GPIO_FLOATING); 
    ELECHOUSE_cc1101.SetTx();
     

 //   mySwitch.enableTransmit(CC1101_CCGDO0A);
}

void CC1101_CLASS::disableTransmit()
{
    digitalWrite(CC1101_CCGDO0A, LOW);
    mySwitch.disableTransmit(); // set Transmit off
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.goSleep();
    digitalWrite(CC1101_CS, HIGH);
}

void CC1101_CLASS::saveSignal() {
//;
}
