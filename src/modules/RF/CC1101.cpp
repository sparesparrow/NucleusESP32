#include "CC1101.h"
#include "Arduino.h"
//#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include "GUI/ScreenManager.h"
#include <sstream>
#include "GUI/events.h"
#include "SPI.h"

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
//SDcard& SD_RF = SDcard::getInstance();
ScreenManager& screenMgr1 = ScreenManager::getInstance();

using namespace std;
int receiverGPIO;
String rawString;
uint16_t sample[SAMPLE_SIZE];
int error_toleranz = 200;
uint8_t samplecount = 0;
uint8_t minsample = 0;

bool CC1101_is_initialized = false;
bool receiverEnabled = false;
bool CC1101_recieve_is_running = false;
bool CC1101_transmit_is_running = false;
bool CC1101_isiddle = true;
bool CC1101_interup_attached = false;
uint32_t actualFreq;
 unsigned int lastTime1 = 0;

int CC1101_MODULATION = 2;
String fullPath;

//RCSwitch mySwitch;

bool reversed;

// New flag to ensure ISR service installation only happens once.
bool isrServiceInstalled = false;

// Fix: Change type from int to gpio_num_t for proper type checking with ESP-IDF functions.
gpio_num_t activeReceiverPin = GPIO_NUM_NC;

// FreeRTOS semaphore for protecting access to shared data (receivedData)
// Use a recursive mutex (or a simple mutex if only one task accesses outside ISR)
SemaphoreHandle_t samplesMutex;


SignalCollection CC1101_CLASS::allData;
volatile bool recordingStarted = false;
volatile int64_t startRec = false;

CC1101_CLASS::ReceivedData CC1101_CLASS::receivedData;

NucleusESP32::NucleusESP32(IReceivedData& receivedData):
    receivedData(receivedData)
{

}


NucleusESP32::decode():
{

}

NucleusESP32::send():
{

}


// Dummy Interrupt Handler (for external use, keep definition)
void IRAM_ATTR InterruptHandler() {
     if (!receiverEnabled)
    {
        return;
    }
    
    const long time = micros();
    const unsigned int duration = time - lastTime1;

    if (duration > 100000)
    {
        samplecount = 0;
    }

    if (duration >= 100)
    {
        sample[samplecount++] = duration;
    }

    if (samplecount >= SAMPLE_SIZE)
    {
        return;
    }

    if (CC1101_MODULATION == 0)
    {
        if (samplecount == 1 && digitalRead(receiverGPIO) != HIGH)
        {
            samplecount = 0;
        }
    }

    lastTime1 = time;
}

// RCSwitch CC1101_CLASS::getRCSwitch() {
//     Serial.println("CC1101_CLASS::getRCSwitch()");
//     return mySwitch;
// }

bool CC1101_CLASS::init() {
   ELECHOUSE_cc1101.setSpiPin(CC1101_SCLK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
    ELECHOUSE_cc1101.Init();

    if (ELECHOUSE_cc1101.getCC1101())
    {
        ELECHOUSE_cc1101.setGDO0(CC1101_CCGDO0A);
        ELECHOUSE_cc1101.setSidle();
        CC1101_isiddle = true;
        CC1101_is_initialized = true;
        return true;
    }
    else
    {
        ELECHOUSE_cc1101.setSidle();
        CC1101_isiddle = true;
        return false;
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

void CC1101_CLASS::enableReceiver() {
     if (!CC1101_is_initialized) {
        CC1101_CLASS::init();
        //CC1101_CLASS::loadPreset();
    }




    memset(sample, 0, sizeof(SAMPLE_SIZE));
    samplecount = 0;

    if (CC1101_MODULATION == 2)
    {
        ELECHOUSE_cc1101.setDcFilterOff(0);
    } 

    if (CC1101_MODULATION == 0)
    {
        ELECHOUSE_cc1101.setDcFilterOff(1);
    }

    ELECHOUSE_cc1101.setSyncMode(0);  // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
    ELECHOUSE_cc1101.setPktFormat(3); // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX.
                                                      // 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
                                                      // 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX.
                                                      // 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
                                                      // ELECHOUSE_cc1101.setSyncMode(3);       
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION); // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);               // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(CC1101_DRATE); // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
    ELECHOUSE_cc1101.setRxBW(CC1101_RX_BW);  // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
    pinMode(CC1101_CCGDO0A, INPUT);
    receiverGPIO = digitalPinToInterrupt(CC1101_CCGDO0A);    
    ELECHOUSE_cc1101.SetRx();
    /////////////////////////////
    receiverEnabled = true;
    //////////////////////////////
    attachInterrupt(CC1101_CCGDO0A, InterruptHandler, CHANGE);
    Serial.print("interrupt active");
}

void CC1101_CLASS::emptyReceive() {
    Serial.println("CC1101_CLASS::emptyReceive()");
}

void CC1101_CLASS::enableReceiverCustom() {
    Serial.println("CC1101_CLASS::enableReceiverCustom()");
}

void CC1101_CLASS::setCC1101Preset(CC1101_PRESET preset) {
    C1101preset = preset;
}

void CC1101_CLASS::disableReceiver() {
   // gpio_isr_handler_remove(CC1101_CCGDO0A);
  //  gpio_uninstall_isr_service();
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
        // CC1101_SYNC = 0x47;          // Sync word from `Custom_preset_data` (uint8_t 3)
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
        CC1101_SYNC = 0x47;          // Sync word from `Custom_preset_data` (uint8_t 3)
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
         CC1101_SYNC = 0x47;          // Sync word from `Custom_preset_data` (uint8_t 3)
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
        //Serial.println(CC1101_MODULATION);
        break;
}
}

bool CC1101_CLASS::CheckReceived() {
 if(CC1101_CLASS::receivedData.sampleCount  > 2046) {
        CC1101_CLASS::receivedData.sampleCount = 0;
        CC1101_CLASS::receivedData.lastReceiveTime = 0;
        return true;
    }
    else if (CC1101_CLASS::receivedData.sampleCount  < 24 or
            (esp_timer_get_time() - CC1101_CLASS::receivedData.lastReceiveTime) > 3000000) {
            return false;
    }
     else if (CC1101_CLASS::receivedData.sampleCount  > 24 and
            (esp_timer_get_time() - startRec) > 1000000) {
        CC1101_CLASS::receivedData.sampleCount = 0;
        CC1101_CLASS::receivedData.lastReceiveTime = 0;
        return true;
    
    }
    return false;
}

void CC1101_CLASS::fskAnalyze() {
    Serial.println("CC1101_CLASS::fskAnalyze()");
}

void CC1101_CLASS::enableScanner(float start, float stop) {
    Serial.println("CC1101_CLASS::enableScanner()");
}

void CC1101_CLASS::sendByteSequence(const uint8_t sequence[], const uint16_t pulseWidth, const uint8_t messageLength) {
    Serial.println("CC1101_CLASS::sendByteSequence()");
}

void CC1101_CLASS::signalAnalyseTask(void* pvParameters) {
    Serial.println("CC1101_CLASS::signalAnalyseTask()");
}

void CC1101_CLASS::startSignalAnalyseTask() {
    Serial.println("CC1101_CLASS::startSignalAnalyseTask()");
}

void CC1101_CLASS::handleSignal() {
////////////////////////////////////////////
        Signal data;

     //   //Serial.println(F("Raw samples: "));
        CC1101_CLASS::receivedData.sampleCount = 0;
        
        for (const auto &sample : CC1101_CLASS::receivedData.samples) {
            //Serial.print(sample);
            data.addSample(sample);
            //Serial.print(" ");
            CC1101_CLASS::receivedData.sampleCount++;
        }

        CC1101_CLASS::allData.addSignal(data);

        

     lv_obj_t * textareaRC;
    if(C1101preset == CUSTOM){
        textareaRC = screenMgr1.text_area_SubGHzCustom;        
    } else {
        textareaRC = screenMgr1.getTextArea();
    }

    lv_textarea_set_text(textareaRC, "\nRAW signal");//, \nCount: ");

    std::ostringstream  rawString;

    for (const auto& sample : CC1101_CLASS::receivedData.samples) {
        rawString << sample << " ";
    }
}

bool CC1101_CLASS::decode() {
        filterSignal(); // This function call will run the filtering logic.
    Serial.println("/////////////////////////////////////////////////////////");
    for (int i = 0; i < CC1101.receivedData.size(); i++) {
        auto sample = CC1101.receivedData.samples[i];
        Serial.println(sample);
    }

    // 1. BMW Protocol (Timing: ~350us / 700us)
    if ((DURATION_DIFF(pulses[0], 350) < 60) && (DURATION_DIFF(pulses[1], 700) < 100)) {
        if (bmwProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = bmwProtocol.getCodeString();
            updateUI(result); // Pomocná funkce pro zobrazení na displeji
            return true;
        }
    }

    // 2. VW / VAG Group (Manchester / PWM hybrid)
    // VW má často pulse[0] kolem 340-400us
    if ((DURATION_DIFF(pulses[0], 380) < 80)) {
        if (vwProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = vwProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 3. Citroen / PSA (Timing: ~370us / 770us)
    if ((DURATION_DIFF(pulses[0], 370) < 70) && (DURATION_DIFF(pulses[1], 770) < 110)) {
        if (citroenProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = citroenProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 4. Fiat V0 (Manchester, velmi krátké te ~200us)
    if ((DURATION_DIFF(pulses[0], 200) < 50)) {
        if (fiatV0Protocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = fiatV0Protocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 5. Ford V0 (Manchester, timing ~250us)
    if ((DURATION_DIFF(pulses[0], 250) < 50)) {
        if (fordV0Protocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = fordV0Protocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 6. Honda Protocol
    if ((DURATION_DIFF(pulses[0], 430) < 50)) {
        if (hondaProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = hondaProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 7. Hyundai Protocol
    if ((DURATION_DIFF(pulses[0], 250) < 50)) {
        if (hyundaiProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = hyundaiProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 8. Hormann Protocol
    if ((DURATION_DIFF(pulses[0], 500) < 40) && (DURATION_DIFF(pulses[1], 1000) < 90)) {
        if (hormannProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            hormannProtocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 9. Came Protocol
    if ((DURATION_DIFF(pulses[0], 320) < 50) && (DURATION_DIFF(pulses[1], 640) < 90)) {
        if (cameProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            cameProtocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 10. Ansonic Protocol
    if ((DURATION_DIFF(pulses[0], 555) < 40) && (DURATION_DIFF(pulses[1], 1111) < 90)) {
        if (ansonicProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            ansonicProtocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 11. NiceFlo Protocol
    if ((DURATION_DIFF(pulses[0], 700) < 50) && (DURATION_DIFF(pulses[1], 1400) < 90)) {
        if (niceFloProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            niceFloProtocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 12. SMC5326 Protocol
    if ((DURATION_DIFF(pulses[0], 300) < 50) && (DURATION_DIFF(pulses[1], 900) < 90)) {
        if (smc5326Protocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            smc5326Protocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 13. Kia Protocol
    if ((DURATION_DIFF(pulses[0], 250) < 50) && (DURATION_DIFF(pulses[1], 500) < 90)) {
        // Corrected data source and size arguments for filtered data.
        if (kiaProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            kiaProtocol.get_string(pulses[0], pulses[1]);
            return true;
        }
    }

    // 14. KeeLoq Protocol (This part seems to have duplicate code in your file, let's keep one pass)
    else if ((DURATION_DIFF(pulses[0], 400) < 100) && (DURATION_DIFF(pulses[1], 800) < 150)) {
        Serial.println("KeeLoq structure detected (Timing match):");
        for (int i = 0; i < CC1101.receivedData.samples.size(); i++) {
            //   keeloqDecoder.feed(CC1101.receivedData.samples.samples[i] < 0, abs(CC1101.receivedData.samples.samples[i]));
        }
        if (keeloqDecoder.hasResult()) {
            // KeeLoq code processing here...
        }
    }

    // The second set of protocol checks (lines 869-953) in your log are duplicates of the above logic.
    // If you intended to have two passes, ensure the logic is distinct. Otherwise, remove one set.
    // Assuming the first set (above) is the core logic and fixing the second pass as per the log errors:

    // --- SECOND PASS OF PROTOCOL CHECKS --- (Based on log lines 869-953)
    // 1. BMW Protocol (Timing: ~350us / 700us)
    if ((DURATION_DIFF(pulses[0], 350) < 60) && (DURATION_DIFF(pulses[1], 700) < 100)) {
        // Fix: Removed extra parenthesis from if condition and used .data()
        if (bmwProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = bmwProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 2. VW / VAG Group (Manchester / PWM hybrid)
    if ((DURATION_DIFF(pulses[0], 380) < 80)) {
        // Fix: Removed extra parenthesis from if condition and used .data()
        if (vwProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = vwProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 3. Citroen / PSA (Timing: ~370us / 770us)
    if ((DURATION_DIFF(pulses[0], 370) < 70) && (DURATION_DIFF(pulses[1], 770) < 110)) {
        // Fix: Added missing comma between data pointer and size, and used .data()
        if (citroenProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = citroenProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 4. Fiat V0 (Manchester, velmi krátké te ~200us)
    if ((DURATION_DIFF(pulses[0], 200) < 50)) {
        // Fix: Added missing comma between data pointer and size, and used .data()
        if (fiatV0Protocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = fiatV0Protocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 5. Ford V0 Protocol
    if ((DURATION_DIFF(pulses[0], 250) < 50)) {
        if (fordV0Protocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = fordV0Protocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 6. Honda Protocol
    if ((DURATION_DIFF(pulses[0], 430) < 50)) {
        // Fix: Added missing parenthesis from if condition and used .data()
        if (hondaProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = hondaProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 7. Hyundai Protocol
    if ((DURATION_DIFF(pulses[0], 250) < 50)) {
        if (hyundaiProtocol.decode(CC1101.receivedData.samples.data(), CC1101.receivedData.size())) {
            String result = hyundaiProtocol.getCodeString();
            updateUI(result);
            return true;
        }
    }

    // 8. Hormann Protocol
    if ((DURATION_DIFF(pulses[0], 500) < 40) && (DURATION_DIFF(pulses[1], 1000) < 90)) {
        if (hormannProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            hormannProtocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 9. Came Protocol
    if ((DURATION_DIFF(pulses[0], 320) < 50) && (DURATION_DIFF(pulses[1], 640) < 90)) {
        if (cameProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            cameProtocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 10. Ansonic Protocol
    if ((DURATION_DIFF(pulses[0], 555) < 40) && (DURATION_DIFF(pulses[1], 1111) < 90)) {
        if (ansonicProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            ansonicProtocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 11. NiceFlo Protocol
    if ((DURATION_DIFF(pulses[0], 700) < 50) && (DURATION_DIFF(pulses[1], 1400) < 90)) {
        if (niceFloProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            niceFloProtocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 12. SMC5326 Protocol
    if ((DURATION_DIFF(pulses[0], 300) < 50) && (DURATION_DIFF(pulses[1], 900) < 90)) {
        if (smc5326Protocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            smc5326Protocol.getCodeString(pulses[0], pulses[1]);
            return true;
        }
    }

    // 13. Kia Protocol (Second instance)
    if ((DURATION_DIFF(pulses[0], 250) < 50) && (DURATION_DIFF(pulses[1], 500) < 90)) {
        if (kiaProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.size())) {
            kiaProtocol.get_string(pulses[0], pulses[1]);
            return true;
        }
    }

    CC1101_CLASS::receivedData.samples.clear();
    CC1101_CLASS::receivedData.sampleCount = 0;
    return false;
}

void CC1101_CLASS::updateUI(String text) {
    Serial.println("CC1101_CLASS::updateUI()");
}

void CC1101_CLASS::sendRaw() {
    Serial.println("CC1101_CLASS::sendRaw()");
}

void CC1101_CLASS::enableRCSwitch() {
    Serial.println("CC1101_CLASS::enableRCSwitch()");
}

void CC1101_CLASS::initRaw() {
    Serial.println("CC1101_CLASS::initRaw()");
}

String CC1101_CLASS::generateFilename(float frequency, int modulation, float bandwidth) {
    Serial.println("CC1101_CLASS::generateFilename()");
    return "";
}

String CC1101_CLASS::generateRandomString(int length) {
    Serial.println("CC1101_CLASS::generateRandomString()");
    return "";
}

void CC1101_CLASS::sendSamples(int timings[], int timingsLength, bool levelFlag) {
    Serial.println("CC1101_CLASS::sendSamples()");
}

void CC1101_CLASS::enableTransmit() {
    Serial.println("CC1101_CLASS::enableTransmit()");
}

void CC1101_CLASS::disableTransmit() {
    Serial.println("CC1101_CLASS::disableTransmit()");
}

void CC1101_CLASS::saveSignal() {
    Serial.println("CC1101_CLASS::saveSignal()");
}

void CC1101_CLASS::sendEncoded(RFProtocol protocol, float frequency, int16_t bitLenght, int8_t repeats, int64_t code) {
    Serial.println("CC1101_CLASS::sendEncoded()");
}

bool CC1101_CLASS::checkReversed(int64_t big) {
    Serial.println("CC1101_CLASS::checkReversed()");
    return false;
}

void CC1101_CLASS::reverseLogicState() {
    Serial.println("CC1101_CLASS::reverseLogicState()");
}

void CC1101_CLASS::filterAll() {
    Serial.println("CC1101_CLASS::filterAll()");
}

void CC1101_CLASS::filterSignal() {
    Serial.println("CC1101_CLASS::filterSignal()");
}

void CC1101_CLASS::SaveToSD() {
    Serial.println("CC1101_CLASS::SaveToSD()");
}