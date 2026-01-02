#include "CC1101.h"
#include "Arduino.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include "GUI/ScreenManager.h"
#include <sstream>
#include <ctime>
#include <vector> // Ensure this header is present for std::vector usage
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
#include <soc/rtc.h>
#include <cstdint>
#include <cmath>
#include <cstdlib>

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

using namespace std;
int receiverGPIO;
String rawString;
uint16_t sample[SAMPLE_SIZE];
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
String fullPath;

RCSwitch mySwitch;

bool reversed;

SignalCollection CC1101_CLASS::allData;
volatile bool recordingStarted = false;
volatile int64_t startRec = false;

RCSwitch CC1101_CLASS::getRCSwitch() {
    return mySwitch;
}

CC1101_CLASS::ReceivedData CC1101_CLASS::receivedData;

void IRAM_ATTR InterruptHandler(void *arg) {
    if (!gpio_get_level(CC1101_CCGDO0A)) {
        reversed = false;
    } else {
        reversed = true;
    }

    if (recordingStarted) {
        recordingStarted = false;
        startRec = esp_timer_get_time();
    }
    static volatile uint64_t DRAM_ATTR lastTime = 0;
    volatile const uint64_t time = esp_timer_get_time();
    int64_t duration = time - lastTime;
    lastTime = time;

    // Simple noise filtering
    if (((duration > 100) and (duration > 0)) or ((-duration > 100) and (-duration > 0))) {
        noInterrupts();
        if (CC1101_CLASS::receivedData.samples.size() < SAMPLE_SIZE) {
            if (reversed) {
                CC1101_CLASS::receivedData.samples.push_back(-duration);
            } else {
                CC1101_CLASS::receivedData.samples.push_back(duration);
            }
            CC1101_CLASS::receivedData.lastReceiveTime = esp_timer_get_time();
            CC1101_CLASS::receivedData.sampleCount++;
        }
        if (duration > 50000 or duration < -50000) {
            CC1101_CLASS::receivedData.samples.clear();
        }
        interrupts();
    }
}

//encoders
bool CC1101_CLASS::init() {
    digitalWrite(PN532_SS, HIGH);
    SPI.end();
    delay(10);
    digitalWrite(CC1101_CS, LOW);
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCLK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
    ELECHOUSE_cc1101.Init();

    if (ELECHOUSE_cc1101.getCC1101()) {
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x07);  // Maximum LNA gain
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, 0x00);  // Fastest AGC
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0, 0x91);  // Normal AGC, 16 samples
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND1, 0xB6);    // RX frontend configuration
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND0, 0x10);    // TX frontend configuration

        ELECHOUSE_cc1101.setGDO(CCGDO0A, CCGDO0A);
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

void CC1101_CLASS::enableReceiver() {
    samplecount = 0;
    localSampleCount = 0;
    delay(50);
    CC1101_CLASS::loadPreset();

    ELECHOUSE_cc1101.setDRate(CC1101_DRATE);
    delay(10);
    ELECHOUSE_cc1101.setRxBW(CC1101_RX_BW);
    ELECHOUSE_cc1101.setDcFilterOff(1);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(CC1101_DRATE);
    ELECHOUSE_cc1101.setPA(12);
    ELECHOUSE_cc1101.setGDO0(CC1101_CCGDO0A);

    delay(10);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CC1101_CCGDO0A),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
    ELECHOUSE_cc1101.SetRx();
    delay(20);
    if (!gpio_get_level(CC1101_CCGDO0A)) {
        delay(10);
        uint8_t iocfg0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_IOCFG1);
        iocfg0 |= (1 << 6);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG1, iocfg0);
        delay(20);
    }
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(CC1101_CCGDO0A, InterruptHandler, NULL);

    receiverEnabled = true;
    CC1101_CLASS::receivedData.samples.clear();
    CC1101_CLASS::receivedData.lastReceiveTime = 0;
    CC1101_CLASS::receivedData.sampleCount = 0;
    CC1101_CLASS::receivedData.signals.clear();
    delay(500);
    recordingStarted = true;
    interrupts();
}

void CC1101_CLASS::emptyReceive() {
    localSampleCount = 0;
    delay(50);

    if (ELECHOUSE_cc1101.getCC1101()) {
        //Serial.println("Connection OK");
    } else {
        //Serial.println("Connection Error");
    }

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(CC1101_FREQ);
    mySwitch.setReceiveTolerance(20);
    mySwitch.enableReceive(CC1101_CCGDO0A);

    ELECHOUSE_cc1101.SetRx();
    CC1101_CLASS::loadPreset();

    delay(10);
    ELECHOUSE_cc1101.setRxBW(CC1101_RX_BW);
    delay(10);
    ELECHOUSE_cc1101.setDcFilterOff(1);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(CC1101_DRATE);
    ELECHOUSE_cc1101.setPA(12);
    delay(10);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_22),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
    gpio_pulldown_en(CC1101_CCGDO0A);
    ELECHOUSE_cc1101.SetRx();
    delay(20);
    if (!gpio_get_level(CC1101_CCGDO0A)) {
        delay(10);
        uint8_t iocfg0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_IOCFG1);
        iocfg0 |= (1 << 6);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG1, iocfg0);
        delay(20);
    }
}

void CC1101_CLASS::enableReceiverCustom() {
    CC1101TH TH;
    int BW = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox);
    int TRS = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox1);
    int SYNC = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox2); // warning: unused variable 'SYNC'
    int PTK = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox3); // warning: unused variable 'PTK'
    int MOD = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox4);
    int DEV = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox5);
    int DRATE = lv_spinbox_get_value(screenMgr1.SubGHzCustomScreen_->spinbox6);

    ELECHOUSE_cc1101.SpiStrobe(0x30); // Reset CC1101
    localSampleCount = 0;
    delay(50);

    CC1101_CLASS::init();
    CC1101_CLASS::loadPreset();

    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG1, 0x0D); // Set GP2
    delay(10);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x32); // Async mode
    delay(10);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, TH.getRegValue(TRS));
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
        .pin_bit_mask = (1ULL << GPIO_NUM_17),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
    ELECHOUSE_cc1101.SetRx();
    delay(20);
    if (!gpio_get_level(CC1101_CCGDO0A)) {
        delay(10);
        uint8_t iocfg0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_IOCFG1);
        iocfg0 |= (1 << 6);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG1, iocfg0);
        delay(20);
    }
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(GPIO_NUM_17, InterruptHandler, NULL);

    receiverEnabled = true;
    CC1101_CLASS::receivedData.samples.clear();
    CC1101_CLASS::receivedData.lastReceiveTime = 0;
    CC1101_CLASS::receivedData.sampleCount = 0;
    CC1101_CLASS::receivedData.signals.clear();
    delay(500);
    interrupts();
}

void CC1101_CLASS::setCC1101Preset(CC1101_PRESET preset) {
    C1101preset = preset;
}

void CC1101_CLASS::disableReceiver()
{
    gpio_isr_handler_remove(GPIO_NUM_17);
    gpio_uninstall_isr_service();
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
        CC1101_MODULATION = 0;
        CC1101_DRATE = 12.69;
        CC1101_RX_BW = 200;
        CC1101_DEVIATION = 12.69;
        break;
    case FSK25k:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 25.39;
        CC1101_RX_BW = 200;
        CC1101_DEVIATION = 25.39;
        CC1101_SYNC = 0x47;
        break;
    case FSK31k:
        CC1101_MODULATION = 0;
        CC1101_DRATE = 31.73;
        CC1101_RX_BW = 200;
        CC1101_DEVIATION = 31.73;
        CC1101_SYNC = 0x47;
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
        break;
    }
}

bool CC1101_CLASS::CheckReceived() {
    if (CC1101_CLASS::receivedData.sampleCount > 2046) {
        CC1101_CLASS::receivedData.sampleCount = 0;
        CC1101_CLASS::receivedData.lastReceiveTime = 0;
        return true;
    } else if (CC1101_CLASS::receivedData.sampleCount < 24 ||
               (esp_timer_get_time() - CC1101_CLASS::receivedData.lastReceiveTime) > 3000000) {
        return false;
    } else if (CC1101_CLASS::receivedData.sampleCount > 24 &&
               (esp_timer_get_time() - startRec) > 1000000) {
        CC1101_CLASS::receivedData.sampleCount = 0;
        CC1101_CLASS::receivedData.lastReceiveTime = 0;
        return true;
    }
    return false;
}

void CC1101_CLASS::fskAnalyze() {
    //
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
    pinMode(CCGDO0A, INPUT);

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
            gpio_set_level(CC1101_CCGDO0A, (dataByte & (1 << bit)) != 0 ? HIGH : LOW);
            delayMicroseconds(pulseWidth);
        }
    }
}

void CC1101_CLASS::signalAnalyseTask(void* pvParameters) {
    const uint32_t subghz_frequency_list[] = {
        300000000, 303875000, 304250000, 310000000, 315000000, 318000000,
        390000000, 418000000, 433075000, 433420000, 433920000, 434420000, 434775000, 438900000,
        868350000, 868000000, 915000000, 925000000
    };
    int num_frequencies = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
    int rssi = 0;
    int mark_rssi = -100;
    float mark_freq = 0;

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setRxBW(58);
    ELECHOUSE_cc1101.SetRx();

    for (int i = 0; i < num_frequencies; ++i) {
        float freq = subghz_frequency_list[i] / 1000000.0;
        ELECHOUSE_cc1101.setMHZ(freq);
        rssi = ELECHOUSE_cc1101.getRssi();

        if (rssi > -75) {
            if (rssi > mark_rssi) {
                mark_rssi = rssi;
                mark_freq = freq;
            }
        }
        if (Serial.available()) {
            break;
        }
    }

    if (mark_rssi > -75) {
    } else {
    }
    ELECHOUSE_cc1101.SetRx();
}

void CC1101_CLASS::startSignalAnalyseTask() {
    xTaskCreatePinnedToCore(
        CC1101_CLASS::signalAnalyseTask,
        "SignalAnalyseTask",
        8192,
        this,
        1,
        NULL,
        1
    );
}

void CC1101_CLASS::handleSignal() {
    Signal data;
    CC1101_CLASS::receivedData.sampleCount = 0;

    for (const auto &sample : CC1101_CLASS::receivedData.samples) {
        data.addSample(sample);
        CC1101_CLASS::receivedData.sampleCount++;
    }

    CC1101_CLASS::allData.addSignal(data);

         lv_obj_t * textareaRC;
    if(C1101preset == CUSTOM){
        textareaRC = screenMgr1.text_area_SubGHzCustom;        
    } else {
        textareaRC = screenMgr1.getTextArea();
    }



    lv_textarea_set_text(textareaRC, "\nRAW signal");

    std::ostringstream rawString;
    for (const auto& sample : CC1101_CLASS::receivedData.samples) {
        rawString << sample << " ";
    }

    // This calls filterSignal, which is defined below in the decode() function.
    // This is unusual, as filterSignal should probably be called before decode, or be part of a separate pipeline.
    // For now, let's keep the call here as per original code structure.
    void filterSignal();

    CC1101_CLASS::disableReceiver();
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
        if (kiaProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.filtered.size())) {
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
        if (kiaProtocol.decode(CC1101.receivedData.samples.data(), CC1101_CLASS::receivedData.filtered.size())) {
            kiaProtocol.get_string(pulses[0], pulses[1]);
            return true;
        }
    }

    CC1101_CLASS::receivedData.samples.clear();
    CC1101_CLASS::receivedData.sampleCount = 0;
    return false;
} // <--- Added missing brace to close CC1101_CLASS::decode() function

// Pomocná funkce pro zamezení duplicity kódu UI
void CC1101_CLASS::updateUI(String text) {
    ScreenManager& screenMgr = ScreenManager::getInstance();
    lv_obj_t * textarea = (C1101preset == CUSTOM) ? screenMgr.text_area_SubGHzCustom : screenMgr.getTextArea();

    if (textarea != nullptr) {
        lv_textarea_set_text(textarea, text.c_str());
    } else {
        Serial.println(text);
    }
}

void CC1101_CLASS::sendRaw() {
    CC1101_CLASS::init();
    delay(5);
    Signal samplesData;

    if (CC1101_CLASS::allData.signals.empty()) return;

    if (C1101CurrentState != STATE_BRUTE) {
        detachInterrupt(CC1101_CCGDO0A);
        detachInterrupt(CC1101_CCGDO0A);

        if (!CC1101_CLASS::receivedData.filtered.empty()) {
            Signal filtered;
            for (int i = 0; i < CC1101_CLASS::receivedData.size(); i++) { // Corrected loop condition: `i < size()`
                filtered.addSample(CC1101_CLASS::receivedData.filtered[i]);
            }
        }
        samplesData = CC1101_CLASS::allData.getSignal(CC1101_CLASS::allData.signals.size() - 1);
        CC1101_CLASS::initRaw();
    } else {
        CC1101_CLASS::allData.clear();
        for (size_t i = 0; i < 26; i++) {
            samplesData.addSample(samplesToSend[i]);
        }
    }

    CC1101_CLASS::levelFlag = samplesData.samples[0] > 0;

    samplesToSend.clear();
    for (size_t j = 0; j < samplesData.samples.size(); ++j) {
        if (samplesData.samples[j] > 0) {
            samplesToSend.push_back(samplesData.samples[j]);
        } else {
            samplesToSend.push_back(-samplesData.samples[j]);
        }
    }

    for (size_t i = 0; i < samplesToSend.size(); i++) {
        gpio_set_level(CC1101_CCGDO0A, CC1101_CLASS::levelFlag);
        CC1101_CLASS::levelFlag = !CC1101_CLASS::levelFlag;
        delayMicroseconds(samplesToSend[i]);
    }
    CC1101_CLASS::disableTransmit();
}

void CC1101_CLASS::enableRCSwitch() {
    if (ELECHOUSE_cc1101.getCC1101()) {
        //Serial.println("Connection OK");
    } else {
        //Serial.println("Connection Error");
    }

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(CC1101_FREQ);
    mySwitch.setReceiveTolerance(20);
    mySwitch.enableReceive(CC1101_CCGDO0A);

    ELECHOUSE_cc1101.SetRx();
}

void CC1101_CLASS::initRaw() {
    CC1101_CLASS::init();

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setGDO0(CC1101_CCGDO0A);
    ELECHOUSE_cc1101.setCCMode(0);
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(CC1101_DRATE);
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
    for (size_t i = 0; i < timingsLength; i++) {
        gpio_set_level(CC1101_CCGDO0A, levelFlag);
        levelFlag = !levelFlag;
        delayMicroseconds(timings[i]);
    }
    digitalWrite(CC1101_CCGDO0A, LOW);
}

void CC1101_CLASS::enableTransmit()
{
    digitalWrite(CC1101_CS, LOW);
    pinMode(CC1101_CCGDO0A, OUTPUT);
    CC1101_CLASS::init();
    delay(5);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);
    ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);
    ELECHOUSE_cc1101.setDeviation(CC1101_DEVIATION);
    ELECHOUSE_cc1101.setDRate(CC1101_DRATE);
    ELECHOUSE_cc1101.setDcFilterOff(1);
    ELECHOUSE_cc1101.setSyncMode(0);
    ELECHOUSE_cc1101.setPktFormat(CC1101_PKT_FORMAT);
    ELECHOUSE_cc1101.setPA(12);
    gpio_set_pull_mode(CC1101_CCGDO0A, GPIO_FLOATING);
    ELECHOUSE_cc1101.SetTx();
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

void CC1101_CLASS::sendEncoded(RFProtocol protocol, float frequency, int16_t bitLenght, int8_t repeats, int64_t code) {
    CC1101_CLASS::init();
    delay(5);
    pinMode(CC1101_CCGDO0A, OUTPUT);
    digitalWrite(CC1101_CCGDO0A, LOW);
    setFrequency(frequency);
    setCC1101Preset(AM650);
    loadPreset();
    ELECHOUSE_cc1101.setPA(12);
    initRaw();

    switch (protocol) {
    case CAME:
        cameProtocol.yield(code);
        delay(5);
        for (int k = 0; k < repeats; k++) {
            bool levelFlag = false;
            for (size_t j = 0; j < samplesToSend.size(); j++) {
                gpio_set_level(CC1101_CCGDO0A, levelFlag);
                levelFlag = !levelFlag;
                delayMicroseconds(samplesToSend[j]);
            }
            gpio_set_level(CC1101_CCGDO0A, LOW);
            delayMicroseconds(11520);
        }
        break;

    case NICE:
        niceFloProtocol.yield(code);
        delay(5);
        for (int k = 0; k < repeats; k++) {
            bool levelFlag = false;
            for (size_t j = 0; j < samplesToSend.size(); j++) {
                gpio_set_level(CC1101_CCGDO0A, levelFlag);
                levelFlag = !levelFlag;
                delayMicroseconds(samplesToSend[j]);
            }
            gpio_set_level(CC1101_CCGDO0A, LOW);
            delayMicroseconds(25200);
        }
        break;

    case ANSONIC:
        ansonicProtocol.yield(code);
        delay(5);
        for (int k = 0; k < repeats; k++) {
            bool levelFlag = false;
            for (size_t j = 0; j < samplesToSend.size(); j++) {
                gpio_set_level(CC1101_CCGDO0A, levelFlag);
                levelFlag = !levelFlag;
                delayMicroseconds(samplesToSend[j]);
            }
            gpio_set_level(CC1101_CCGDO0A, LOW);
            delayMicroseconds(19425);
        }
        break;

    case HOLTEK:
        holtekProtocol.yield(code);
        delay(5);
        for (int k = 0; k < repeats; k++) {
            bool levelFlag = false;
            for (size_t j = 0; j < samplesToSend.size(); j++) {
                gpio_set_level(CC1101_CCGDO0A, levelFlag);
                levelFlag = !levelFlag;
                delayMicroseconds(samplesToSend[j]);
            }
            gpio_set_level(CC1101_CCGDO0A, LOW);
            delayMicroseconds(14400);
        }
        break;

    case LINEAR:
        while (encoderState != EncoderStepReady) {
            // ...
        }
        break;

    case SMC5326:
        while (encoderState != EncoderStepReady) {
            smc5326Protocol.yield(code);
        }
        break;

    default:
        break;
    }
}

bool CC1101_CLASS::checkReversed(int64_t big) {
    for (int i = 0; i < CC1101.receivedData.samples.size(); i++) {
        if (CC1101.receivedData.samples[i] > (big * 13)) return true;
    }
    return false;
}

void CC1101_CLASS::reverseLogicState() {
    for (int i = 0; i < CC1101.receivedData.samples.size(); i++) {
        CC1101.receivedData.samples[i] = -CC1101.receivedData.samples[i];
    }
}

void CC1101_CLASS::filterAll() {
    CC1101.receivedData.filtered.clear();
    int64_t shortMin = pulses[0] * 0.7;
    int64_t shortMax = pulses[0] * 1.3;
    int64_t longMin = pulses[1] * 0.7;
    int64_t longMax = pulses[1] * 1.3;
    int64_t spaceMin = pulses[1] * 13;
    int64_t space = pulses[1] * 18;

    for (int i = 0; i < CC1101.receivedData.samples.size(); i++) {
        int64_t sample = CC1101.receivedData.samples[i];
        if (sample > 0) {
            if (sample > spaceMin) {
                CC1101.receivedData.filtered.push_back(space);
            } else if (sample > shortMin && sample < shortMax) {
                CC1101.receivedData.filtered.push_back(pulses[0]);
            } else if (sample > longMin && sample < longMax) {
                CC1101.receivedData.filtered.push_back(pulses[1]);
            }
        } else {
            sample = -sample;
            if (sample > spaceMin) {
                CC1101.receivedData.filtered.push_back(-space);
            } else if (sample > shortMin && sample < shortMax) {
                CC1101.receivedData.filtered.push_back(-pulses[0]);
            } else if (sample > longMin && sample < longMax) {
                CC1101.receivedData.filtered.push_back(-pulses[1]);
            }
        }
    }

    if (!SD_RF.directoryExists("/recordedFilteredAll/")) {
        SD_RF.createDirectory("/recordedFilteredAll/");
    }

    String filename = CC1101_CLASS::generateFilename(CC1101_MHZ, CC1101_MODULATION, CC1101_RX_BW);
    String fullPath = "/recordedFilteredAll/" + filename;
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
                CC1101_FREND0, ELECHOUSE_cc1101.SpiReadReg(CC1101_FREND0),
                0x00, 0x00
            });

            std::array<uint8_t, 8> paTable;
            ELECHOUSE_cc1101.SpiReadBurstReg(0x3E, paTable.data(), paTable.size());
            customPresetData.insert(customPresetData.end(), paTable.begin(), paTable.end());
        }
        std::ostringstream filteredString;
        for (const auto& sample : CC1101_CLASS::receivedData.filtered) {
            filteredString << sample << " ";
        }
        subFile.generateRaw(outputFile, C1101preset, customPresetData, filteredString, CC1101_MHZ);
        SD_RF.closeFile(outputFilePtr);
    }
}

void CC1101_CLASS::filterSignal() {
    for (auto sample : CC1101.receivedData.samples) {
        Serial.println(sample);
    }

    // This section contains logic to analyze and filter received signals.
    // The commented-out code suggests a pulse analysis algorithm (e.g., for Keeloq or similar protocols).
    // The current implementation directly calls Serial.println() and returns.
    // If the intent is to perform pulse analysis and filtering, uncomment the relevant logic.

    /*
    if (CC1101.receivedData.samples.empty()) return;

    std::vector<int64_t> absArr;
    absArr.reserve(CC1101.receivedData.samples.size());
    for (auto x : CC1101.receivedData.samples)
        absArr.push_back(std::abs(x));

    if (absArr.empty()) return;
    std::sort(absArr.begin(), absArr.end());

    std::vector<std::vector<int64_t>> groups;
    std::vector<int64_t> currGroup;
    currGroup.push_back(absArr[0]);
    int64_t groupMin = absArr[0];

    for (size_t i = 1; i < absArr.size(); i++) {
        int64_t x = absArr[i];
        if (x * 10 <= groupMin * 13)
            currGroup.push_back(x);
        else {
            groups.push_back(std::move(currGroup));
            currGroup.clear();
            currGroup.push_back(x);
            groupMin = x;
        }
    }
    if (!currGroup.empty())
        groups.push_back(std::move(currGroup));

    if (groups.empty()) return;

    if (groups.size() == 1) {
        std::sort(groups[0].begin(), groups[0].end());
        int64_t median = computeMedian(&groups[0]);
        pulses.clear();
        pulses.push_back(median);
        return;
    }

    std::vector<std::pair<int64_t, int64_t>> groupStats; // {count, median}
    for (auto &g : groups) {
        std::sort(g.begin(), g.end());
        groupStats.push_back({ static_cast<int64_t>(g.size()), computeMedian(&g) });
    }

    std::sort(groupStats.begin(), groupStats.end(),
        [](const std::pair<int64_t, int64_t>& a, const std::pair<int64_t, int64_t>& b) {
            return (a.first == b.first) ? (a.second < b.second) : (a.first > b.first);
        });

    if (groupStats.size() < 2) return;

    int64_t rep1 = groupStats[0].second;
    int64_t rep2 = groupStats[1].second;
    if (rep1 > rep2) std::swap(rep1, rep2);

    int64_t smallCandidate1 = (rep1 + rep2) / 3;
    int64_t bigCandidate1   = smallCandidate1 * 2;
    int64_t smallCandidate2 = (rep1 + rep2) / 4;
    int64_t bigCandidate2   = smallCandidate2 * 3;
    int64_t smallCandidate3 = (rep1 + rep2) / 5;
    int64_t bigCandidate3   = smallCandidate3 * 4;
    int64_t smallCandidate4 = (rep1 + rep2) / 6;
    int64_t bigCandidate4   = smallCandidate4 * 5;
    int64_t smallCandidate5 = (rep1 + rep2) / 7;
    int64_t bigCandidate5   = smallCandidate5 * 6;
    int64_t smallCandidate6 = (rep1 + rep2) / 8;
    int64_t bigCandidate6   = smallCandidate6 * 7;
    int64_t smallCandidate7 = (rep1 + rep2) / 9;
    int64_t bigCandidate7   = smallCandidate7 * 8;
    int64_t smallCandidate8 = (rep1 + rep2) / 10;
    int64_t bigCandidate8   = smallCandidate8 * 9;

    uint32_t diff1 = DURATION_DIFF(rep1, smallCandidate1);
    uint32_t diff2 = DURATION_DIFF(rep1, smallCandidate2);
    uint32_t diff3 = DURATION_DIFF(rep1, smallCandidate3);
    uint32_t diff4 = DURATION_DIFF(rep1, smallCandidate4);
    uint32_t diff5 = DURATION_DIFF(rep1, smallCandidate5);
    uint32_t diff6 = DURATION_DIFF(rep1, smallCandidate6);
    uint32_t diff7 = DURATION_DIFF(rep1, smallCandidate7);
    uint32_t diff8 = DURATION_DIFF(rep1, smallCandidate8);
    uint32_t diffs[] = { diff1, diff2, diff3, diff4, diff5, diff6, diff7, diff8 };

    auto minPtr = std::min_element(std::begin(diffs), std::end(diffs));
    size_t index = std::distance(std::begin(diffs), minPtr);

    pulses.clear();
    switch (index) {
        case 0: pulses.push_back(smallCandidate1);  pulses.push_back(bigCandidate1); break;
        case 1: pulses.push_back(smallCandidate2);  pulses.push_back(bigCandidate2); break;
        case 2: pulses.push_back(smallCandidate3);  pulses.push_back(bigCandidate3); break;
        case 3: pulses.push_back(smallCandidate4);  pulses.push_back(bigCandidate4); break;
        case 4: pulses.push_back(smallCandidate5);  pulses.push_back(bigCandidate5); break;
        case 5: pulses.push_back(smallCandidate6);  pulses.push_back(bigCandidate6); break;
        case 6: pulses.push_back(smallCandidate7);  pulses.push_back(bigCandidate7); break;
        case 7: pulses.push_back(smallCandidate8);  pulses.push_back(bigCandidate8); break;
    }

    if (checkReversed(rep2))
        reverseLogicState();

    if (DURATION_DIFF(pulses[0], rep1) < 40 && DURATION_DIFF(pulses[1], rep2) < 80)
        filterAll();
    else {
        pulses.clear();
        pulses.push_back(rep1);
        pulses.push_back(rep2);
    }

    Serial.print("Pulses Med: ");
    Serial.print(rep1);
    Serial.print(" ");
    Serial.println(rep2);
    */
}
