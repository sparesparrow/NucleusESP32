#include "SDcard.h"
#include "globals.h"
#include <SD.h>
#include <SPI.h>

SPIClass sd(SDCARD_SPI_HOST);

bool SDInit() {
    // Begin SPI with specific pins
    SPI.begin(SDCARD_SCK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);
    
    // Ensure CC1101 is disabled
    digitalWrite(CC1101_CS, HIGH);  
    
    // Begin SD card initialization
    if (!SD.begin(SDCARD_CS)) {
        Serial.println("SD Card MOUNT FAIL");
        return false;
    } else {
        Serial.println("SD Card MOUNT SUCCESS");
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.println("SDCard Size: " + String(cardSize) + "MB");
        return true;
    }
}

String disconnectSD() {
    SPI.endTransaction();
    digitalWrite(SDCARD_CS, HIGH);  
    return "SD card disconnected.\n";
}

#define MAX_LENGHT_RAW_ARRAY 4096

float tempFreq;
int tempSample[MAX_LENGHT_RAW_ARRAY];
int tempSampleCount;

bool read_sd_card_flipper_file(String filename)
{
        if (!SDInit()) {
        Serial.println("SD mounting failed!");
        return false;
    } else {
        Serial.println("SD mounted");
    }
  Serial.print("Read Flipper File");

    File file = SD.open(filename, FILE_READ);
    if (!file)
    {
        Serial.println("Failed to open file: " + String(filename));
        return false;
    }

    // Reset Current
    memset(tempSample, 0, sizeof(MAX_LENGHT_RAW_ARRAY));       
    tempSampleCount = 0;

    char *buf = (char *) malloc(MAX_LENGHT_RAW_ARRAY);
    String line = "";

    while (file.available())
    {
        line = file.readStringUntil('\n');
        line.toCharArray(buf, MAX_LENGHT_RAW_ARRAY);
        const char sep[2] = ":";
        const char values_sep[2] = " ";

        char *key = strtok(buf, sep);
        char *value;

        if (key != NULL)
        {
            value = strtok(NULL, sep);

            if (!strcmp(key, "Frequency"))
            {
                tempFreq = atoi(value) / 1000000.0f;
            }

            if (!strcmp(key, "RAW_Data"))
            {
                char *pulse = strtok(value, values_sep);
                int i;
                while (pulse != NULL && tempSampleCount < MAX_LENGHT_RAW_ARRAY)
                {
                    tempSample[tempSampleCount] = atoi(pulse);
                    pulse = strtok(NULL, values_sep);
                    tempSampleCount++;
                }
            }
        }
    }

    file.close();

    free(buf);

  return true;
}

