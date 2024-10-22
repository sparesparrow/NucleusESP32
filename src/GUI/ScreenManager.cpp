#include "ScreenManager.h"
#include "KeyboardHelper.h"
#include "ContainerHelper.h"
#include "ButtonHelper.h"
#include "globals.h"
#include "events.h"
//#include "settingsButton.h"
#include "FS.h"
#//include "fileBrowserHelper.h"
#include "modules/ETC/SDcard.h"
#include "modules/dataProcessing/SubGHzParser.h"
#include "modules/dataProcessing/dataProcessing.h"



#define MAX_PATH_LENGTH 256

EVENTS events;



char** file_list;  // Array of strings for file names
int file_count = 0;
lv_obj_t* list;

const char* pathBUffer;

lv_obj_t* previous_screen = NULL;  // To store the previous screen


// Singleton instance management
ScreenManager &ScreenManager::getInstance()
{
    static ScreenManager instance;
    return instance;
}

// Constructor
ScreenManager::ScreenManager()
    : ReplayScreen_(nullptr),
      SourAppleScreen_(nullptr),  
      text_area_(nullptr),
      freqInput_(nullptr),
      settingsButton_(nullptr),
      filenameInput_(nullptr),
      kb_freq_(nullptr),
      kb_qwert_(nullptr),
      fileName_container_(nullptr),
      topLabel_container_(nullptr),
      RCSwitchMethodScreen_(nullptr),
      button_container1_(nullptr),
      button_container2_(nullptr),
      C1101preset_container_(nullptr),
      C1101PTK_container_(nullptr),
      C1101PTK_dropdown_(nullptr),
      C1101SYNC_container_(nullptr),
      C1101pulseLenght_container_(nullptr),
      pulseLenghInput_(nullptr),
      topLabel_RCSwitchMethod_container_(nullptr),
      secondLabel_container_(nullptr),
      text_area_replay(nullptr),
      button_container_RCSwitchMethod2_(nullptr),
      button_container_RCSwitchMethod1_(nullptr)
      
{
}

// Destructor
ScreenManager::~ScreenManager()
{
    // Cleanup if necessary
}

lv_obj_t *ScreenManager::getFreqInput()
{
    return freqInput_;
}

lv_obj_t *ScreenManager::getTextArea()
{
    return text_area_replay; 
}

lv_obj_t *ScreenManager::getTextAreaRCSwitchMethod()
{
    return text_area_;
}

lv_obj_t *ScreenManager::getTextAreaSourAple()
{
    return text_area_SourApple;
}

lv_obj_t *ScreenManager::getPulseLenghtInput()
{
    return pulseLenghInput_;
}

lv_obj_t *ScreenManager::getFilenameInput()
{
    return filenameInput_;
}

lv_obj_t *ScreenManager::getKeyboardFreq()
{
    return kb_freq_;
}

lv_obj_t *ScreenManager::getPresetDropdown()
{
    return C1101buffer_dropdown_;
}

lv_obj_t *ScreenManager::getSyncDropdown()
{
    return C1101SYNC_dropdown_;
}

lv_obj_t *ScreenManager::getPTKDropdown()
{
    return C1101PTK_dropdown_;
}

void ScreenManager::createReplayScreen() {
    ContainerHelper containerHelper;

    ReplayScreen_ = lv_obj_create(NULL);

    lv_scr_load(ReplayScreen_);
    lv_obj_set_flex_flow(ReplayScreen_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ReplayScreen_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);


    containerHelper.createContainer(&topLabel_container_, ReplayScreen_, LV_FLEX_FLOW_ROW, 35, 240);
    lv_obj_set_style_border_width(topLabel_container_, 0, LV_PART_MAIN);


    kb_qwert_ = KeyboardHelper::createKeyboard(ReplayScreen_, LV_KEYBOARD_MODE_TEXT_LOWER);
    kb_freq_ = KeyboardHelper::createKeyboard(ReplayScreen_, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(kb_freq_, freqInput_);


    containerHelper.fillTopContainer(topLabel_container_, "Mhz:  ", TEXT_AREA, &freqInput_, "433.92", "433.92", 10, NULL, NULL);
    lv_obj_set_size(freqInput_, 70, 30);                   
    lv_obj_add_event_cb(freqInput_, EVENTS::ta_freq_event_cb, LV_EVENT_ALL, kb_freq_);


    containerHelper.createContainer(&secondLabel_container_, ReplayScreen_, LV_FLEX_FLOW_ROW, 35, 240);
    lv_obj_set_style_border_width(secondLabel_container_, 0, LV_PART_MAIN);

    lv_obj_t * C1101preset_dropdown_ = lv_dropdown_create(secondLabel_container_);
    lv_dropdown_set_options(C1101preset_dropdown_, "AM650\n"
                                "AM270\n"
                                "FM238\n"
                                "FM476\n"
                                "FM95\n"
                                "FM15k\n"
                                "PAGER\n"
                                "HND1\n"
                                "HND2\n"
                                );

    lv_obj_add_event_cb(C1101preset_dropdown_, EVENTS::ta_preset_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    buttonSettings_ = lv_btn_create(secondLabel_container_);
    lv_obj_set_size(buttonSettings_, 90, 30);
    // Create label inside the button, making the button parent of the label
    lv_obj_t * labelButton = lv_label_create(buttonSettings_); 
    lv_label_set_text(labelButton, "Settings");  // Set label text as the placeholder
    lv_obj_center(labelButton);  // Center the label inside the butto
    lv_obj_clear_flag(buttonSettings_, LV_OBJ_FLAG_SCROLLABLE); // Double ensure
    lv_obj_add_event_cb(buttonSettings_, EVENTS::createRFSettingsScreen, LV_EVENT_CLICKED, NULL);

    // Create main text area
    text_area_replay = lv_textarea_create(ReplayScreen_);
    lv_obj_set_size(text_area_replay, 240, 140);
    lv_obj_align(text_area_replay, LV_ALIGN_CENTER, 0, -20);
    lv_textarea_set_text(text_area_replay, "RAW protocol tool.\nSet/get frequency, pulse lenght and bugger size.\nDuring radio operation device may not respond.");
    lv_obj_set_scrollbar_mode(text_area_replay, LV_SCROLLBAR_MODE_OFF); 
    lv_textarea_set_cursor_click_pos(text_area_replay, false);


    containerHelper.createContainer(&button_container_RCSwitchMethod1_, ReplayScreen_, LV_FLEX_FLOW_ROW, 35, 240);


    lv_obj_t *listenButton = ButtonHelper::createButton(button_container_RCSwitchMethod1_, "Listen");
    lv_obj_add_event_cb(listenButton, EVENTS::btn_event_RAW_REC_run, LV_EVENT_CLICKED, NULL); 

    lv_obj_t *saveButton = ButtonHelper::createButton(button_container_RCSwitchMethod1_, "Save");
    lv_obj_add_event_cb(saveButton, EVENTS::save_RF_to_sd_event, LV_EVENT_CLICKED, NULL); 


    containerHelper.createContainer(&button_container_RCSwitchMethod2_, ReplayScreen_, LV_FLEX_FLOW_ROW, 35, 240);


    lv_obj_t *playButton = ButtonHelper::createButton(button_container_RCSwitchMethod2_, "Play");
    lv_obj_t *exitButton = ButtonHelper::createButton(button_container_RCSwitchMethod2_, "Exit");


    lv_obj_add_event_cb(playButton, EVENTS::sendCapturedEvent, LV_EVENT_CLICKED, NULL); 
    lv_obj_add_event_cb(exitButton, EVENTS::exitReplayEvent, LV_EVENT_CLICKED, NULL); 

}

void ScreenManager::createSourAppleScreen() {
    ContainerHelper containerHelper;

    SourAppleScreen_ = lv_obj_create(NULL);

    lv_scr_load(SourAppleScreen_);
    lv_obj_set_flex_flow(SourAppleScreen_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(SourAppleScreen_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * topLabel_container_;
    containerHelper.createContainer(&topLabel_container_, SourAppleScreen_, LV_FLEX_FLOW_ROW, 35, 240);
    lv_obj_set_style_border_width(topLabel_container_, 0, LV_PART_MAIN);


    // kb_qwert_ = KeyboardHelper::createKeyboard(SourAppleScreen_, LV_KEYBOARD_MODE_TEXT_LOWER);
    // kb_freq_ = KeyboardHelper::createKeyboard(SourAppleScreen_, LV_KEYBOARD_MODE_NUMBER);
    // lv_keyboard_set_textarea(kb_freq_, freqInput_);


    // containerHelper.fillTopContainer(topLabel_container_, "Mhz:  ", TEXT_AREA, &freqInput_, "433.92", "433.92", 10, NULL, NULL);
    // lv_obj_set_size(freqInput_, 70, 30);                   
    // lv_obj_add_event_cb(freqInput_, EVENTS::ta_freq_event_cb, LV_EVENT_ALL, kb_freq_);


    // containerHelper.createContainer(&secondLabel_container_, SourAppleScreen_, LV_FLEX_FLOW_ROW, 35, 240);
    // lv_obj_set_style_border_width(secondLabel_container_, 0, LV_PART_MAIN);

    // lv_obj_t * C1101preset_dropdown_ = lv_dropdown_create(secondLabel_container_);
    // lv_dropdown_set_options(C1101preset_dropdown_, "AM650\n"
    //                             "AM270\n"
    //                             "FM238\n"
    //                             "FM476\n"
    //                             "FM95\n"
    //                             "FM15k\n"
    //                             "PAGER\n"
    //                             "HND1\n"
    //                             "HND2\n"
    //                             );

    // lv_obj_add_event_cb(C1101preset_dropdown_, EVENTS::ta_preset_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // buttonSettings_ = lv_btn_create(secondLabel_container_);
    // lv_obj_set_size(buttonSettings_, 90, 30);
    // // Create label inside the button, making the button parent of the label
    // lv_obj_t * labelButton = lv_label_create(buttonSettings_); 
    // lv_label_set_text(labelButton, "Settings");  // Set label text as the placeholder
    // lv_obj_center(labelButton);  // Center the label inside the butto
    // lv_obj_clear_flag(buttonSettings_, LV_OBJ_FLAG_SCROLLABLE); // Double ensure
    // lv_obj_add_event_cb(buttonSettings_, EVENTS::createRFSettingsScreen, LV_EVENT_CLICKED, NULL);

    // Create main text area
    text_area_SourApple = lv_textarea_create(SourAppleScreen_);
    lv_obj_set_size(text_area_SourApple, 240, 140);
    lv_obj_align(text_area_SourApple, LV_ALIGN_CENTER, 0, -20);
    lv_textarea_set_text(text_area_SourApple, "Sour Apple\nWill spam BLE devices\nMay cause crash of Apple devices");
    lv_obj_set_scrollbar_mode(text_area_SourApple, LV_SCROLLBAR_MODE_OFF); 
    lv_textarea_set_cursor_click_pos(text_area_SourApple, false);

    lv_obj_t *buttonContainer;
    containerHelper.createContainer(&buttonContainer, SourAppleScreen_, LV_FLEX_FLOW_ROW, 35, 240);


    lv_obj_t *listenButton = ButtonHelper::createButton(buttonContainer, "Start");
    lv_obj_add_event_cb(listenButton, EVENTS::btn_event_SourApple_Start, LV_EVENT_CLICKED, NULL); 

    lv_obj_t *saveButton = ButtonHelper::createButton(buttonContainer, "Stop");
    lv_obj_add_event_cb(saveButton, EVENTS::btn_event_SourApple_Stop, LV_EVENT_CLICKED, NULL); 


    // containerHelper.createContainer(&button_container_RCSwitchMethod2_, SourAppleScreen_, LV_FLEX_FLOW_ROW, 35, 240);


    // lv_obj_t *playButton = ButtonHelper::createButton(button_container_RCSwitchMethod2_, "Play");
    // lv_obj_t *exitButton = ButtonHelper::createButton(button_container_RCSwitchMethod2_, "Exit");


    // lv_obj_add_event_cb(playButton, EVENTS::sendCapturedEvent, LV_EVENT_CLICKED, NULL); 
    // lv_obj_add_event_cb(exitButton, EVENTS::exitReplayEvent, LV_EVENT_CLICKED, NULL); 

}



void ScreenManager::createmainMenu()
{
    lv_obj_t *mainMenu = lv_obj_create(NULL);                                            // Create a new screen
    lv_scr_load(mainMenu);                                                               // Load the new screen, make it active
    lv_obj_t *btn_subGhz_main = lv_btn_create(mainMenu);                                 /*Add a button the current screen*/
    lv_obj_set_pos(btn_subGhz_main, 25, 10);                                             /*Set its position*/
    lv_obj_set_size(btn_subGhz_main, 150, 50);                                           /*Set its size*/
    lv_obj_add_event_cb(btn_subGhz_main, EVENTS::btn_event_subGhzTools, LV_EVENT_CLICKED, NULL); /*Assign a callback to the button*/

    lv_obj_t *label_subGhz_main = lv_label_create(btn_subGhz_main); /*Add a label to the button*/
    lv_label_set_text(label_subGhz_main, "RF SubGhz Tools");        /*Set the labels text*/
    lv_obj_center(label_subGhz_main);


    lv_obj_t *btn_BT_main = lv_btn_create(mainMenu);                                 /*Add a button the current screen*/
    lv_obj_set_pos(btn_BT_main, 25, 70);                                             /*Set its position*/
    lv_obj_set_size(btn_BT_main, 150, 50);                                           /*Set its size*/
    lv_obj_add_event_cb(btn_BT_main, EVENTS::btn_event_BTTools, LV_EVENT_CLICKED, NULL); /*Assign a callback to the button*/

    lv_obj_t *label_BT_main = lv_label_create(btn_BT_main); /*Add a label to the button*/
    lv_label_set_text(label_BT_main, "BlueTooth");        /*Set the labels text*/
    lv_obj_center(label_BT_main);


}

void ScreenManager::createBTMenu()
{
    lv_obj_t *BTMenu = lv_obj_create(NULL);                                            // Create a new screen
    lv_scr_load(BTMenu);                                                               // Load the new screen, make it active
    lv_obj_t *btn_BT_main = lv_btn_create(BTMenu);                                 /*Add a button the current screen*/
    lv_obj_set_pos(btn_BT_main, 25, 10);                                             /*Set its position*/
    lv_obj_set_size(btn_BT_main, 150, 50);                                           /*Set its size*/
    lv_obj_add_event_cb(btn_BT_main, EVENTS::btn_event_SourApple, LV_EVENT_CLICKED, NULL); /*Assign a callback to the button*/

    lv_obj_t *label_SourApple_main = lv_label_create(btn_BT_main); /*Add a label to the button*/
    lv_label_set_text(label_SourApple_main, "Sour Apple");        /*Set the labels text*/
    lv_obj_center(label_SourApple_main);




       lv_obj_t *btn_c1101Others_menu = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(btn_c1101Others_menu, 25, 250);
    lv_obj_set_size(btn_c1101Others_menu, 200, 50);

    lv_obj_t *label_c1101Others_menu = lv_label_create(btn_c1101Others_menu);
    lv_label_set_text(label_c1101Others_menu, "Back");
    lv_obj_center(label_c1101Others_menu);
    lv_obj_add_event_cb(btn_c1101Others_menu, EVENTS::btn_event_mainMenu_run, LV_EVENT_CLICKED, NULL);

}

void ScreenManager::createRFMenu()
{
    lv_obj_t *rfMenu = lv_obj_create(NULL); // Create a new screen
    lv_scr_load(rfMenu);                    // Load the new screen, make it active

    lv_obj_t *btn_playZero_menu = lv_btn_create(rfMenu);
    lv_obj_set_pos(btn_playZero_menu, 25, 10);
    lv_obj_set_size(btn_playZero_menu, 200, 50);
    lv_obj_add_event_cb(btn_playZero_menu, EVENTS::btn_event_playZero_run, LV_EVENT_ALL, NULL);

    lv_obj_t *label_playZero_menu = lv_label_create(btn_playZero_menu);
    lv_label_set_text(label_playZero_menu, "Transmit saved codes");
    lv_obj_center(label_playZero_menu);

    lv_obj_t *btn_teslaCharger_menu = lv_btn_create(rfMenu);
    lv_obj_set_pos(btn_teslaCharger_menu, 25, 70);
    lv_obj_set_size(btn_teslaCharger_menu, 200, 50);
    lv_obj_add_event_cb(btn_teslaCharger_menu, EVENTS::btn_event_teslaCharger_run, LV_EVENT_ALL, NULL);

    lv_obj_t *label_teslaCharger_menu = lv_label_create(btn_teslaCharger_menu);
    lv_label_set_text(label_teslaCharger_menu, "Transmit tesla charger code");
    lv_obj_center(label_teslaCharger_menu);

    lv_obj_t *btn_c1101Alanalyzer_menu = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(btn_c1101Alanalyzer_menu, 25, 130);
    lv_obj_set_size(btn_c1101Alanalyzer_menu, 200, 50);
    lv_obj_add_event_cb(btn_c1101Alanalyzer_menu, EVENTS::btn_event_Replay_run, LV_EVENT_ALL, NULL);

    lv_obj_t *label_c1101Alanalyzer_menu = lv_label_create(btn_c1101Alanalyzer_menu);
    lv_label_set_text(label_c1101Alanalyzer_menu, "rec/play");
    lv_obj_center(label_c1101Alanalyzer_menu);


    lv_obj_t *btn_c1101Others_menu = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(btn_c1101Others_menu, 25, 250);
    lv_obj_set_size(btn_c1101Others_menu, 200, 50);

    lv_obj_t *label_c1101Others_menu = lv_label_create(btn_c1101Others_menu);
    lv_label_set_text(label_c1101Others_menu, "Back");
    lv_obj_center(label_c1101Others_menu);
    lv_obj_add_event_cb(btn_c1101Others_menu, EVENTS::btn_event_mainMenu_run, LV_EVENT_CLICKED, NULL);

}

void ScreenManager::createRFSettingsScreen()
{
    ContainerHelper containerHelper;



    const char *SYNC_STRINGS[] = {
        "No sync",
        "16 bit",
        "16/16 bit",
        "30/32 bit",
        "no sync - carrier-sense +",
        "15/16 - carrier-sense +",
        "16/16 - carrier-sense +",
        "30/32 - carrier-sense +"};

    const char *PTK_STRINGS[] = {
        "Normal",
        "Sync Serial",
        "Send random data",
        "Async Serial"};

    lv_obj_t *RFSettingsScreen = lv_obj_create(NULL);

    lv_scr_load(RFSettingsScreen);
    lv_obj_set_flex_flow(RFSettingsScreen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(RFSettingsScreen, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Create top label container
    containerHelper.createContainer(&C1101SYNC_container_, RFSettingsScreen, LV_FLEX_FLOW_ROW, 35, 240);
    containerHelper.createContainer(&C1101PTK_container_, RFSettingsScreen, LV_FLEX_FLOW_ROW, 35, 240);
    containerHelper.createContainer(&button_container_settings, RFSettingsScreen, LV_FLEX_FLOW_ROW, 35, 240);

    containerHelper.fillTopContainer(C1101SYNC_container_, "Sync:", DROPDOWN, &C1101SYNC_dropdown_, NULL, NULL, 12, NULL, EVENTS::ta_filename_event_cb, SYNC_STRINGS, 8);
    containerHelper.fillTopContainer(C1101PTK_container_, "PTK:", DROPDOWN, &C1101PTK_dropdown_, NULL, NULL, 12, NULL, EVENTS::ta_filename_event_cb, PTK_STRINGS, 4);
    lv_obj_set_size(C1101SYNC_dropdown_, 165, 30);
    lv_obj_set_size(C1101PTK_dropdown_, 165, 30);

    lv_obj_t *SaveSetting = ButtonHelper::createButton(button_container_settings, "Save");
    lv_obj_t *CancelSettings = ButtonHelper::createButton(button_container_settings, "Cancel");

    lv_obj_add_event_cb(SaveSetting, EVENTS::saveRFSettingEvent, LV_EVENT_CLICKED, NULL); // Assign event callback for Play if needed
    lv_obj_add_event_cb(CancelSettings, EVENTS::cancelRFSettingEvent, LV_EVENT_CLICKED, NULL); // Assign event callback for Exit if needed
}

void ScreenManager::createSubPlayerScreen() {
Serial.println("Initializing playZeroScreen...");
    
    SDInit();
    
    // Initialize dynamic memory
    current_dir = new char[MAX_PATH_LENGTH];
    strcpy(current_dir, "/");
    Serial.print("Initialized current_dir: ");
    Serial.println(current_dir);
    
    selected_file = new char[MAX_PATH_LENGTH];
    Serial.println("Initialized selected_file.");

    file_list = nullptr;  // Initially, no files are allocated
    Serial.println("file_list set to nullptr.");
    
    // Store the current screen before switching
    previous_screen = lv_scr_act();

    // Create a new screen
    lv_obj_t* new_screen = lv_obj_create(NULL);  // Create a new screen
    lv_scr_load(new_screen);  // Load the new screen to clear the previous screen

    createFileBrowser(new_screen);  // Create the file browser on the new screen
    Serial.println("Created file browser on new screen.");
    
    ScreenManager::createFileBrowser(new_screen);
}

void ScreenManager::createFileBrowser(lv_obj_t* parent) {
    
       Serial.println("Setting up file browser...");

    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    ScreenManager::updateFileList(current_dir);  // Update the list with files and folders

    // Label to show selected file
    selected_label = lv_label_create(parent);
    lv_label_set_text(selected_label, "No file selected");
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN); // Arrange children in a row
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); // Align buttons in the container

    //  // Create a horizontal container for the buttons
    lv_obj_t* button_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(button_container, LV_PCT(100), 50); // Set container width to 100% of the parent and height to 50px
    lv_obj_set_flex_flow(button_container, LV_FLEX_FLOW_ROW); // Arrange children in a row
    lv_obj_set_flex_align(button_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); // Align buttons in the container
    lv_obj_clear_flag(button_container, LV_OBJ_FLAG_SCROLLABLE); // Double ensure


 // Button to load the selected file
    lv_obj_t* load_btn = lv_btn_create(button_container);
    lv_obj_add_event_cb(load_btn, EVENTS::load_btn_event_cb_sub, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label = lv_label_create(load_btn);
    lv_label_set_text(label, "Load File");

    // Back button for navigating to the parent directory
    lv_obj_t* back_btn = lv_btn_create(button_container);
    lv_obj_add_event_cb(back_btn, EVENTS::back_btn_event_cb_sub, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, LV_SYMBOL_LEFT "Back");

    Serial.println("File browser setup complete.");
}


void ScreenManager::updateFileList(const char* directory) {
    Serial.print("Updating file list for directory: ");
    Serial.println(directory);

    if (list == NULL) {
        list = lv_list_create(lv_scr_act());  // Create list on the current active screen
        lv_obj_set_size(list, 240, 240);
        Serial.println("Created new list object.");
    } else {
        lv_obj_clean(list);  // Clear the existing buttons
        Serial.println("Cleared existing list.");
    }

    // Get the list of files and folders
    file_count = ScreenManager::getFilteredFileList(directory);
    Serial.print("File count: ");
    Serial.println(file_count);

    // Populate the list with file names
    if (file_count > 0) {
        for (int i = 0; i < file_count; i++) {
            lv_obj_t* btn = lv_list_add_btn(list, 
                            file_list[i][strlen(file_list[i]) - 1] == '/' ? LV_SYMBOL_DIRECTORY : LV_SYMBOL_FILE, 
                            file_list[i]);
            lv_obj_set_user_data(btn, (void*)i);  // Store the file index in user data
            lv_obj_add_event_cb(btn, EVENTS::file_btn_event_cb_sub, LV_EVENT_CLICKED, file_list[i]);
            Serial.print("Added button for file: ");
            Serial.println(file_list[i]);
        }
    } else {
        Serial.println("No files or folders found.");
    }
}


int ScreenManager::getFilteredFileList(const char* directory) {
    Serial.print("Opening directory: ");
    Serial.println(directory);

    File dir = SD.open(directory);
    if (!dir) {
        Serial.print("Failed to open directory: ");
        Serial.println(directory);
        return 0;
    }

    int count = 0;
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) {
            break; 
        }
        count++;
        entry.close();
    }
    Serial.print("Number of entries found: ");
    Serial.println(count);

    // Allocate memory for file_list after counting files
    file_list = new char*[count];

    dir.rewindDirectory();  // Reset directory reading to the start

    count = 0;
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) {
            break;
        }

        int name_length = strlen(entry.name()) + (entry.isDirectory() ? 2 : 1); // Account for "/" if directory
        file_list[count] = new char[name_length];
        if (entry.isDirectory()) {
            snprintf(file_list[count], name_length, "%s/", entry.name());
        } else {
            snprintf(file_list[count], name_length, "%s", entry.name());
        }
        Serial.print("Added entry to file_list[");
        Serial.print(count);
        Serial.print("]: ");
        Serial.println(file_list[count]);

        count++;
        entry.close();
    }

    dir.close();
    Serial.println("Directory closed.");
    return count;
}



void ScreenManager::useSelectedFile(const char* filepath) {
    SubGHzParser parser;
    parser.loadFile(filepath);
    SubGHzData data = parser.parseContent();
    parser.printParsedData(data);

    SDInit();
    lv_label_set_text(selected_label, "Transmitting");
    String fullPath = String(filepath);
    Serial.print("Using file at path: ");
    Serial.println(fullPath);

    if (SD.exists(fullPath.c_str())) {
        read_sd_card_flipper_file(fullPath.c_str());
        C1101CurrentState = STATE_SEND_FLIPPER;
        delay(1000);
    } else {
        Serial.println("File does not exist.");
        lv_label_set_text(selected_label, "File not found");
        return;  // Exit if file does not exist
    }

        // Signal transmitted, so let's refresh the screen
    lv_label_set_text(selected_label, "Signal transmitted");
}



