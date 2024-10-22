#include "replayScreen.h"
#include "modules/RF/CC1101.h"
#include "globals.h"

ReplayScreen::ReplayScreen()
    : screenManager_(ScreenManager::getInstance()) {
}

ReplayScreen::~ReplayScreen() {
    // Cleanup pokud je potřeba
}

void ReplayScreen::initialize() {
    screenManager_.createReplayScreen();
}
