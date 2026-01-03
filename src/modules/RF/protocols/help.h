#ifndef HELP_H
#define HELP_H

enum ManchesterState {
    ManchesterStateMid0,
    ManchesterStateMid1,
    ManchesterStateStart0,
    ManchesterStateStart1
};

enum ManchesterEvent {
    ManchesterEventReset,
    ManchesterEventShortLow,
    ManchesterEventShortHigh,
    ManchesterEventLongLow,
    ManchesterEventLongHigh
};
#endif