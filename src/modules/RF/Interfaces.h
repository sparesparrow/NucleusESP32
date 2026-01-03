#ifndef C1101_H
#define C1101_H

#include "../../globals.h" 

class INucleusESP32
{
public:
    virtual ~NucleusESP32() = default;
    virtual  decode();
    virtual void send();
};


#endif
