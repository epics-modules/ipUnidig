//ipUnidig.h

/********************COPYRIGHT NOTIFICATION**********************************
This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
****************************************************************************/
/*
    Original Author: Mark Rivers
    Date: 9/19/99
    Current Author: Mark Rivers

*/

#ifndef ipUnidigH
#define ipUnidigH

#define cmdSetBits      0x1
#define cmdClearBits    0x2
#define cmdStartMonitor 0x3
#define cmdStopMonitor  0x4
#define cmdSetDAC       0x5

class IndustryPackModule;
   
class IpUnidig
{
public:
    static IpUnidig * init(
        const char *moduleName, const char *carrierName, const char *siteName);
    IpUnidig(IndustryPackModule* pIPM, unsigned char manufacturer, 
                                       unsigned char model);
    int setBits(UINT32 mask);
    int clearBits(UINT32 mask);
    int readBits(UINT32 *value);
    int setDAC(UINT16 value);
private:
    IpUnidig(IndustryPackModule *pIndustryPackModule);
    IndustryPackModule *pIPM;
    unsigned char manufacturer;
    unsigned char model;
    volatile UINT16 *outputRegisterLow;
    volatile UINT16 *outputRegisterHigh;
    volatile UINT16 *outputEnableLow;
    volatile UINT16 *outputEnableHigh;
    volatile UINT16 *inputRegisterLow;
    volatile UINT16 *inputRegisterHigh;
    volatile UINT16 *controlRegister0;
    volatile UINT16 *controlRegister1;
    volatile UINT16 *DACRegister;
};

#endif //ipUnidigH
