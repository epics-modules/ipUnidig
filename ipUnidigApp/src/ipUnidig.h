//IpUnidig.h

/********************COPYRIGHT NOTIFICATION**********************************
This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
****************************************************************************/
/*
    Original Author: Mark Rivers
    Date: 9/19/99
    Current Author: Mark Rivers
  
    Modifications:
    1-Apr-2003  MLR  Changes to support interrupts

*/

#ifndef IpUnidigH
#define IpUnidigH

#include <fppLib.h>
#define cmdSetBits      0x1
#define cmdClearBits    0x2
#define cmdStartMonitor 0x3
#define cmdStopMonitor  0x4
#define cmdSetDAC       0x5

typedef void (*IpUnidigCallback)(void *pvt, unsigned int data);
typedef struct {
   IpUnidigCallback callback;
   void *pvt;
   int mask;
} IpUnidigClient;

class IndustryPackModule;
   
class IpUnidig
{
public:
    static IpUnidig * init(
        const char *moduleName, const char *carrierName, const char *siteName,
        int intVec, int risingMask, int fallingMask, int maxClients);
    IpUnidig(IndustryPackModule* pIPM, unsigned char manufacturer, 
                                       unsigned char model, 
                                       int intVec, int risingMask, 
                                       int fallingMask, int maxClients);
    int setBits(UINT32 mask);
    int clearBits(UINT32 mask);
    int readBits(UINT32 *value);
    int setDAC(UINT16 value);
    int registerCallback(IpUnidigCallback callback, void *pvt, int mask);
private:
    IpUnidig(IndustryPackModule *pIndustryPackModule);
    static void intFunc(void*); // Interrupt function
    static void rebootCallback(void *);
    IndustryPackModule *pIPM;
    unsigned char manufacturer;
    unsigned char model;
    int supportsInterrupts;
    int rebooting;
    int maxClients;
    int numClients;
    IpUnidigClient *client;
    FP_CONTEXT *pFpContext;
    IpUnidigCallback *clientCallback;
    UINT32 risingMask;
    UINT32 fallingMask;
    UINT32 polarityMask;
    volatile UINT16 *outputRegisterLow;
    volatile UINT16 *outputRegisterHigh;
    volatile UINT16 *outputEnableLow;
    volatile UINT16 *outputEnableHigh;
    volatile UINT16 *inputRegisterLow;
    volatile UINT16 *inputRegisterHigh;
    volatile UINT16 *controlRegister0;
    volatile UINT16 *controlRegister1;
    volatile UINT16 *intVecRegister;
    volatile UINT16 *intEnableRegisterLow;
    volatile UINT16 *intEnableRegisterHigh;
    volatile UINT16 *intPolarityRegisterLow;
    volatile UINT16 *intPolarityRegisterHigh;
    volatile UINT16 *intClearRegisterLow;
    volatile UINT16 *intClearRegisterHigh;
    volatile UINT16 *intPendingRegisterLow;
    volatile UINT16 *intPendingRegisterHigh;
    volatile UINT16 *DACRegister;
};

#endif //IpUnidigH
