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
   23-Apr-2003  MLR  Added functions for changing the rising and falling masks
   27-May-2003  MLR  Converted to EPICS R3.13
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

   
class IpUnidig
{
public:
    static IpUnidig * init(
        ushort_t carrier, ushort_t slot,
        int intVec, int risingMask, int fallingMask, int maxClients);
    IpUnidig(ushort_t carrier, ushort_t slot, unsigned char manufacturer, 
             unsigned char model, int intVec, int risingMask, 
             int fallingMask, int maxClients);
    int setBits(epicsUInt32 mask);
    int clearBits(epicsUInt32 mask);
    int readBits(epicsUInt32 *value);
    int setDAC(epicsUInt16 value);
    epicsUInt32 getRisingMask();
    void setRisingMaskBits(epicsUInt32 mask);
    void clearRisingMaskBits(epicsUInt32 mask);
    epicsUInt32 getFallingMask();
    void setFallingMaskBits(epicsUInt32 mask);
    void clearFallingMaskBits(epicsUInt32 mask);
    int registerCallback(IpUnidigCallback callback, void *pvt, int mask);
private:
    IpUnidig(ushort_t carrier, ushort_t slot);
    static void intFunc(void*); // Interrupt function
    static void rebootCallback(void *);
    void writeIntEnableRegs();
    unsigned char manufacturer;
    unsigned char model;
    int supportsInterrupts;
    int rebooting;
    int maxClients;
    int numClients;
    IpUnidigClient *client;
    FP_CONTEXT *pFpContext;
    epicsUInt32 risingMask;
    epicsUInt32 fallingMask;
    epicsUInt32 polarityMask;
    volatile epicsUInt16 *outputRegisterLow;
    volatile epicsUInt16 *outputRegisterHigh;
    volatile epicsUInt16 *outputEnableLow;
    volatile epicsUInt16 *outputEnableHigh;
    volatile epicsUInt16 *inputRegisterLow;
    volatile epicsUInt16 *inputRegisterHigh;
    volatile epicsUInt16 *controlRegister0;
    volatile epicsUInt16 *controlRegister1;
    volatile epicsUInt16 *intVecRegister;
    volatile epicsUInt16 *intEnableRegisterLow;
    volatile epicsUInt16 *intEnableRegisterHigh;
    volatile epicsUInt16 *intPolarityRegisterLow;
    volatile epicsUInt16 *intPolarityRegisterHigh;
    volatile epicsUInt16 *intClearRegisterLow;
    volatile epicsUInt16 *intClearRegisterHigh;
    volatile epicsUInt16 *intPendingRegisterLow;
    volatile epicsUInt16 *intPendingRegisterHigh;
    volatile epicsUInt16 *DACRegister;
};

#endif //IpUnidigH
