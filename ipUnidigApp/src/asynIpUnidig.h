/* drvIpUnidig.h

    Author: Mark Rivers
    Date: 29-June-2004
  
    Modifications:
    1-Apr-2003  MLR  Changes to support interrupts
   23-Apr-2003  MLR  Added functions for changing the rising and falling masks
   27-May-2003  MLR  Converted to EPICS R3.14
   29-Jun-2004  MLR  Converted from MPF to asyn
*/

#ifndef drvIpUnidigH
#define drvIpUnidigH

#include <asynDriver.h>

typedef void (*ipUnidigCallback)(void *pvt, epicsUInt32 data);

/* Interface supported by ipUnidigAsyn drivers. */
#define asynIpUnidigType "asynIpUnidig"
typedef struct {
    asynStatus (*setBits)          (void *drvPvt, asynUser *pasynUser, 
                                    epicsUInt32 mask);
    asynStatus (*clearBits)        (void *drvPvt, asynUser *pasynUser, 
                                    epicsUInt32 mask);
    asynStatus (*setDAC)           (void *drvPvt, asynUser *pasynUser, 
                                    epicsUInt16 value);
    epicsUInt32 (*getRisingMask)   (void *drvPvt, asynUser *pasynUser);
    void (*setRisingMaskBits)      (void *drvPvt, asynUser *pasynUser, 
                                    epicsUInt32 mask);
    void (*clearRisingMaskBits)    (void *drvPvt, asynUser *pasynUser, 
                                    epicsUInt32 mask);
    epicsUInt32 (*getFallingMask)  (void *drvPvt, asynUser *pasynUser);
    void (*setFallingMaskBits)     (void *drvPvt, asynUser *pasynUser, 
                                    epicsUInt32 mask);
    void (*clearFallingMaskBits)   (void *drvPvt, asynUser *pasynUser, 
                                    epicsUInt32 mask);
    asynStatus (*registerCallback) (void *drvPvt, asynUser *pasynUser, 
                                    ipUnidigCallback callback, epicsUInt32 mask,
                                    void *pvt);
    asynStatus (*cancelCallback)   (void *drvPvt, asynUser *pasynUser, 
                                    ipUnidigCallback callback, epicsUInt32 mask,
                                    void *pvt);
} asynIpUnidig;

#endif /* IpUnidigH */
