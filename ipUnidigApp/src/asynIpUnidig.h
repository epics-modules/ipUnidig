/* asynIpUnidig.h

    Author: Mark Rivers
    Date: 29-June-2004

    Almost all functions of the driver are now in the generic asynUInt32Digital and 
    asynUInt32DigitalCallback interfaces.  Only setDAC remains here.
*/

#ifndef asynIpUnidigH
#define asynIpUnidigH

#include <asynDriver.h>

/* Interface supported by ipUnidigAsyn drivers. */
#define asynIpUnidigType "asynIpUnidig"
typedef struct {
    asynStatus (*setDAC)           (void *drvPvt, asynUser *pasynUser, 
                                    epicsUInt16 value);
} asynIpUnidig;

#endif /* asynIpUnidigH */
