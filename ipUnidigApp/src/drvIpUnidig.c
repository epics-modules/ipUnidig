/* drvIpUnidig.cc

    Author: Mark Rivers

    This is the driver for the Greenspring IP-Unidig series of digital I/O IP
    modules.  It also supports the Systran DIO316I module.

    Modifications:
    1-Apr-2003  MLR  Added support for interrupts on the IP-Unidig-I series.
                     Added additional arguments to IpUnidig::init and
                     IpUnidig::IpUnidig to support this.
    23-Apr-2003 MLR  Added functions for changing the rising and falling masks
    27-May-2003 MLR  Converted to EPICS R3.14.
    29-Jun-2004 MLR  Coverted from MPF to asyn, and from C++ to C
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <intLib.h>
#include <fppLib.h>
#include <iv.h>
#include <drvIpac.h>

#include <errlog.h>
#include <ellLib.h>
#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsEvent.h>
#include <epicsExport.h>
#include <iocsh.h>

/*#include "Reboot.h" */
#include "drvIpUnidig.h"


#define GREENSPRING_ID 0xF0
#define SYSTRAN_ID     0x45

#define UNIDIG_E          0x51 /*IP-Unidig-E          (24 I/O, LineSafe) */
#define UNIDIG            0x61 /*IP-Unidig            (24 I/O) */
#define UNIDIG_D          0x62 /*IP-Unidig-D          (24 I/O, differential) */
#define UNIDIG_O_24IO     0x63 /*IP-Unidig-O-24IO     (24 I/O, optically iso.) */
#define UNIDIG_HV_16I8O   0x64 /*IP-Unidig-HV-16I8O   (16I, 8O, high voltage) */
#define UNIDIG_E48        0x65 /*IP-Unidig-E48        (48 I/O, LineSafe) */
#define UNIDIG_I_O_24I    0x66 /*IP-Unidig-I-O-24I    (24 I, optical,  interrupts) */
#define UNIDIG_I_E        0x67 /*IP-Unidig-I-E        (24 I/O, LineSafe, interrupts) */
#define UNIDIG_I          0x68 /*IP-Unidig-I          (24 I/O, interrupts) */
#define UNIDIG_I_D        0x69 /*IP-Unidig-I-D        (24 I/O, differential, interrupts) */
#define UNIDIG_I_O_24IO   0x6A /*IP-Unidig-I-O-24IO   (24 I/O, optical,  interrupts) */
#define UNIDIG_I_HV_16I8O 0x6B /*IP-Unidig-I-HV-16I8O (16I, 8O, high voltage, ints.) */
#define UNIDIG_T          0x6D /*IP-Unidig-T          (Timer) */
#define UNIDIG_T_D        0x6E /*IP-Unidig-T-D        (Timer, differential) */
#define UNIDIG_O_12I12O   0x6F /*IP-Unidig-O-12I12O   (12I, 12O, optical) */
#define UNIDIG_I_O_12I12O 0x70 /*IP-Unidig-I-O-12I12O (12I, 12O, optical, interrupts) */
#define UNIDIG_P          0x71 /*IP-Unidig-P          (16I, 16O) */
#define UNIDIG_P_D        0x72 /*IP-Unidig-P-D        (16I, 16O, differential) */
#define UNIDIG_O_24I      0x73 /*IP-Unidig-O-24I      (24I. opt. iso.) */
#define UNIDIG_HV_8I16O   0x74 /*IP-Unidig-HV-8I16O   (8I, 16O, high voltage) */
#define UNIDIG_I_HV_8I16O 0x75 /*IP-Unidig-I-HV-8I16O (8I, 16O, high voltage, ints.) */

#define SYSTRAN_DIO316I   0x63


typedef struct {
   ELLNODE *next;
   ELLNODE *previous;
   asynUser *pasynUser;
   ipUnidigCallback callback;
   void *pvt;
   int mask;
} ipUnidigClient;

typedef struct {
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
} ipUnidigRegisters;

typedef struct {
    unsigned char manufacturer;
    unsigned char model;
    char *portName;
    asynUser *pasynUser;
    epicsUInt16 *baseAddress;
    int supportsInterrupts;
    int rebooting;
    FP_CONTEXT *pFpContext;
    epicsUInt32 risingMask;
    epicsUInt32 fallingMask;
    epicsUInt32 polarityMask;
    ipUnidigRegisters regs;
    ELLLIST intClientList;
    ELLLIST devClientList;
    epicsUInt32 bits;
    epicsUInt32 oldBits;
    int valueChange;
    double pollTime;
    epicsEventId eventID;
    asynInterface common;
    void *commonPvt;
    asynInterface ipUnidig;
    void *ipUnidigPvt;
} drvIpUnidigPvt;

static void report                 (void *drvPvt, FILE *fp, int details);
static asynStatus connect          (void *drvPvt, asynUser *pasynUser);
static asynStatus disconnect       (void *drvPvt, asynUser *pasynUser);
static asynStatus setBits          (void *drvPvt, asynUser *pasynUser,
                                    epicsUInt32 mask);
static asynStatus clearBits        (void *drvPvt, asynUser *pasynUser,
                                    epicsUInt32 mask);
static asynStatus readBits         (void *drvPvt, asynUser *pasynUser,
                                    epicsUInt32 *value);
static asynStatus setDAC           (void *drvPvt, asynUser *pasynUser,
                                    epicsUInt16 value);
static epicsUInt32 getRisingMask   (void *drvPvt, asynUser *pasynUser);
static void setRisingMaskBits      (void *drvPvt, asynUser *pasynUser,
                                    epicsUInt32 mask);
static void clearRisingMaskBits    (void *drvPvt, asynUser *pasynUser,
                                    epicsUInt32 mask);
static epicsUInt32 getFallingMask  (void *drvPvt, asynUser *pasynUser);
static void setFallingMaskBits     (void *drvPvt, asynUser *pasynUser,
                                    epicsUInt32 mask);
static void clearFallingMaskBits   (void *drvPvt, asynUser *pasynUser,
                                    epicsUInt32 mask);
static asynStatus registerIntCallback (void *drvPvt, asynUser *pasynUser,
                                    ipUnidigCallback callback, void *pvt,
                                    int mask);
static asynStatus registerDevCallback (void *drvPvt, asynUser *pasynUser,
                                    ipUnidigCallback callback, void *pvt);
static void pollerThread            (drvIpUnidigPvt *pPvt);
static void pollerCallback          (void *drvPvt, epicsUInt32 newValue);
static void intFunc                 (void*); /* Interrupt function */
static void rebootCallback          (void *);
static void writeIntEnableRegs      ();

/* asynCommon methods */
static const struct asynCommon ipUnidigCommon = {
    report,
    connect,
    disconnect
};

/* asynIpUnidig methods */
static const asynIpUnidig drvIpUnidig = {
    setBits,
    clearBits,
    readBits,
    setDAC,
    getRisingMask,
    setRisingMaskBits,
    clearRisingMaskBits,
    getFallingMask,
    setFallingMaskBits,
    clearFallingMaskBits,
    registerIntCallback,
    registerDevCallback
};


int initIpUnidig(const char *portName, ushort_t carrier, ushort_t slot,
                 int msecPoll, int intVec, int risingMask, 
                 int fallingMask, int biMask)
{
    drvIpUnidigPvt *pPvt;
    ipac_idProm_t *id;
    unsigned char manufacturer;
    unsigned char model;
    epicsUInt16 *base;
    int status;
    int priority=0;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), "initIpUnidig");
    ellInit(&pPvt->intClientList);
    ellInit(&pPvt->devClientList);
    pPvt->portName = epicsStrDup(portName);

    /* Default of 100 msec for backwards compatibility with old version */
    if (msecPoll == 0) msecPoll = 100;
    pPvt->pollTime = msecPoll / 1000.;
    pPvt->eventID = epicsEventCreate(epicsEventEmpty);
    /* registerCallback(callBack, pPvt, biMask); */

    if (ipmCheck(carrier, slot)) {
       errlogPrintf("initIpUnidig: bad carrier or slot\n");
       return(-1);
    }
    id = (ipac_idProm_t *) ipmBaseAddr(carrier, slot, ipac_addrID);
    base = (epicsUInt16 *) ipmBaseAddr(carrier, slot, ipac_addrIO);
    pPvt->baseAddress = base;

    manufacturer = id->manufacturerId & 0xff;
    model = id->modelId & 0xff;
    switch (manufacturer) {
    case GREENSPRING_ID:
       switch (model) {
          case UNIDIG_E:
          case UNIDIG:
          case UNIDIG_D:
          case UNIDIG_O_24IO:
          case UNIDIG_HV_16I8O:
          case UNIDIG_E48:
          case UNIDIG_I_O_24I:
          case UNIDIG_I_E:
          case UNIDIG_I:
          case UNIDIG_I_D:
          case UNIDIG_I_O_24IO:
          case UNIDIG_I_HV_16I8O:
          case UNIDIG_O_12I12O:
          case UNIDIG_I_O_12I12O:
          case UNIDIG_O_24I:
          case UNIDIG_HV_8I16O:
          case UNIDIG_I_HV_8I16O:
             break;
          default:
             errlogPrintf("initIpUnidig model 0x%x not supported\n",model);
             return(-1);
             break;
       }
       break;

    case SYSTRAN_ID:
       if(model != SYSTRAN_DIO316I) {
          errlogPrintf("initIpUnidig model 0x%x not Systran DIO316I\n",model);
          return(-1);
       }
       break;
       
    default: 
       errlogPrintf("initIpUnidig manufacturer 0x%x not supported\n", 
                    manufacturer);
       return(-1);
       break;
    }

    pPvt->manufacturer = manufacturer;
    pPvt->model = model;

    /* Set up the register pointers.  Set the defaults for most modules */
    /* Define registers in units of 16-bit words */
    pPvt->regs.outputRegisterLow        = base;
    pPvt->regs.outputRegisterHigh       = base + 0x1;
    pPvt->regs.inputRegisterLow         = base + 0x2;
    pPvt->regs.inputRegisterHigh        = base + 0x3;
    pPvt->regs.outputEnableLow          = base + 0x8;
    pPvt->regs.outputEnableHigh         = base + 0x5;
    pPvt->regs.controlRegister0         = base + 0x6;
    pPvt->regs.intVecRegister           = base + 0x8;
    pPvt->regs.intEnableRegisterLow     = base + 0x9;
    pPvt->regs.intEnableRegisterHigh    = base + 0xa;
    pPvt->regs.intPolarityRegisterLow   = base + 0xb;
    pPvt->regs.intPolarityRegisterHigh  = base + 0xc;
    pPvt->regs.intClearRegisterLow      = base + 0xd;
    pPvt->regs.intClearRegisterHigh     = base + 0xe;
    pPvt->regs.intPendingRegisterLow    = base + 0xd;
    pPvt->regs.intPendingRegisterHigh   = base + 0xe;
    pPvt->regs.DACRegister              = base + 0xe;
    
    /* Set things up for specific models which need to be treated differently */
    switch (manufacturer) {
    case GREENSPRING_ID: 
       switch (model) {
       case UNIDIG_O_24IO:
       case UNIDIG_O_12I12O:
       case UNIDIG_I_O_24IO:
       case UNIDIG_I_O_12I12O:
          /* Enable outputs */
          *pPvt->regs.controlRegister0 |= 0x4;
          break;
       case UNIDIG_HV_16I8O:
       case UNIDIG_I_HV_16I8O:
          /*  These modules don't allow access to outputRegisterLow */
          pPvt->regs.outputRegisterLow = NULL;
          /* Set the comparator DAC for 2.5 volts.  Each bit is 15 mV. */
          *pPvt->regs.DACRegister = 2500/15;
          break;
       }
       break;
    case SYSTRAN_ID:
       switch (model) {
       case SYSTRAN_DIO316I:
          /* Different register layout */
          pPvt->regs.outputRegisterLow  = base;
          pPvt->regs.outputRegisterHigh = base + 0x1;
          pPvt->regs.inputRegisterLow   = base + 0x2;
          pPvt->regs.inputRegisterHigh  = NULL;
          pPvt->regs.controlRegister0   = base + 0x3;
          pPvt->regs.controlRegister1   = base + 0x4;
          /* Enable outputs for ports 0-3 */
          *pPvt->regs.controlRegister0  |= 0xf;
          /* Set direction of ports 0-1 to be output */
          *pPvt->regs.controlRegister1  |= 0x3;
          break;
       }
       break;
    }
    switch (model) {
       case UNIDIG_I_O_24I:
       case UNIDIG_I_E:
       case UNIDIG_I:
       case UNIDIG_I_D:
       case UNIDIG_I_O_24IO:
       case UNIDIG_I_HV_16I8O:
       case UNIDIG_I_O_12I12O:
       case UNIDIG_I_HV_8I16O:
          pPvt->supportsInterrupts = TRUE;
          break;
       default:
          pPvt->supportsInterrupts = FALSE;
          break;
    }

    /* Start the thread to poll and handle interrupt callbacks to 
     * device support */
    epicsThreadCreate("ipUnidig",
                      epicsThreadPriorityMedium, 10000,
                      (EPICSTHREADFUNC)pollerThread,
                      pPvt);
 
    pPvt->risingMask = risingMask;
    pPvt->fallingMask = fallingMask;
    pPvt->polarityMask = risingMask;
    /* If the interrupt vector is zero, don't bother with interrupts, 
     * since the user probably didn't pass this
     * parameter to IpUnidig::init().  This is an optional parameter added
     * after initial release. */
    if (pPvt->supportsInterrupts && (intVec !=0)) {
       /* Interrupt support */
       /* Write to the interrupt polarity and enable registers */
       if (fppProbe()==OK)
          pPvt->pFpContext = (FP_CONTEXT *)calloc(1, sizeof(FP_CONTEXT));
       else
          pPvt->pFpContext = NULL;
       *pPvt->regs.intVecRegister = intVec;
       if (intConnect(INUM_TO_IVEC(intVec),(VOIDFUNCPTR)intFunc,(int)pPvt)==ERROR){
                errlogPrintf("ipUnidig intConnect Failure\n");
       }
       *pPvt->regs.intPolarityRegisterLow  = (epicsUInt16) pPvt->polarityMask;
       *pPvt->regs.intPolarityRegisterHigh = (epicsUInt16) (pPvt->polarityMask >> 16);
       writeIntEnableRegs(pPvt);

       registerIntCallback(pPvt, pPvt->pasynUser, pollerCallback, pPvt, biMask);
    }

    /* Link with higher level routines */
    pPvt->common.interfaceType = asynCommonType;
    pPvt->common.pinterface  = (void *)&ipUnidigCommon;
    pPvt->common.drvPvt = pPvt;
    pPvt->ipUnidig.interfaceType = asynIpUnidigType;
    pPvt->ipUnidig.pinterface  = (void *)&drvIpUnidig;
    pPvt->ipUnidig.drvPvt = pPvt;
    status = pasynManager->registerPort(pPvt->portName,
                                   0, /*not multiDevice*/
                                   1,
                                   priority,
                                   0);
    if (status != asynSuccess) {
        errlogPrintf("initIpUnidig ERROR: Can't register port\n");
        return -1;
    }
    status = pasynManager->registerInterface(pPvt->portName,&pPvt->common);
    if (status != asynSuccess) {
        errlogPrintf("initIpUnidig ERROR: Can't register common.\n");
        return -1;
    }
    status = pasynManager->registerInterface(pPvt->portName,&pPvt->ipUnidig);
    if (status != asynSuccess) {
        errlogPrintf("initIpUnidig ERROR: Can't register ipUnidig.\n");
        return -1;
    }

    /* Create asynUser for asynTrace */
    pPvt->pasynUser = pasynManager->createAsynUser(0, 0);
    pPvt->pasynUser->userPvt = pPvt;

    /* Connect to device */
    status = pasynManager->connectDevice(pPvt->pasynUser, pPvt->portName, 0);
    if (status != asynSuccess) {
        errlogPrintf("initIpUnidig, connectDevice failed %s\n",
                     pPvt->pasynUser->errorMessage);
        return(-1);
    }


    /* Reboot::rebootHookAdd(rebootCallback,(void *)this); */
    return(0);
}


static asynStatus setBits(void *drvPvt, asynUser *pasynUser, epicsUInt32 mask)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;
    ipUnidigRegisters r = pPvt->regs;

    /* For the IP-Unidig differential output models, must enable outputs
     * Don't do this for all outputs in constructor, since then nothing could
     * be an input */
    if(pPvt->rebooting) taskSuspend(0);
    if ((pPvt->manufacturer == GREENSPRING_ID)  &&
        ((pPvt->model == UNIDIG_D) || (pPvt->model == UNIDIG_I_D))) {
         *r.outputEnableLow  |= (epicsUInt16) mask;
         *r.outputEnableHigh |= (epicsUInt16) (mask >> 16);
    }
    if (r.outputRegisterLow)  *r.outputRegisterLow  |= (epicsUInt16) mask;
    if (r.outputRegisterHigh) *r.outputRegisterHigh |= (epicsUInt16) (mask >> 16);
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvIpUnidig::setBits, mask=%x\n", mask);
    return(asynSuccess);
}

static asynStatus clearBits(void *drvPvt, asynUser *pasynUser, epicsUInt32 mask)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;
    ipUnidigRegisters r = pPvt->regs;

    /* For the IP-Unidig differential output models, must enable outputs
     * Don't do this for all outputs in constructor, since then nothing could
     * be an input */
    if(pPvt->rebooting) taskSuspend(0);
    if ((pPvt->manufacturer == GREENSPRING_ID)  &&
        ((pPvt->model == UNIDIG_D) || (pPvt->model == UNIDIG_I_D))) {
         *r.outputEnableLow  |= (epicsUInt16) mask;
         *r.outputEnableHigh |= (epicsUInt16) (mask >> 16);
    }
    if (r.outputRegisterLow)  *r.outputRegisterLow  &= (epicsUInt16) ~mask;
    if (r.outputRegisterHigh) *r.outputRegisterHigh &= (epicsUInt16) (~mask >> 16);
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvIpUnidig::clearBits, mask=%x\n", mask);
    return(asynSuccess);
}

static asynStatus readBits(void *drvPvt, asynUser *pasynUser, 
                           epicsUInt32 *value)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;
    ipUnidigRegisters r = pPvt->regs;

    if(pPvt->rebooting) taskSuspend(0);
    *value = 0;
    if (r.inputRegisterLow)  *value  = (epicsUInt32) *r.inputRegisterLow;
    if (r.inputRegisterHigh) *value |= (epicsUInt32) (*r.inputRegisterHigh << 16);
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvIpUnidig::readBits, *value=%x\n", *value);
    return(asynSuccess);
}

static asynStatus setDAC(void *drvPvt, asynUser *pasynUser, epicsUInt16 value)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;
    ipUnidigRegisters r = pPvt->regs;

    if ((pPvt->manufacturer == GREENSPRING_ID)  &&
        ((pPvt->model == UNIDIG_HV_16I8O)   || (pPvt->model == UNIDIG_HV_8I16O)  ||
         (pPvt->model == UNIDIG_I_HV_16I8O) || (pPvt->model == UNIDIG_HV_8I16O))) {
         *r.DACRegister  = value;
         return(asynSuccess);
    } else {
       asynPrint(pasynUser, ASYN_TRACE_ERROR,
                 "drvIpUnidig:setDAC not allowed for this model\n");
       return(asynError);
    }
    return(asynSuccess); 
}

static epicsUInt32 getRisingMask(void *drvPvt, asynUser *pasynUser)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;

    return(pPvt->risingMask);
}

static void setRisingMaskBits(void *drvPvt, asynUser *pasynUser, 
                              epicsUInt32 mask)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;

    pPvt->risingMask |= mask;
    writeIntEnableRegs(pPvt);
}

static void clearRisingMaskBits(void *drvPvt, asynUser *pasynUser, 
                                epicsUInt32 mask)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;

    pPvt->risingMask &= ~mask;
    writeIntEnableRegs(pPvt);
}

static epicsUInt32 getFallingMask(void *drvPvt, asynUser *pasynUser)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;

    return(pPvt->fallingMask);
}

static void setFallingMaskBits(void *drvPvt, asynUser *pasynUser, 
                               epicsUInt32 mask)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;

    pPvt->fallingMask |= mask;
    writeIntEnableRegs(pPvt);
}

static void clearFallingMaskBits(void *drvPvt, asynUser *pasynUser, 
                                 epicsUInt32 mask)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;

    pPvt->fallingMask &= ~mask;
    writeIntEnableRegs(pPvt);
}


static asynStatus registerIntCallback (void *drvPvt, asynUser *pasynUser, 
                                       ipUnidigCallback callback, void *pvt, 
                                       int mask)
{
    /* Registers a callback to be called at interrupt level */
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;
    int intKey;
    ipUnidigClient *pClient;

    if(pPvt->rebooting) taskSuspend(0);
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvIpUnidig::registerIntCallack, callback=%p, pvt=%p\n", 
              callback, pvt);
    pClient = callocMustSucceed(1, sizeof(*pClient), 
                                "drvIpUnidig::registerIntCallback");
    pClient->pvt = pvt;
    pClient->callback = callback;
    pClient->pasynUser = pasynUser;
    pClient->mask = mask;
    /* Disable interrupts while adding this callback */
    intKey = intLock();
    ellAdd(&pPvt->intClientList, (ELLNODE *)pClient);
    intUnlock(intKey);
    return(asynSuccess);
}

static void intFunc(void *drvPvt)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;
    ipUnidigRegisters r = pPvt->regs;
    ipUnidigClient *pClient;
    epicsUInt32 inputs=0, pendingLow, pendingHigh, pendingMask, invertMask;

    /* Save and restore FP registers so application interrupt functions can do
     * floating point operations.  Skip if no fpp hardware present. */
    if (pPvt->pFpContext != NULL) fppSave (pPvt->pFpContext);
    /* Clear the interrupts by copying from the interrupt pending register to the
     * interrupt clear register */
    *r.intClearRegisterLow = pendingLow = *r.intPendingRegisterLow;
    *r.intClearRegisterHigh = pendingHigh = *r.intPendingRegisterHigh;
    pendingMask = pendingLow | (pendingHigh << 16);
    /* Read the current input.  Don't use readBits because that can print debugging. */
    if (r.inputRegisterLow)  inputs = (epicsUInt32) *r.inputRegisterLow;
    if (r.inputRegisterHigh) inputs |= (epicsUInt32) (*r.inputRegisterHigh << 16);
    /* Call the callback routines which have registered */
    pClient = (ipUnidigClient *)ellFirst(&pPvt->intClientList);
    while(pClient) {
        if ((pendingMask & pClient->mask) != 0) {
            pClient->callback(pClient->pvt, inputs);
        pClient = (ipUnidigClient *)ellNext(pClient);
        }
    }

    /* Are there any bits which should generate interrupts on both the rising
     * and falling edge, and which just generated this interrupt? */
    invertMask = pendingMask & pPvt->risingMask & pPvt->fallingMask;
    if (invertMask != 0) {
        /* We want to invert all bits in the polarityMask that are set in 
         * invertMask. This is done with xor. */
        pPvt->polarityMask = pPvt->polarityMask ^ invertMask;
        *r.intPolarityRegisterLow  = (epicsUInt16) pPvt->polarityMask;
        *r.intPolarityRegisterHigh = (epicsUInt16) (pPvt->polarityMask >> 16);
    }
    if (pPvt->pFpContext != NULL) fppRestore(pPvt->pFpContext);
}


static asynStatus registerDevCallback (void *drvPvt, asynUser *pasynUser,
                                       ipUnidigCallback callback, void *pvt)
{
    /* Registers a callback to be called at device support level */
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;
    ipUnidigClient *pClient;

    pClient = callocMustSucceed(1, sizeof(*pClient),
                                "drvIpUnidig::registerDevCallback");
    pClient->pvt = pvt;
    pClient->callback = callback;
    pClient->pasynUser = pasynUser;
    ellAdd(&pPvt->devClientList, (ELLNODE *)pClient);
    /* Make sure a callback happens for the initial value */
    pPvt->valueChange = 1;
    epicsEventSignal(pPvt->eventID);
    return(asynSuccess);
}

static void pollerThread(drvIpUnidigPvt *pPvt)
{
    /* This function runs in a separate thread.  It waits for the poll
     * time, or an interrupt, whichever comes first.  If the bits read from
     * the ipUnidig have changed then it does callbacks to all clients that
     * have registered with registerDevCallback */
    ipUnidigClient *pClient;

    while(1) {      
        /*  Wait for an interrupt or for the poll time, whichever comes first */
        if (epicsEventWaitWithTimeout(pPvt->eventID, pPvt->pollTime) !=
                                      epicsEventWaitOK) {
            /* The wait timed out, so there was no interrupt, so we need
             * to read the bits.  If there was an interrupt the bits got
             * passed to the callback. */
            readBits(pPvt, pPvt->pasynUser, &pPvt->bits);
        } else {
            asynPrint(pPvt->pasynUser, ASYN_TRACE_FLOW,
                      "drvIpUnidig::pollerThread, got interrupt\n");
        }
        asynPrint(pPvt->pasynUser, ASYN_TRACEIO_DRIVER,
                  "drvIpUnidig::pollerThread, bits=%x\n", pPvt->bits);

        if (pPvt->bits != pPvt->oldBits) pPvt->valueChange = 1;
        if (pPvt->valueChange) {
            pPvt->valueChange = 0;
            pPvt->oldBits = pPvt->bits;
            /* Call the callback routines which have registered */
            pClient = (ipUnidigClient *)ellFirst(&pPvt->devClientList);
            while(pClient) {
                asynPrint(pPvt->pasynUser, ASYN_TRACE_FLOW,
                          "drvIpUnidig::pollerThread, calling client\n");
                pClient->callback(pClient->pvt, pPvt->bits);
                pClient = (ipUnidigClient *)ellNext(pClient);
            }
        }
    }
}

static void pollerCallback(void *drvPvt, epicsUInt32 newValue)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;

    pPvt->bits = newValue;
    epicsEventSignal(pPvt->eventID);
}


static void writeIntEnableRegs(drvIpUnidigPvt *pPvt)
{
    ipUnidigRegisters r = pPvt->regs;

    *r.intEnableRegisterLow  = (epicsUInt16) (pPvt->risingMask | 
                                              pPvt->fallingMask);
    *r.intEnableRegisterHigh = (epicsUInt16) ((pPvt->risingMask | 
                                               pPvt->fallingMask) >> 16);
}

static void rebootCallback(void *drvPvt)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;
    ipUnidigRegisters r = pPvt->regs;

    *r.intEnableRegisterLow = 0;
    *r.intEnableRegisterHigh = 0;
    pPvt->rebooting = 1;
}


/* asynCommon routines */

/* Report  parameters */
static void report(void *drvPvt, FILE *fp, int details)
{
    drvIpUnidigPvt *pPvt = (drvIpUnidigPvt *)drvPvt;

    assert(pPvt);
    fprintf(fp, "drvIpUnidig %s: connected at base address %p\n",
            pPvt->portName, pPvt->baseAddress);
    if (details >= 1) {
        fprintf(fp, "  risingMask=%x\n", pPvt->risingMask);
        fprintf(fp, "  fallingMask=%x\n", pPvt->fallingMask);
    }
}

/* Connect */
static asynStatus connect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionConnect(pasynUser);
    return(asynSuccess);
}

/* Connect */
static asynStatus disconnect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionDisconnect(pasynUser);
    return(asynSuccess);
}


/* iocsh functions */
static const iocshArg initArg0 = { "Server name",iocshArgString};
static const iocshArg initArg1 = { "Carrier",iocshArgInt};
static const iocshArg initArg2 = { "Slot",iocshArgInt};
static const iocshArg initArg3 = { "msecPoll",iocshArgInt};
static const iocshArg initArg4 = { "intVec",iocshArgInt};
static const iocshArg initArg5 = { "risingMask",iocshArgInt};
static const iocshArg initArg6 = { "fallingMask",iocshArgInt};
static const iocshArg initArg7 = { "biMask",iocshArgInt};
static const iocshArg * const initArgs[8] = {&initArg0,
                                             &initArg1,
                                             &initArg2,
                                             &initArg3,
                                             &initArg4,
                                             &initArg5,
                                             &initArg6,
                                             &initArg7};
static const iocshFuncDef initFuncDef = {"initIpUnidig",8,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    initIpUnidig(args[0].sval, args[1].ival, args[2].ival,
                 args[3].ival, args[4].ival, args[5].ival,
                 args[6].ival, args[7].ival);
}
void ipUnidigRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(ipUnidigRegister);

