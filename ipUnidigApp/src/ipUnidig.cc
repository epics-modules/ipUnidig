//ipUnidig.cc

/********************COPYRIGHT NOTIFICATION**********************************
This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
****************************************************************************/
/*
    Original Author: Mark Rivers
    Date: 9/19/99
    Current Author: Mark Rivers

    This is the driver for the Greenspring IP-Unidig series of digital I/O IP
    modules.  It also supports the Systran DIO316I module.

    Modifications:
    1-Apr-2003  MLR  Added support for interrupts on the IP-Unidig-I series.
                     Added additional arguments to IpUnidig::init and
                     IpUnidig::IpUnidig to support this.
    23-Apr-2003 MLR  Added functions for changing the rising and falling masks
*/

#include <vxWorks.h>
#include <iv.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "epicsPrint.h"

#include "IndustryPackModule.h"
#include "Reboot.h"
#include "ipUnidig.h"

#define GREENSPRING_ID 0xF0
#define SYSTRAN_ID     0x45

#define UNIDIG_E          0x51 //IP-Unidig-E          (24 I/O, LineSafe)
#define UNIDIG            0x61 //IP-Unidig            (24 I/O)
#define UNIDIG_D          0x62 //IP-Unidig-D          (24 I/O, differential)
#define UNIDIG_O_24IO     0x63 //IP-Unidig-O-24IO     (24 I/O, optically iso.)
#define UNIDIG_HV_16I8O   0x64 //IP-Unidig-HV-16I8O   (16I, 8O, high voltage)
#define UNIDIG_E48        0x65 //IP-Unidig-E48        (48 I/O, LineSafe)
#define UNIDIG_I_O_24I    0x66 //IP-Unidig-I-O-24I    (24 I, optical,  interrupts)
#define UNIDIG_I_E        0x67 //IP-Unidig-I-E        (24 I/O, LineSafe, interrupts)
#define UNIDIG_I          0x68 //IP-Unidig-I          (24 I/O, interrupts)
#define UNIDIG_I_D        0x69 //IP-Unidig-I-D        (24 I/O, differential, interrupts)
#define UNIDIG_I_O_24IO   0x6A //IP-Unidig-I-O-24IO   (24 I/O, optical,  interrupts)
#define UNIDIG_I_HV_16I8O 0x6B //IP-Unidig-I-HV-16I8O (16I, 8O, high voltage, ints.)
#define UNIDIG_T          0x6D //IP-Unidig-T          (Timer)
#define UNIDIG_T_D        0x6E //IP-Unidig-T-D        (Timer, differential)
#define UNIDIG_O_12I12O   0x6F //IP-Unidig-O-12I12O   (12I, 12O, optical)
#define UNIDIG_I_O_12I12O 0x70 //IP-Unidig-I-O-12I12O (12I, 12O, optical, interrupts)
#define UNIDIG_P          0x71 //IP-Unidig-P          (16I, 16O)
#define UNIDIG_P_D        0x72 //IP-Unidig-P-D        (16I, 16O, differential)
#define UNIDIG_O_24I      0x73 //IP-Unidig-O-24I      (24I. opt. iso.)
#define UNIDIG_HV_8I16O   0x74 //IP-Unidig-HV-8I16O   (8I, 16O, high voltage)
#define UNIDIG_I_HV_8I16O 0x75 //IP-Unidig-I-HV-8I16O (8I, 16O, high voltage, ints.)

#define SYSTRAN_DIO316I   0x63

extern "C"
{
#ifdef NODEBUG
#define DEBUG(l,f,v...) ;
#else
#define DEBUG(l,f,v...) { if(l<=IpUnidigDebug) epicsPrintf(f,## v); }
#endif
volatile int IpUnidigDebug = 0;
}


IpUnidig * IpUnidig::init(
    const char *moduleName, const char *carrierName, const char *siteName,
    int intVec, int risingMask, int fallingMask, int maxClients)
{
    IndustryPackModule *pIPM = IndustryPackModule::createIndustryPackModule(
        moduleName,carrierName,siteName);
    if(!pIPM) return(0);
    unsigned char manufacturer = pIPM->getManufacturer();
    unsigned char model = pIPM->getModel();
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
             printf("initIpUnidig model 0x%x not supported\n",model);
             return(0);
             break;
       }
       break;

    case SYSTRAN_ID:
       if(model != SYSTRAN_DIO316I) {
          printf("initIpUnidig model 0x%x not Systran DIO316I\n",model);
          return(0);
       }
       break;
       
    default: 
       printf("initIpUnidig manufacturer 0x%x not supported\n", manufacturer);
       return(0);
       break;
    }
    IpUnidig *pIpUnidig = new IpUnidig(pIPM, manufacturer, model, 
                                 intVec, risingMask, fallingMask, maxClients);
    return(pIpUnidig);
}

IpUnidig::IpUnidig(IndustryPackModule *pIndustryPackModule, 
                   unsigned char manufacturer, unsigned char model, 
                   int intVec, int risingMask, int fallingMask, int maxClients)
: pIPM(pIndustryPackModule), manufacturer(manufacturer), model(model),
                             rebooting(0), maxClients(maxClients), numClients(0)
{
   // Set up the register pointers.  Set the defaults for most modules
   // Define registers in units of 16-bit words

    UINT16 *base = (UINT16 *) pIPM->getMemBaseIO();
    outputRegisterLow        = base;
    outputRegisterHigh       = base + 0x1;
    inputRegisterLow         = base + 0x2;
    inputRegisterHigh        = base + 0x3;
    outputEnableLow          = base + 0x8;
    outputEnableHigh         = base + 0x5;
    controlRegister0         = base + 0x6;
    intVecRegister           = base + 0x8;
    intEnableRegisterLow     = base + 0x9;
    intEnableRegisterHigh    = base + 0xa;
    intPolarityRegisterLow   = base + 0xb;
    intPolarityRegisterHigh  = base + 0xc;
    intClearRegisterLow      = base + 0xd;
    intClearRegisterHigh     = base + 0xe;
    intPendingRegisterLow    = base + 0xd;
    intPendingRegisterHigh   = base + 0xe;
    DACRegister              = base + 0xe;
    
    // Set things up for specific models which need to be treated differently
    switch (manufacturer) {
    case GREENSPRING_ID: 
       switch (model) {
       case UNIDIG_O_24IO:
       case UNIDIG_O_12I12O:
       case UNIDIG_I_O_24IO:
       case UNIDIG_I_O_12I12O:
          // Enable outputs
          *controlRegister0 |= 0x4;
          break;
       case UNIDIG_HV_16I8O:
       case UNIDIG_I_HV_16I8O:
          //  These modules don't allow access to outputRegisterLow
          outputRegisterLow = NULL;
          // Set the comparator DAC for 2.5 volts.  Each bit is 15 mV.
          *DACRegister = 2500/15;
          break;
       }
       break;
    case SYSTRAN_ID:
       switch (model) {
       case SYSTRAN_DIO316I:
          // Different register layout
          outputRegisterLow  = base;
          outputRegisterHigh = base + 0x1;
          inputRegisterLow   = base + 0x2;
          inputRegisterHigh  = NULL;
          controlRegister0   = base + 0x3;
          controlRegister1   = base + 0x4;
          // Enable outputs for ports 0-3
          *controlRegister0  |= 0xf;
          // Set direction of ports 0-1 to be output
          *controlRegister1  |= 0x3;
          break;
       }
       break;
    default: 
       printf("IpUnidig:IpUnidig manufacturer 0x%x not supported\n", 
                    manufacturer);
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
          supportsInterrupts = TRUE;
          break;
       default:
          supportsInterrupts = FALSE;
          break;
    }
    if (maxClients < 5) maxClients = 5;
    client = (IpUnidigClient *) calloc(maxClients, sizeof(IpUnidigClient));
    this->risingMask = risingMask;
    this->fallingMask = fallingMask;
    this->polarityMask = risingMask;
    // If the interrupt vector is zero, don't bother with interrupts, 
    // since the user probably didn't pass this
    // parameter to IpUnidig::init().  This is an optional parameter added
    // after initial release.
    if (supportsInterrupts && (intVec !=0)) {
       // Interrupt support
       // Write to the interrupt polarity and enable registers
       if (fppProbe()==OK)
          pFpContext = (FP_CONTEXT *)calloc(1, sizeof(FP_CONTEXT));
       else
          pFpContext = NULL;
       *intVecRegister = intVec;
       if (intConnect(INUM_TO_IVEC(intVec),(VOIDFUNCPTR)intFunc,(int)this)==ERROR){
                printf("ipUnidig intConnect Failure\n");
       }
       *intPolarityRegisterLow  = (UINT16) polarityMask;
       *intPolarityRegisterHigh = (UINT16) (polarityMask >> 16);
       writeIntEnableRegs();
       Reboot::rebootHookAdd(rebootCallback,(void *)this);
       pIPM->intEnable(0);
    }
}


int IpUnidig::setBits(UINT32 mask)
{
    // For the IP-Unidig differential output models, must enable outputs
    // Don't do this for all outputs in constructor, since then nothing could
    // be an input
    if(rebooting) taskSuspend(0);
    if ((manufacturer == GREENSPRING_ID)  &&
        ((model == UNIDIG_D) || (model == UNIDIG_I_D))) {
         *outputEnableLow  |= (UINT16) mask;
         *outputEnableHigh |= (UINT16) (mask >> 16);
    }
    if (outputRegisterLow)  *outputRegisterLow  |= (UINT16) mask;
    if (outputRegisterHigh) *outputRegisterHigh |= (UINT16) (mask >> 16);
    DEBUG(5, "IpUnidig::setBits, mask=%x\n", mask);
    DEBUG(6, "IpUnidig::setBits, outputRegisterLow=%p, outputRegisterHigh=%p\n", 
              outputRegisterLow, outputRegisterHigh);
    return(0);
}

int IpUnidig::clearBits(UINT32 mask)
{
    // For the IP-Unidig differential output models, must enable outputs
    // Don't do this for all outputs in constructor, since then nothing could
    // be an input
    if(rebooting) taskSuspend(0);
    if ((manufacturer == GREENSPRING_ID)  &&
        ((model == UNIDIG_D) || (model == UNIDIG_I_D))) {
         *outputEnableLow  |= (UINT16) mask;
         *outputEnableHigh |= (UINT16) (mask >> 16);
    }
    if (outputRegisterLow)  *outputRegisterLow  &= (UINT16) ~mask;
    if (outputRegisterHigh) *outputRegisterHigh &= (UINT16) (~mask >> 16);
    DEBUG(5, "IpUnidig::clearBits, mask=%x\n", mask);
    DEBUG(6, "IpUnidig::clearBits, outputRegisterLow=%p, outputRegisterHigh=%p\n", 
              outputRegisterLow, outputRegisterHigh);
    return(0);
}

int IpUnidig::readBits(UINT32 *value)
{
    if(rebooting) taskSuspend(0);
    *value = 0;
    if (inputRegisterLow)  *value  = (UINT32) *inputRegisterLow;
    if (inputRegisterHigh) *value |= (UINT32) (*inputRegisterHigh << 16);
    DEBUG(5, "IpUnidig::readBits, *value=%x\n", *value);
    DEBUG(6, "IpUnidig::readBits, inputRegisterLow=%p, inputRegisterHigh=%p\n", 
              inputRegisterLow, inputRegisterHigh);
    return(0);
}

int IpUnidig::setDAC(UINT16 value)
{
    if ((manufacturer == GREENSPRING_ID)  &&
        ((model == UNIDIG_HV_16I8O)   || (model == UNIDIG_HV_8I16O)  ||
         (model == UNIDIG_I_HV_16I8O) || (model == UNIDIG_HV_8I16O))) {
         *DACRegister  = value;
         return(0);
    } else {
       printf("IpUnidig:setDAC not allowed for this model\n");
       return(-1);
    } 
}

UINT32 IpUnidig::getRisingMask()
{
   return(risingMask);
}

void IpUnidig::setRisingMaskBits(UINT32 mask)
{
   risingMask |= mask;
   writeIntEnableRegs();
}

void IpUnidig::clearRisingMaskBits(UINT32 mask)
{
   risingMask &= ~mask;
   writeIntEnableRegs();
}

UINT32 IpUnidig::getFallingMask()
{
   return(fallingMask);
}

void IpUnidig::setFallingMaskBits(UINT32 mask)
{
   fallingMask |= mask;
   writeIntEnableRegs();
}

void IpUnidig::clearFallingMaskBits(UINT32 mask)
{
   fallingMask &= ~mask;
   writeIntEnableRegs();
}

int IpUnidig::registerCallback(IpUnidigCallback callback, void *pvt, int mask)
   {
   IpUnidigClient *pClient;
   if(rebooting) taskSuspend(0);
   DEBUG(1,"IpUnidig::registerCallack, callback=%p, pvt=%p\n", callback, pvt);
   if (numClients >= maxClients) {
      printf("IpUnidig::registerClient: too many clients\n");
      return(-1);
   }
   // Disable interrupts while adding this callback
   int intKey = intLock();
   numClients++;
   pClient = &client[numClients-1];
   pClient->pvt = pvt;
   pClient->callback = callback;
   pClient->mask = mask;
   intUnlock(intKey);
   return(0);
}

void IpUnidig::intFunc(void *v)
{
   IpUnidig *t = (IpUnidig *) v;
   IpUnidigClient *pClient;
   int    i;
   UINT32 inputs=0, pendingLow, pendingHigh, pendingMask, invertMask;

   // Save and restore FP registers so application interrupt functions can do
   // floating point operations.  Skip if no fpp hardware present.
   if (t->pFpContext != NULL) fppSave (t->pFpContext);
   // Clear the interrupts by copying from the interrupt pending register to the
   // interrupt clear register
   *t->intClearRegisterLow = pendingLow = *t->intPendingRegisterLow;
   *t->intClearRegisterHigh = pendingHigh = *t->intPendingRegisterHigh;
   pendingMask = pendingLow | (pendingHigh << 16);
   // Read the current input.  Don't use readBits because that can print debugging.
   if (t->inputRegisterLow)  inputs = (UINT32) *t->inputRegisterLow;
   if (t->inputRegisterHigh) inputs |= (UINT32) (*t->inputRegisterHigh << 16);
   // Call the callback routines which have registered
   for (i = 0; i < t->numClients; i++) {
      pClient = &t->client[i];
      if ((pendingMask & pClient->mask) != 0) {
         pClient->callback(pClient->pvt, inputs);
      }
   }
   // Are there any bits which should generate interrupts on both the rising
   // and falling edge, and which just generated this interrupt?
   invertMask = pendingMask & t->risingMask & t->fallingMask;
   if (invertMask != 0) {
      // We want to invert all bits in the polarityMask that are set in 
      // invertMask. This is done with xor.
      t->polarityMask = t->polarityMask ^ invertMask;
      *t->intPolarityRegisterLow  = (UINT16) t->polarityMask;
      *t->intPolarityRegisterHigh = (UINT16) (t->polarityMask >> 16);
   }
   if (t->pFpContext != NULL) fppRestore(t->pFpContext);
}

void IpUnidig::writeIntEnableRegs()
{
   *intEnableRegisterLow  = (UINT16) (risingMask | fallingMask);
   *intEnableRegisterHigh = (UINT16) ((risingMask | fallingMask) >> 16);
}

void IpUnidig::rebootCallback(void *v)
{
   IpUnidig *pIpUnidig = (IpUnidig *)v;
   *pIpUnidig->intEnableRegisterLow = 0;
   *pIpUnidig->intEnableRegisterHigh = 0;
   pIpUnidig->rebooting = true;
}
