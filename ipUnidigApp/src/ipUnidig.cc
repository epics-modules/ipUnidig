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
*/

#include <vxWorks.h>
#include <iv.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
 
#include "IndustryPackModule.h"
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


IpUnidig * IpUnidig::init(
    const char *moduleName, const char *carrierName, const char *siteName)
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
    IpUnidig *pIpUnidig = new IpUnidig(pIPM, manufacturer, model);
    return(pIpUnidig);
}

IpUnidig::IpUnidig(IndustryPackModule *pIndustryPackModule, 
                   unsigned char manufacturer, unsigned char model)
: pIPM(pIndustryPackModule), manufacturer(manufacturer), model(model)
{
   // Set up the register pointers.  Set the defaults for most modules
   // Define registers in units of 16-bit words

    UINT16 *base = (UINT16 *) pIPM->getMemBaseIO();
    outputRegisterLow  = base;
    outputRegisterHigh = base + 0x1;
    inputRegisterLow   = base + 0x2;
    inputRegisterHigh  = base + 0x3;
    outputEnableLow    = base + 0x8;
    outputEnableHigh   = base + 0xa;
    controlRegister0   = base + 0xc;
    DACRegister        = base + 0xe;
    
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
}

int IpUnidig::setBits(UINT32 mask)
{
    // For the IP-Unidig differential output models, must enable outputs
    // Don't do this for all outputs in constructor, since then nothing could
    // be an input
    if ((manufacturer == GREENSPRING_ID)  &&
        ((model == UNIDIG_D) || (model == UNIDIG_I_D))) {
         *outputEnableLow  |= (UINT16) mask;
         *outputEnableHigh |= (UINT16) (mask >> 16);
    }
    if (outputRegisterLow)  *outputRegisterLow  |= (UINT16) mask;
    if (outputRegisterHigh) *outputRegisterHigh |= (UINT16) (mask >> 16);
    return(0);
}

int IpUnidig::clearBits(UINT32 mask)
{
    // For the IP-Unidig differential output models, must enable outputs
    // Don't do this for all outputs in constructor, since then nothing could
    // be an input
    if ((manufacturer == GREENSPRING_ID)  &&
        ((model == UNIDIG_D) || (model == UNIDIG_I_D))) {
         *outputEnableLow  |= (UINT16) mask;
         *outputEnableHigh |= (UINT16) (mask >> 16);
    }
    if (outputRegisterLow)  *outputRegisterLow  &= (UINT16) ~mask;
    if (outputRegisterHigh) *outputRegisterHigh &= (UINT16) (~mask >> 16);
    return(0);
}

int IpUnidig::readBits(UINT32 *value)
{
    *value = 0;
    if (inputRegisterLow)  *value  = (UINT32) *inputRegisterLow;
    if (inputRegisterHigh) *value |= (UINT32) (*inputRegisterHigh << 16);
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

