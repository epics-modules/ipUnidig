/* devIpUnidig.c 

    Author:  Mark Rivers
    28-June-2004 Converted from MPF to plain driver calls 

   This file provides device support for the following records for the
   Greensprings Unidig digital I/O IP module.
      longin
      binary input (bi)
      binary output (bo)

   For inputs the code is written to be very efficient, and to allow
   true and pseudo interrupt driven operation.  It works as follows:
      - The server reads the Unidig inputs at 10 Hz.
      - If it detects a change in any input it sends a message to the longin
        device support.  This message contains the new value of the input
        register.
      - Binary input records get their bit values from this longin record,
        and not directly from the hardware.  Changes in the longin record
        trigger I/O event scanning of the binary input records

*/

#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

#include <alarm.h>
#include <recGbl.h>
#include <dbAccess.h>
#include <dbDefs.h>
#include <link.h>
#include <cantProceed.h>
#include <dbCommon.h>
#include <dbScan.h>
#include <recSup.h>
#include <devSup.h>
#include <biRecord.h>
#include <boRecord.h>
#include <longinRecord.h>
#include <epicsPrint.h>
#include <epicsExport.h>

#include "drvIpUnidig.h"

typedef struct {
   longinRecord *pli;
   asynUser *pasynUser;
   asynIpUnidig *pasynIpUnidig;
   void *asynIpUnidigPvt;
   int mask;
   int registered;
   IOSCANPVT ioScanPvt;
} devIpUnidigPvt;

typedef struct  {
    long       number;
    DEVSUPFUN  report;
    DEVSUPFUN  init;
    DEVSUPFUN  init_record;
    DEVSUPFUN  get_ioint_info;
    DEVSUPFUN  io;
} dsetIpUnidig;

typedef enum {recTypeBo, recTypeLi} recType;

static long initCommon(dbCommon *pr, DBLINK *plink, recType rt);
static long initBi(biRecord *pbi);
static long readBi(biRecord *pbi);
static long initLi(longinRecord *pli);
static long readLi(longinRecord *pli);
static long initBo(boRecord *pbo);
static long writeBo(boRecord *pbo);
static void callbackLi(void *v, epicsUInt32 value);
static long getIoIntInfoBi(int cmd, biRecord *pbi, IOSCANPVT *iopvt);


dsetIpUnidig devBiIpUnidig = {5, 0, 0, initBi, getIoIntInfoBi, readBi};
dsetIpUnidig devBoIpUnidig = {5, 0, 0, initBo, 0, writeBo};
dsetIpUnidig devLiIpUnidig = {5, 0, 0, initLi, 0, readLi};

epicsExportAddress(dset, devBiIpUnidig);
epicsExportAddress(dset, devBoIpUnidig);
epicsExportAddress(dset, devLiIpUnidig);

static long initBo(boRecord *pbo)
{
    initCommon((dbCommon *)pbo, &pbo->out, recTypeBo);
    return(2); /* Do not convert */
}

static long initLi(longinRecord *pli)
{
    devIpUnidigPvt *pPvt;
    int status;

    status = initCommon((dbCommon *)pli, &pli->inp, recTypeLi);
    if (status) return(-1);
    pPvt = (devIpUnidigPvt *)pli->dpvt;
    scanIoInit(&pPvt->ioScanPvt);
    return(0);
}

static long initBi(biRecord *pbi)
{
    devIpUnidigPvt *pPvt;
    DBLINK *plink = &pbi->inp;
    struct vmeio *io = (struct vmeio*)&(plink->value);
    char *pv = io->parm;
    int bit = io->signal;
    longinRecord *pli;
    DBADDR db;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), "devIpUnidig::initBi");
    pbi->dpvt = pPvt;
    if(dbNameToAddr(pv,&db)) {
        errlogPrintf("%s devIpUnidig::initBi Can't locate longin record %s\n",
                     pbi->name, pv);
        pbi->pact = 1;
        return(-1);
    }
    if(bit < 0 || bit > 23) {
        errlogPrintf("%s devIpUnidig::initBi Illegal INP Bit field (0-23)=%d\n",
                     pbi->name, bit);
        pbi->pact = 1;
        return(-1);
    }
    pPvt->mask = 1 << bit;
    pli=(longinRecord*)db.precord;
    pPvt->pli = pli;
    return(0);
}

static long getIoIntInfoBi(int cmd, biRecord *pbi, IOSCANPVT *iopvt)
{
    devIpUnidigPvt *pPvt = (devIpUnidigPvt *)pbi->dpvt;
    devIpUnidigPvt *pLiPvt = (devIpUnidigPvt *)pPvt->pli->dpvt;

    *iopvt = pLiPvt->ioScanPvt;
    return 0;
}

static long readBi(biRecord *pbi)
{
    devIpUnidigPvt *pPvt = (devIpUnidigPvt *)pbi->dpvt;
    devIpUnidigPvt *pLiPvt = (devIpUnidigPvt *)pPvt->pli->dpvt;

    pbi->rval = (pPvt->pli->val & pPvt->mask) ? 1 : 0;
    asynPrint(pLiPvt->pasynUser, ASYN_TRACEIO_DEVICE,
              "devIpUnidig::readBi %s value=%d\n", pbi->name, pbi->rval);
    recGblSetSevr(pbi, pPvt->pli->stat, pPvt->pli->sevr);
    pbi->udf=0;
    return 0;
}


static long initCommon(dbCommon *pr, DBLINK *plink, recType rt)
{
    devIpUnidigPvt *pPvt;
    struct vmeio *pvmeio;
    char *port;
    int signal;
    asynStatus status;
    asynUser *pasynUser;
    asynInterface *pasynInterface;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), "devIpUnidig::initCommon");
    pr->dpvt = pPvt;

    /* Get the signal from the VME signal */
    pvmeio = (struct vmeio *)&plink->value;
    signal = pvmeio->signal;
    if(signal<0 || signal>23) {
        errlogPrintf("%s devIpUnidig::initCommon: Illegal signal field (0-23)=%d\n",
                    pr->name, signal);
        goto bad;
    }

    pPvt->mask =  1 << signal;

    /* Get the port name from the parm field */
    port = pvmeio->parm;

    /* Create asynUser */
    pasynUser = pasynManager->createAsynUser(0, 0);
    pasynUser->userPvt = pPvt;
    pPvt->pasynUser = pasynUser;

    /* Connect to device */
    status = pasynManager->connectDevice(pasynUser, port, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devIpUnidig::initCommon, connectDevice failed %s\n",
                  pasynUser->errorMessage);
        goto bad;
    }

    /* Get the asynIpUnidig interface */
    pasynInterface = pasynManager->findInterface(pasynUser, asynIpUnidigType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devIpUnidig::initCommon, find asynIpUnidig interface failed\n");
        goto bad;
    }
    pPvt->pasynIpUnidig = (asynIpUnidig *)pasynInterface->pinterface;
    pPvt->asynIpUnidigPvt = pasynInterface->drvPvt;
    return(0);

bad:
    pr->pact=1;
    return(-1);
}

static long writeBo(boRecord *pbo)
{
    devIpUnidigPvt *pPvt = (devIpUnidigPvt *)pbo->dpvt;
    int status;

    if (pbo->val == 0) {
        status = pPvt->pasynIpUnidig->clearBits(pPvt->asynIpUnidigPvt, 
                                                pPvt->pasynUser, pPvt->mask);
        asynPrint(pPvt->pasynUser, ASYN_TRACEIO_DEVICE,
                  "devIpUnidig::writeBo, clearBits=%x\n", pPvt->mask);
    } else {
        status = pPvt->pasynIpUnidig->setBits(pPvt->asynIpUnidigPvt, 
                                              pPvt->pasynUser, pPvt->mask);
        asynPrint(pPvt->pasynUser, ASYN_TRACEIO_DEVICE,
                  "devIpUnidig::writeBo, setBits=%x\n", pPvt->mask);
    }
    if (status == 0)
        pbo->udf=0;
    else
        recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
    return status;
}

static long readLi(longinRecord *pli)
{
    devIpUnidigPvt *pPvt = (devIpUnidigPvt *)pli->dpvt;
    int status;

    /* If already registered, nothing to do */
    if (pPvt->registered) return(0);
    status = pPvt->pasynIpUnidig->registerDevCallback(pPvt->asynIpUnidigPvt, 
                                                      pPvt->pasynUser, 
                                                      callbackLi,
                                                      pli);
    pPvt->registered = 1;
    return status;
}

static void callbackLi(void *v, epicsUInt32 value)
{
    longinRecord *pli = (longinRecord *)v;
    devIpUnidigPvt *pPvt = (devIpUnidigPvt *)pli->dpvt;

    pli->val = value;
    pli->udf = 0;
    asynPrint(pPvt->pasynUser, ASYN_TRACEIO_DEVICE,
              "devIpUnidig::callbackLi, new value=%x\n", value);
    scanOnce(pli);
    scanIoRequest(pPvt->ioScanPvt);
}
