// devIpUnidig.cc

// Original author: Mark Rivers
// Date: 9/19/99
// Current author:  Mark Rivers
// Modifications:
// 25-Nov-2000  MLR  Minor changes to function prototypes to avoid errors from newer
//                   GCC verions
// 23-OCT-2002  MLR  Moved code from completeIO to startIO, and made .db file set
//                   PINI=YES to be compatible with mpf1-9 and above 
// 27-May-2003  MLR  Converted to EPICS R3.14.

// This file provides device support for the following records for the
// Greensprings Unidig digital I/O IP module.
//    longin
//    binary input (bi)
//    binary output (bo)

// For inputs the code is written to be very efficient, and to allow
// "pseudo-interrupt" driven operation.  It works as follows:
//    - The server reads the Unidig inputs at 10 Hz.
//    - If it detects a change in any input it sends a message to the longin
//      device support.  This message contains the new value of the input
//      register.
//    - Binary input records get their bit values from this longin record,
//      and not directly from the hardware.  Changes in the longin record
//      trigger I/O event scanning of the binary input records

#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include <recGbl.h>
#include <alarm.h>

#include "dbAccess.h"
#include "dbDefs.h"
#include "link.h"
#include "epicsPrint.h"
#include "dbCommon.h"
#include "longinRecord.h"
#include "biRecord.h"
#include "boRecord.h"
#include "recSup.h"
#include "epicsExport.h"

#include "Message.h"
#include "Int32Message.h"
#include "DevMpf.h"
#include "ipUnidig.h"

// longin record device support

extern "C"
{
#ifdef NODEBUG
#define LIDEBUG(l,f,v...) ;
#else
#define LIDEBUG(l,f,v...) { if(l<=devLiIpUnidigDebug) printf(f,## v); }
#endif
volatile int devLiIpUnidigDebug = 0;
}
epicsExportAddress(int, devLiIpUnidigDebug);

class LiIpUnidig : public DevMpf
{
public:
        LiIpUnidig(dbCommon*,DBLINK*);

        long startIO(dbCommon* pr);
        long completeIO(dbCommon* pr,Message* m);

        static long dev_init(void*);
};

MAKE_DSET(devLiIpUnidig,LiIpUnidig::dev_init)

long LiIpUnidig::dev_init(void* v)
{
        LIDEBUG(2,"devLiIpUnidig::dev_init(v)\n");
        longinRecord *li = (longinRecord*)v;
        LiIpUnidig* pLiIpUnidig = new LiIpUnidig((dbCommon*)li,&(li->inp));
        pLiIpUnidig->bind();
        return(0);
}

LiIpUnidig::LiIpUnidig(dbCommon* pr,DBLINK* l) : DevMpf(pr,l,true)
   // The "true" flag in the previous line means that I/O event scanning is
   // allowed
{
        LIDEBUG(2,"LiIpUnidig::LiIpUnidig, entry\n");
}

long LiIpUnidig::startIO(dbCommon* pr)
{
        // This routine is typically called once when the record first processes.
        LIDEBUG(5,"LiIpUnidig::startIO, enter, record=%s\n", pr->name);
        Int32Message *psend = new Int32Message;
        psend->cmd = cmdStartMonitor;
        sendReply(psend);
        return(0);
}

long LiIpUnidig::completeIO(dbCommon* pr,Message* m)
{
        longinRecord* li = (longinRecord*)pr;
        Int32Message *pim = (Int32Message*)m;

        int rc = pim->status;
        LIDEBUG(2,"LiIpUnidig::CompleteIO, status=%d\n", rc);

        if(rc==0) {
           li->val = pim->value;
           li->udf=0;
           LIDEBUG(2,"LiIpUnidig::CompleteIO, new value=%x\n",
                                 li->val);
        } else
           recGblSetSevr(pr,READ_ALARM,INVALID_ALARM);
        scanIoRequest(ioscanpvt);
        delete m;
        return rc;
}



/*************** BI SUPPORT ***************/

// The bi records should have their INP field set to VME_IO, with
// C = not used
// S = bit (typically in the range 0-23)
// parm field = @longinRecord

extern "C"
{
#ifdef NODEBUG
#define BIDEBUG(l,f,v...) ;
#else
#define BIDEBUG(l,f,v...) { if(l<=devBiIpUnidigDebug) printf(f,## v); }
#endif
volatile int devBiIpUnidigDebug = 0;
}
epicsExportAddress(int, devBiIpUnidigDebug);

class DevPvt
{
public:
    epicsUInt32 mask;     // mask for this particular bit
    longinRecord *pli;
};

// static long bi_init_record(biRecord*);
static long bi_init_record(void*);
static long bi_ioint_info(int cmd,dbCommon* pr,IOSCANPVT* iopvt);
// static long bi_read(biRecord*);
static long bi_read(void*);

extern "C" {
MPF_DSET devBiIpUnidig =
{5,NULL,NULL,bi_init_record,bi_ioint_info,bi_read,NULL};
};
epicsExportAddress(dset, devBiIpUnidig);


static long bi_init_record(void* v)
{
    biRecord *pr = (biRecord *)v;
    DevPvt* pvt = new DevPvt;
    link *plink = &pr->inp;
    if(plink->type!=VME_IO)
    {
        epicsPrintf("%s bi_init_record Illegal INP field\n",pr->name);
        pr->pact = TRUE;
        return(0);
    }
    vmeio *io = (vmeio*)&(plink->value);
    char *pv = io->parm;
    DBADDR db;
    if(dbNameToAddr(pv,&db)) {
        epicsPrintf("%s bi_init_record Can't locate longin record %s\n",
                              pr->name, pv);
        pr->pact = TRUE;
        return(0);
    }
    longinRecord *pli=(longinRecord*)db.precord;
    int bit = io->signal;
    if(bit < 0 || bit > 23) {
        epicsPrintf("%s bi_init_record Illegal INP Bit field (0-23)=%d\n",
                        pr->name, bit);
        pr->pact = TRUE;
        return(0);
    }
    pvt->mask = 1 << bit;
    pvt->pli = pli;
    pr->dpvt = (void *)pvt;
    BIDEBUG(2,"bi_init_record %s, mask=%d, longin record=%s\n", 
                     pr->name, pvt->mask, pli->name);
    return(0);
}

static long bi_ioint_info(int ,dbCommon* pr,IOSCANPVT* iopvt)
{
    BIDEBUG(2,"bi_ioint_info entry, record=%s\n", pr->name);
    DevPvt* pvt = (DevPvt*)pr->dpvt;
    if(!pvt) return(0);
    LiIpUnidig *pLiIpUnidig = (LiIpUnidig *)pvt->pli->dpvt;
    IOSCANPVT ioscanpvt = pLiIpUnidig->ioscanpvt;
    if (!ioscanpvt) return 0;
    *iopvt = ioscanpvt;
    BIDEBUG(2,"bi_ioint_info, record=%s, success\n", pr->name);
    return 0;
}

static long bi_read(void* v)
{
    biRecord *pr = (biRecord *)v;
    DevPvt* pvt = (DevPvt*)pr->dpvt;
    if(!pvt) return(0);
    pr->rval = (pvt->pli->val & pvt->mask) ? 1 : 0;
    BIDEBUG(2,"bi_read, record=%s, value=%d\n", pr->name, pr->val);
    recGblSetSevr(pr,pvt->pli->stat,pvt->pli->sevr);
    pr->udf=0;
    return 0;
}


/*************** BO SUPPORT ***************/

extern "C"
{
#ifdef NODEBUG
#define BODEBUG(l,f,v...) ;
#else
#define BODEBUG(l,f,v...) { if(l<=devBoIpUnidigDebug) printf(f,## v); }
#endif
volatile int devBoIpUnidigDebug = 0;
}
epicsExportAddress(int, devBoIpUnidigDebug);

class BoIpUnidig : public DevMpf
{
public:
        BoIpUnidig(dbCommon*,DBLINK*);

        long startIO(dbCommon* pr);
        long completeIO(dbCommon* pr,Message* m);

        static long dev_init(void*);
private:
        epicsUInt32 mask;
};

MAKE_DSET(devBoIpUnidig,BoIpUnidig::dev_init)

long BoIpUnidig::dev_init(void* v)
{
        BODEBUG(2,"devBoIpUnidig::dev_init(v)\n");
        boRecord *bo = (boRecord*)v;
        BoIpUnidig* pBoIpUnidig = new BoIpUnidig((dbCommon*)bo,&(bo->out));
        pBoIpUnidig->bind();
        return(MPF_NoConvert);
}

BoIpUnidig::BoIpUnidig(dbCommon* pr,DBLINK* l) : DevMpf(pr,l,false)
{
        vmeio* io = (vmeio*)&(l->value);
        int bit = io->signal;
        if(bit<0 || bit>23)
        {
            epicsPrintf("%s devBoUnidig (init_record) Illegal OUT signal field (0-23)=%d\n",
                pr->name, bit);
        }

        BODEBUG(2,"BoIpUnidig::BoIpUnidig(pr,l)\n");

        mask = 1 << bit;

}

long BoIpUnidig::startIO(dbCommon* pr)
{
        Int32Message *pim = new Int32Message;
        boRecord* bo = (boRecord*)pr;

        pim->address=mask;
        if (bo->val == 0) pim->cmd = cmdClearBits;
        else pim->cmd = cmdSetBits;
        BODEBUG(2,"BoIpUnidig::StartIO(pr), mask=%x, cmd=%d\n", mask, pim->cmd);
        return sendReply(pim);
}

long BoIpUnidig::completeIO(dbCommon* pr,Message* m)
{
        boRecord* bo = (boRecord*)pr;
        Int32Message *pim = (Int32Message*)m;

        BODEBUG(2,"BoIpUnidig::CompleteIO(pr,m)\n");
        long rc=pim->status;

        if(rc==0)
                bo->udf=0;
        else
                recGblSetSevr(pr,READ_ALARM,INVALID_ALARM);
        delete m;
        return rc;
}
