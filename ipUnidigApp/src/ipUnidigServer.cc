//ipUnidigServer.cc

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
    July 31, 2001 MLR  Minor change to avoid compiler warning
    April 1, 2003 MLR  Add support for interrupts.  Added 6 parameters to call
                       to initIpUnidig to configure interrupts.  These
                       parameters are all optional, and the previous behavior
                       will be obtained if they are omitted.
    May 27, 2003 MLR   Converted to EPICS R3.14.

    This is the code for input and output servers using the Greensprings 
    ipUnidig series of digital I/O IP modules.

    Separate tasks are started for an input server (reading bits) and an 
    output server (setting and clearing bits).  The reason is that the input
    server "polls" the inputs and sends messages back to a client if any input
    has changed.  The poll rate is 10Hz.  Using the same task for the output
    server would introduce a 0.1 second latency which would not be desireable.
    The name of the input server is <moduleName>In and the name of the output
    server is <moduleName>Out, where <moduleName> is the name of the module
    passed to initIpUnidig.  These servers run as tasks called ipUnidigIn
    and ipUnidigOut.
*/

#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsExport.h>
#include <iocsh.h>

#include "Message.h"
#include "Int32Message.h"
#include "mpfType.h"
#include "ipUnidig.h"

static char outputTaskname[] = "ipUnidigOut";
static char inputTaskname[]  = "ipUnidigIn";

extern "C"
{
#ifdef NODEBUG
#define DEBUG(l,f,v...) ;
#else
#define DEBUG(l,f,v...) { if(l<=IpUnidigServerDebug) printf(f,## v); }
#endif
volatile int IpUnidigServerDebug = 0;
}
epicsExportAddress(int, IpUnidigServerDebug);

class IpUnidigOutputServer {
public:
    IpUnidig *pIpUnidig;
    static void ipUnidigOutputServer(IpUnidigOutputServer *);
    IpUnidigOutputServer(const char *moduleName, IpUnidig *pIpUnidig, 
                         int queueSize);
private:
    MessageServer *pMessageServer;
    char *serverName;
};

class IpUnidigInputServer {
public:
    static void ipUnidigInputServer(IpUnidigInputServer *);
    IpUnidigInputServer(const char *serverName, IpUnidig *pIpUnidig,
                        int queueSize, int msecPoll, int biMask);
private:
    static void callBack(void *v, unsigned int bits);
    IpUnidig *pIpUnidig;
    MessageServer *pMessageServer;
    char *serverName;
    Int32Message *preceive;
    UINT32 bits;
    UINT32 oldBits;
    bool valueChange;
    epicsEventId eventID;
    double pollTime;
};


extern "C" int initIpUnidig(
    const char *serverName, ushort_t carrier, ushort_t slot,
    int queueSize, int msecPoll, 
    int intVec, int risingMask, int fallingMask, int biMask, int maxClients)
{
    IpUnidig *pIpUnidig = IpUnidig::init(serverName, carrier, slot,
                          intVec, risingMask, fallingMask, maxClients);
    if(!pIpUnidig) return(0);

    IpUnidigInputServer *pIpUnidigInputServer = 
            new IpUnidigInputServer(serverName, pIpUnidig, queueSize,
            msecPoll, biMask);
    epicsThreadId inputThreadId = epicsThreadCreate(inputTaskname,
                             epicsThreadPriorityMedium, 10000,
                             (EPICSTHREADFUNC)IpUnidigInputServer::ipUnidigInputServer,
                             (void*) pIpUnidigInputServer);
    if(inputThreadId == NULL)
        errlogPrintf("%s IpUnidig Input Server ThreadCreate Failure\n",
            serverName);
    
    IpUnidigOutputServer *pIpUnidigOutputServer = 
            new IpUnidigOutputServer(serverName, pIpUnidig, queueSize);
    epicsThreadId outputThreadId = epicsThreadCreate(outputTaskname,
                             epicsThreadPriorityMedium, 10000,
                             (EPICSTHREADFUNC)IpUnidigOutputServer::ipUnidigOutputServer,
                             (void*) pIpUnidigOutputServer);
    if(outputThreadId == NULL)
        errlogPrintf("%s IpUnidig Output Server ThreadCreate Failure\n",
            serverName);
    
    return(0);
}


// ------------------- Output server ------------------------
IpUnidigOutputServer::IpUnidigOutputServer(const char *moduleName, 
                                           IpUnidig *pIpUnidig, int queueSize)
: pIpUnidig(pIpUnidig)
{
    serverName = (char *) malloc(strlen(moduleName) + 10);
    strcpy(serverName, moduleName);
    strcat(serverName, "Out");
    pMessageServer = new MessageServer(serverName, queueSize);
}

void IpUnidigOutputServer::ipUnidigOutputServer(
                        IpUnidigOutputServer *pIpUnidigOutputServer)
{
    while(true) {
        MessageServer *pMessageServer = pIpUnidigOutputServer->pMessageServer;
        IpUnidig *pIpUnidig = pIpUnidigOutputServer->pIpUnidig;
        pMessageServer->waitForMessage();
        Message *inmsg;
        while((inmsg = pMessageServer->receive())) {
            if(inmsg->getType()!=messageTypeInt32) {
                printf("%s ipUnidigOutputServer got illegal message type %d\n",
                    pMessageServer->getName(), inmsg->getType());
                break;
            }
            Int32Message *pmessage = (Int32Message *)inmsg;
            pmessage->status = 0;
            UINT32 mask = pmessage->address;
            switch (pmessage->cmd) {
               case cmdSetBits:
                  if(pIpUnidig->setBits(mask)) pmessage->status= -1;
                  break;
               case cmdClearBits:
                  if(pIpUnidig->clearBits(mask)) pmessage->status= -1;
                  break;
               case cmdSetDAC:
                  if(pIpUnidig->setDAC(pmessage->value)) pmessage->status= -1;
                  break;
               default:
                  printf("%s ipUnidigOutputServer got illegal command %d\n",
                    pMessageServer->getName(), pmessage->cmd);
                  pmessage->status= -1;
                  break;
            }
            DEBUG(2, "ipUnidigOutputServer, cmd=%d, mask=%x, status=%d\n", 
                      pmessage->cmd, mask, pmessage->status);
            pMessageServer->reply(pmessage);
        }
    }
}


// ------------------- Input server ------------------------

IpUnidigInputServer::IpUnidigInputServer(const char *moduleName, 
                       IpUnidig *pIpUnidig, int queueSize, int msecPoll,
                       int biMask)
: pIpUnidig(pIpUnidig), preceive(0)
{
    serverName = (char *) malloc(strlen(moduleName) + 10);
    strcpy(serverName, moduleName);
    strcat(serverName, "In");
    // Default of 100 msec for backwards compatibility with old version
    if (msecPoll == 0) msecPoll = 100;
    pollTime = msecPoll / 1000.;
    eventID = epicsEventCreate(epicsEventEmpty);
    pIpUnidig->registerCallback(callBack, (void *)this, biMask);
    pMessageServer = new MessageServer(serverName, queueSize);
}


void IpUnidigInputServer::ipUnidigInputServer(IpUnidigInputServer *pServer)
{
   MessageServer *pMessageServer = pServer->pMessageServer;
   IpUnidig *pIpUnidig = pServer->pIpUnidig;
   while(true) {
        pMessageServer->waitForMessage();
        while(true) {
            Message *message = pMessageServer->receive();
            if(message) {
                if(message->getType()!=messageTypeInt32) {
                   printf("%s ipUnidigInputServer got illegal message type %d\n",
                      pServer->serverName, message->getType());
                   delete message;
                   continue;
                }
                if(pServer->preceive) delete pServer->preceive;
                Int32Message *pInt32= (Int32Message *)message;
                if(pInt32->cmd==cmdStopMonitor) { //stop monitoring
                    pServer->preceive = 0;
                } else if(pInt32->cmd!=cmdStartMonitor) {
                    pServer->preceive = 0;
                    printf("%s received illegal cmd %d\n",
                        pServer->serverName,pInt32->cmd);
                } else {
                    if(pServer->preceive)
                        printf("%s received additional client."
                        "Replaces original\n",pServer->serverName);
                    pServer->preceive = pInt32;
                }
                // make sure next client gets first message
                pServer->valueChange = true;
            }
            if(pServer->preceive==0) break;
            int sendStatus = 0;
            // Wait for an interrupt or for the poll time, whichever comes first
            if (epicsEventWaitWithTimeout(pServer->eventID, pServer->pollTime) !=
                epicsEventWaitOK) {
               // The wait timed out, so there was no interrupt, so we need
               // to read the bits.  If there was an interrupt the bits got
               // passed to the callback.
               pIpUnidig->readBits(&pServer->bits);
            } else {
               DEBUG(4, "ipUnidigInputServer, got interrupt\n");
            }   
            DEBUG(5, "UnidigInputServer, bits=%x\n", pServer->bits);
            if (pServer->bits != pServer->oldBits) pServer->valueChange = true;
            if (pServer->valueChange) {
                pServer->valueChange = false;
                pServer->oldBits = pServer->bits;
                Int32Message *psend =
                    (Int32Message *)pMessageServer->
                           allocReplyMessage(pServer->preceive,messageTypeInt32);
                psend->value = pServer->bits;
                psend->status = 0;
                sendStatus = pMessageServer->reply(psend);
            }
            if(sendStatus!=0) {
                delete pServer->preceive;
                pServer->preceive = 0;
            }
        }
    }
    return;
}

void IpUnidigInputServer:: callBack(void *v, unsigned int bits)
{
   // This is the function that gets called by IpUnidig when an interrupt occurs
   IpUnidigInputServer *t = (IpUnidigInputServer *) v;
   t->bits = bits;
   epicsEventSignal(t->eventID);
}

static const iocshArg initArg0 = { "Server name",iocshArgString};
static const iocshArg initArg1 = { "Carrier",iocshArgInt};
static const iocshArg initArg2 = { "Slot",iocshArgInt};
static const iocshArg initArg3 = { "queueSize",iocshArgInt};
static const iocshArg initArg4 = { "msecPoll",iocshArgInt};
static const iocshArg initArg5 = { "intVec",iocshArgInt};
static const iocshArg initArg6 = { "risingMask",iocshArgInt};
static const iocshArg initArg7 = { "fallingMask",iocshArgInt};
static const iocshArg initArg8 = { "biMask",iocshArgInt};
static const iocshArg initArg9 = { "maxClients",iocshArgInt};
static const iocshArg * const initArgs[10] = {&initArg0,
                                              &initArg1,
                                              &initArg2,
                                              &initArg3,
                                              &initArg4,
                                              &initArg5,
                                              &initArg6,
                                              &initArg7,
                                              &initArg8,
                                              &initArg9};
static const iocshFuncDef initFuncDef = {"initIpUnidig",10,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    initIpUnidig(args[0].sval, args[1].ival, args[2].ival,
                 args[3].ival, args[4].ival, args[5].ival,
                 args[6].ival, args[7].ival, args[8].ival,
                 args[9].ival);
}
void ipUnidigServerRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(ipUnidigServerRegister);
