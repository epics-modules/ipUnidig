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

    Modification: July 31, 2001  MLR Minor change to avoid compiler warning

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

#include <vxWorks.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <taskLib.h>
#include <sysLib.h>

#include "Message.h"
#include "Int32Message.h"
#include "mpfType.h"
#include "ipUnidig.h"

static char outputTaskname[] = "ipUnidigOut";
static char inputTaskname[]  = "ipUnidigIn";

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
    IpUnidigInputServer(const char *moduleName, IpUnidig *pIpUnidig,
                        int queueSize);
private:
    IpUnidig *pIpUnidig;
    MessageServer *pMessageServer;
    char *serverName;
    Int32Message *preceive;
    UINT32 oldBits;
    bool valueChange;
};


extern "C" int initIpUnidig(
    const char *moduleName, const char *carrierName, const char *siteName,
    int queueSize)
{
    int taskId;
    
    IpUnidig *pIpUnidig = IpUnidig::init(moduleName,carrierName,siteName);
    if(!pIpUnidig) return(-1);

    IpUnidigInputServer *pIpUnidigInputServer = 
            new IpUnidigInputServer(moduleName, pIpUnidig, queueSize);
    taskId = taskSpawn(inputTaskname,100,VX_FP_TASK,2000,
        (FUNCPTR)IpUnidigInputServer::ipUnidigInputServer,
        (int)pIpUnidigInputServer,0,0,0,0,0,0,0,0,0);
    if(taskId==ERROR) {
        printf("%s ipUnidigInputServer taskSpawn Failure\n", moduleName);
        return(-1);
    }
    
    IpUnidigOutputServer *pIpUnidigOutputServer = 
            new IpUnidigOutputServer(moduleName, pIpUnidig, queueSize);
    taskId = taskSpawn(outputTaskname,100,VX_FP_TASK,2000,
        (FUNCPTR)IpUnidigOutputServer::ipUnidigOutputServer,
        (int)pIpUnidigOutputServer,0,0,0,0,0,0,0,0,0);
    if(taskId==ERROR) {
        printf("%s ipUnidigOutputServer taskSpawn Failure\n", moduleName);
        return(-1);
    }
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
            pMessageServer->reply(pmessage);
        }
    }
}


// ------------------- Input server ------------------------

IpUnidigInputServer::IpUnidigInputServer(const char *moduleName, 
                                           IpUnidig *pIpUnidig, int queueSize)
: pIpUnidig(pIpUnidig), preceive(0)
{
    serverName = (char *) malloc(strlen(moduleName) + 10);
    strcpy(serverName, moduleName);
    strcat(serverName, "In");
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
            UINT32 bits;
            pIpUnidig->readBits(&bits);
            if (bits != pServer->oldBits) pServer->valueChange = true;
            if (pServer->valueChange) {
                pServer->valueChange = false;
                pServer->oldBits = bits;
                Int32Message *psend =
                    (Int32Message *)pMessageServer->
                           allocReplyMessage(pServer->preceive,messageTypeInt32);
                psend->value = bits;
                psend->status = 0;
                sendStatus = pMessageServer->reply(psend);
            }
            if(sendStatus!=0) {
                delete pServer->preceive;
                pServer->preceive = 0;
            }
            taskDelay(sysClkRateGet() / 10);  // Wait 1/10 second
        }
    }
    return;
}

