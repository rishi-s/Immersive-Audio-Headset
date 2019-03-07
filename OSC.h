/*
 *  Created on: 14 December, 2018
 *      Author: Rishi Shukla
 ***** Code extended and adapted from OSC example *****
 */

#ifndef OSC_
#define OSC_

#include <Bela.h>
#include <OSCServer.h>
#include <OSCClient.h>
#include <SpatialSceneParams.h> // definition of audio sources and context
#include <SampleStream.h>       // adapted code for streaming/processing audio


extern int gStreams;
extern float gInputVolume[NUM_STREAMS];
extern int gVBAPDefaultAzimuth[NUM_STREAMS];
extern int gVBAPDefaultElevation[NUM_STREAMS];


OSCServer oscServer;
OSCClient oscClient;

int gHRTF=1;		    // global variable to store HRTF set for binauralisation
bool gCalibrate=0;  // global variable to store headtracking calibration state



// parse messages received by OSC Server
int parseMessage(oscpkt::Message msg){

    rt_printf("received message to: %s\n", msg.addressPattern().c_str());

    int intArg;
    float floatArg;
    for(unsigned int stream=0; stream<gStreams; stream++){
      std::string number=to_string(stream);
      if (msg.match("/one/azimuth"+number).popInt32(intArg).isOkNoMoreArgs()){
        rt_printf("received azimuth command %i \n", intArg);
        gVBAPDefaultAzimuth[stream]=intArg;
        rt_printf("Source azimuth is %i \n", gVBAPDefaultAzimuth[stream]);
      }
      else if (msg.match("/one/elevation"+number).popInt32(intArg).isOkNoMoreArgs()){
        rt_printf("received elevation command %i \n", intArg);
        gVBAPDefaultElevation[stream]=intArg;
        rt_printf("Source elevation is %i \n", gVBAPDefaultElevation[stream]);
      }
      else if (msg.match("/one/volume"+number).popFloat(floatArg).isOkNoMoreArgs()){
        rt_printf("received volume command %f \n", floatArg);
        gInputVolume[stream]=floatArg;
        rt_printf("Source volume is %f \n", gInputVolume[stream]);
        return floatArg;
      }
    }
    if (msg.match("/one/calibrate").popInt32(intArg).isOkNoMoreArgs()){
        rt_printf("received calibrate command %i \n", intArg);
        gCalibrate=intArg;
        rt_printf("Calibration is %i \n", gCalibrate);
    }
    else if (msg.match("/one/hrtf").popInt32(intArg).isOkNoMoreArgs()){
        rt_printf("received HRTF command %i \n", intArg);
        gHRTF=intArg;
        rt_printf("HRTF is %i \n", gHRTF);
    }
    return intArg;
}

int localPort = 7562;
int remotePort = 7563;
const char* remoteIp = "192.168.1.7";

bool setupOSC(){
    // setup OSC ports
    oscServer.setup(localPort);
    oscClient.setup(remotePort, remoteIp);
    // send an OSC message to address /osc-setup
    // wait 1 second for a reply on /osc-setup-reply
    bool handshakeReceived = false;
    oscClient.sendMessageNow(oscClient.newMessage.to("/osc-setup").add(123).add(456).end());
    oscServer.receiveMessageNow(1000);
    while (oscServer.messageWaiting()){
        if (oscServer.popMessage().match("/osc-setup-reply")){
            handshakeReceived = true;
        }
    }
    if (handshakeReceived){
        rt_printf("handshake received!\n");
    } else {
        rt_printf("timeout!\n");
    }
	return true;
}

void checkOSC()
{
  // receive OSC messages, parse them, and send back an acknowledgment
  while (oscServer.messageWaiting()){
    parseMessage(oscServer.popMessage());
    oscClient.queueMessage(oscClient.newMessage.to("/test").add(123).add(456).end());
  }
}

#endif /* OSC_ */
