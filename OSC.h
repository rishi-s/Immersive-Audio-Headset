/*
 *  Created on: 14 December, 2018
 *      Author: Rishi Shukla
 ***** Code extended and adapted from OSC example *****
 */

#ifndef OSC_
#define OSC_

#include <string>
#include <cstdio>
#include <Bela.h>
#include <OSCServer.h>
#include <OSCClient.h>
#include "SpatialSceneParams.h" // definition of audio sources and context
#include "SampleStream.h"       // adapted code for streaming/processing audio
#include "ABRoutine.h"          // HRTF comparison trial structure

extern int gStreams;
extern int gCurrentState;
extern float gInputVolume[NUM_STREAMS];
extern int gVBAPDefaultAzimuth[NUM_STREAMS];
extern int gVBAPDefaultElevation[NUM_STREAMS];
extern bool gHeardAState;
extern bool gHeardBState;

int gOSCCounter=0;

OSCServer oscServer;
OSCClient oscClient;

int gHRTF=1;		    // global variable to store HRTF set for binauralisation
bool gCalibrate=0;  // global variable to store headtracking calibration state


void sendCurrentStatusOSC();




// parse messages received by OSC Server
int parseMessage(oscpkt::Message msg){

    rt_printf("received message to: %s\n", msg.addressPattern().c_str());

    int intArg;
    float floatArg;

    // Channel-based controls (azimuth, elevation, volume)
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
        gCurrentState=kPlaying;
      }
      else if (msg.match("/one/volume"+number).popFloat(floatArg).isOkNoMoreArgs()){
        rt_printf("received volume command %f \n", floatArg);
        gInputVolume[stream]=floatArg;
        rt_printf("Source volume is %f \n", gInputVolume[stream]);
        return floatArg;
      }
    }

    //Global controls (HRTF and head-tracker calibration)
    if (msg.match("/one/calibrate").popInt32(intArg).isOkNoMoreArgs()){
        rt_printf("received calibrate command %i \n", intArg);
        gCalibrate=intArg;
        rt_printf("Calibration is %i \n", gCalibrate);
    }
    else if (msg.match("/one/hrtf").popInt32(intArg).isOkNoMoreArgs()){
        rt_printf("received HRTF command %i \n", intArg);
        gHRTF=intArg;
        gCurrentState=kStopped;
        rt_printf("HRTF is %i \n", gHRTF);
    }


    // HRTF selection routine input controls
    if(gFixedTrajectory && gComparIndex<22){

      // If "Play A" is tapped, set the correct playback text and playing states
      if (msg.match("/one/playAButton").popFloat(floatArg).isOkNoMoreArgs()){
        if(floatArg>0.0){
          gPlaybackText="";
          gPlayingA=false;
          gPlayingB=false;
          gCurrentState=kStopped;
          gHRTF=gComparMatches[gComparIndex][0];
        }
        if(floatArg==0.0){
          gPlaybackText="PLAYING A";
          gCurrentState=kPlaying;
          gPlayingA=true;
          gPlayingB=false;
        }
      }

      // If "Play B" is tapped, set the correct playback text and playing state
      else if (msg.match("/one/playBButton").popFloat(floatArg).isOkNoMoreArgs()){
        if(floatArg>0.0){
          gPlaybackText="";
          gPlayingA=false;
          gPlayingB=false;
          gCurrentState=kStopped;
          gHRTF=gComparMatches[gComparIndex][1];
        }
        if(floatArg==0.0){
          gPlaybackText="PLAYING B";
          gCurrentState=kPlaying;
          gPlayingA=false;
          gPlayingB=true;
        }
      }

      // If "Toggle A" is tapped, set the correct states and display values ...
      else if (msg.match("/one/chooseAToggle").popFloat(floatArg).isOkNoMoreArgs()){

        // When both A and B have been heard ...
        if (gHeardAState && gHeardBState){
          // if A is not already on, change state, toggle off B and and set text
          if(gChoiceState == kNoneSelected || gChoiceState == kBSelected) {
            gChoiceState=kASelected;
            oscClient.queueMessage(oscClient.newMessage.to("/one/chooseBToggle").\
              add(0.0f).end());
            gSubmitText="SUBMIT A";
          }
          // when A is already on, change state and text
          else if (gChoiceState==kASelected){
            gChoiceState=kNoneSelected;
            gSubmitText=" ";
          }
        }
      }

      // If "Toggle B" is tapped, set the correct submit text and selection state
      else if (msg.match("/one/chooseBToggle").popFloat(floatArg).isOkNoMoreArgs()){

        // When both A and B have been heard ...
        if(gHeardAState && gHeardBState){
          // if B is not already on, change state, toggle off A and change text
          if(gChoiceState == kNoneSelected || gChoiceState == kASelected) {
            gChoiceState=kBSelected;
            gSubmitText="SUBMIT B";
          }
          // if B is already on, change state and text
          else if (gChoiceState==kBSelected) {
            gChoiceState=kNoneSelected;
            gSubmitText=" ";
          }
        }
      }

      // If "Submit" is tapped ...
      else if (msg.match("/one/submitAnsButton").popFloat(floatArg).isOkNoMoreArgs()){
        // and there is an option selected, toggle off the correct selection state
        if(gHeardAState && gHeardBState){
          if(gChoiceState!=kNoneSelected){
            // then reset text displays and increment comparison number.
            gHeardAState=false;
            gHeardBState=false;
            gPlayingA=false;
            gPlayingB=false;
            gHRTFResponses[gComparIndex]=gChoiceState;
            if(++gComparIndex<=21){
              gPlaybackText=" ";
              gSubmitText=" ";
              gProgressText="Choice " + to_string(gComparIndex) + " of 21.";
            }
            else {
              gPlaybackText="END OF TASK";
              gSubmitText=" ";
              gProgressText="END";
              for(int i=0;i<7;i++){
                gHRTFResponseCount[i]=count(gHRTFResponses,gHRTFResponses+7,i);
                rt_printf("Score for HRTF %i is %i", i, gHRTFResponseCount[i]);
              }
              writeHRTFResponses();
            }
            gChoiceState=kNoneSelected;
            gCurrentState=kStopped;
          }
        }
      }
    }
    return intArg;
}

int localPort = 7562;
int remotePort = 7563;
const char* remoteIp = "192.168.1.2";

bool setupOSC(){
    // setup OSC ports
    oscServer.setup(localPort);
    oscClient.setup(remotePort, remoteIp);
    oscClient.sendMessageNow(oscClient.newMessage.to("/one/choiceText").\
      add(gProgressText).end());
    oscClient.queueMessage(oscClient.newMessage.to("/one/playAText").\
      add(std::string("*Play A*")).end());
    oscClient.queueMessage(oscClient.newMessage.to("/one/playBText").\
      add(std::string("*Play B*")).end());
    oscClient.sendMessageNow(oscClient.newMessage.to("/one/playbackText").\
      add(gPlaybackText).end());
    oscClient.sendMessageNow(oscClient.newMessage.to("/one/chooseAToggle").\
      add(0.0f).end());
    oscClient.sendMessageNow(oscClient.newMessage.to("/one/chooseBToggle").\
      add(0.0f).end());
    oscClient.sendMessageNow(oscClient.newMessage.to("/one/submitAnsText").\
      add(gSubmitText).end());
	return true;
}

void checkOSC(){
  // send curret status by OSC
  if(++gOSCCounter>=4410){
    sendCurrentStatusOSC();
    gOSCCounter=0;
  }
  while (oscServer.messageWaiting()){
    parseMessage(oscServer.popMessage());
    sendCurrentStatusOSC();
  }
}

void sendCurrentStatusOSC(){
  oscClient.queueMessage(oscClient.newMessage.to("/one/choiceText").\
    add(gProgressText).end());
  oscClient.queueMessage(oscClient.newMessage.to("/one/playbackText").\
    add(gPlaybackText).end());
  oscClient.queueMessage(oscClient.newMessage.to("/one/submitAnsText").\
    add(gSubmitText).end());
  if(gHeardAState){
    oscClient.queueMessage(oscClient.newMessage.to("/one/playAText").\
      add(std::string("Play A")).end());
  }
  else {
    oscClient.queueMessage(oscClient.newMessage.to("/one/playAText").\
      add(std::string("*Play A*")).end());
  }
  if(gHeardBState){
    oscClient.queueMessage(oscClient.newMessage.to("/one/playBText").\
      add(std::string("Play B")).end());
  }
  else {
    oscClient.queueMessage(oscClient.newMessage.to("/one/playBText").\
      add(std::string("*Play B*")).end());
  }

  if(gChoiceState==kNoneSelected){
    oscClient.queueMessage(oscClient.newMessage.to("/one/chooseAToggle").\
      add(0.0f).end());
    oscClient.queueMessage(oscClient.newMessage.to("/one/chooseBToggle").\
      add(0.0f).end());
  }
  else if (gChoiceState==kASelected) {
    oscClient.queueMessage(oscClient.newMessage.to("/one/chooseAToggle").\
      add(1.0f).end());
    oscClient.queueMessage(oscClient.newMessage.to("/one/chooseBToggle").\
      add(0.0f).end());
  }
  else if (gChoiceState==kBSelected) {
    oscClient.queueMessage(oscClient.newMessage.to("/one/chooseAToggle").\
      add(0.0f).end());
    oscClient.queueMessage(oscClient.newMessage.to("/one/chooseBToggle").\
      add(1.0f).end());
  }

}

#endif /* OSC_ */
