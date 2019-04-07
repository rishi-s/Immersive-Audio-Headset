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
extern bool gHeadLocked;
extern float gInputVolume[NUM_STREAMS];
extern int gVBAPDefaultAzimuth[NUM_STREAMS];
extern int gVBAPDefaultElevation[NUM_STREAMS];
extern int gVBAPUpdateAzimuth[NUM_STREAMS];
extern int gVBAPUpdateElevation[NUM_STREAMS];
extern bool gHeardAState;
extern bool gHeardBState;
extern bool gLooping;

int gOSCCounter=0;
float gTimeCounter=0;

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


  // If in test mode and HRTF comparison index is within range
  if(gFixedTrajectory && gComparIndex<22){
    // Look for the following messages:

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
          // then reset state machines and log selection ...
          gHeardAState=false;
          gHeardBState=false;
          gPlayingA=false;
          gPlayingB=false;
          gHRTFResponses[gComparIndex]= \
            gComparMatches[gComparIndex][gChoiceState-1];
          gHRTFResponseTimes[gComparIndex]=gTimeCounter;
          gTimeCounter=0;
          gOSCCounter=0;
          // and if in range, increment comparison number ...
          if(++gComparIndex<=21){
            gPlaybackText=" ";
            gSubmitText=" ";
            gProgressText="Choice " + to_string(gComparIndex) + " of 21.";
          }
          // otherwise close task and write results.
          else {
            gPlaybackText="END OF TASK";
            gSubmitText=" ";
            gProgressText="END";
            gHeadLocked=0;
            for(int i=0;i<7;i++){
              gHRTFResponseCount[i]=count(gHRTFResponses+1,gHRTFResponses+22,i);
              rt_printf("Score for HRTF %i is %i \n", i, gHRTFResponseCount[i]);
            }
            writeHRTFResponses();
            getWinnerAndLoser();
            rt_printf("Winning HRTF is %i \n", gWinningHRTF);
            rt_printf("Losing HRTF is %i \n", gLosingHRTF);
          }
          gChoiceState=kNoneSelected;
          gCurrentState=kStopped;
        }
      }
    }
  }
  // Otherwise, if we have finished the HRTF comparison
  else {
    //Look for the following messages
    // If using toggle calibration, rotate through states
    if (msg.match("/one/calibrate").popFloat(floatArg).isOkNoMoreArgs()){
      if(floatArg>0.0){
        gCalibrate=1;
        if(gCalibrateState==kAhead) {
          gCalibrateState=kAzimuth;
          gCalibrateText="Azimuth calibrated";
        }
        else if(gCalibrateState==kDown){
          gCalibrateState=kElevation;
          gCalibrateText="Elevation calibrated";
        }
      }
      else if(floatArg==0.0){
        gCalibrate=0;
        if(gCalibrateState==kAzimuth) {
          gCalibrateState=kDown;
          gCalibrateText="Look down";
        }
        else if(gCalibrateState==kElevation){
          gCalibrateState=kAhead;
          gCalibrateText="Look ahead";
          gInputVolume[0]=0.0;
          gVBAPDefaultAzimuth[1]=gLocationTrials[0][0];
          gVBAPDefaultElevation[1]=gLocationTrials[0][1];
          gInputVolume[1]=0.5;
          gLooping=true;
          gCurrentState=kPlaying;
        }
      }
    }

    // If a location is submitted
    else if (msg.match("/two/positionSubmit").popFloat(floatArg).isOkNoMoreArgs()){
      // then log head-tracked azi/ele for source against current index...
      if(floatArg>0.0){
        gLocalisationResponses[gLocationIndex][0]=gVBAPUpdateAzimuth[1];
        gLocalisationResponses[gLocationIndex][1]=gVBAPUpdateElevation[1];
        gLocalisationResponseTimes[gLocationIndex]=gTimeCounter;
        gCurrentState=kStopped;
        gTimeCounter=0;
        gOSCCounter=0;
      }

      if(floatArg==0.0){
        // and if in range, increment comparison number ...
        if(++gLocationIndex<42){
          gVBAPDefaultAzimuth[1]=gLocationTrials[gLocationIndex][0];
          gVBAPDefaultElevation[1]=gLocationTrials[gLocationIndex][1];
          gLocationText="Target " + to_string(gLocationIndex-1) + " of 40.";
          gCurrentState=kPlaying;
        }
        // otherwise close task and write results.
        else {
          gLocationText="END";
          writeLocationResponses();
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
    gTimeCounter+=0.1;
  }
  while (oscServer.messageWaiting()){
    parseMessage(oscServer.popMessage());
    sendCurrentStatusOSC();
  }
}

void sendCurrentStatusOSC(){
  oscClient.queueMessage(oscClient.newMessage.to("/two/targetText").\
    add(gLocationText).end());
  oscClient.queueMessage(oscClient.newMessage.to("/one/choiceText").\
    add(gProgressText).end());
  oscClient.queueMessage(oscClient.newMessage.to("/one/playbackText").\
    add(gPlaybackText).end());
  oscClient.queueMessage(oscClient.newMessage.to("/one/submitAnsText").\
    add(gSubmitText).end());
  oscClient.queueMessage(oscClient.newMessage.to("/one/calibrateText").\
    add(gCalibrateText).end());
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
