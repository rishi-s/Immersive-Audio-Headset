/*
 *  Created on: 14 December, 2018
 *      Author: Rishi Shukla
 ***** Code extended and adapted from OSC example *****
 */

#ifndef OSC_
#define OSC_

#include <iostream>
#include <string>
#include <cstdio>
#include <Bela.h>
#include <libraries/OscReceiver/OscReceiver.h>
#include <libraries/OscSender/OscSender.h>
#include <libraries/Pipe/Pipe.h>
#include "spatialisation/SpatialSceneParams.h" // definition of audio sources and context
#include "spatialisation/SampleStream.h"       // adapted code for streaming/processing audio
#include "spatialisation/ABRoutine.h"          // HRTF comparison trial structure


extern int gCurrentState;
extern bool gHeadLocked;
extern float gInputVolume[NUM_STREAMS];
extern int gVBAPDefaultAzimuth[NUM_FIXED_POSITIONS];
extern int gVBAPDefaultElevation[NUM_FIXED_POSITIONS];
extern float gVBAPDefaultVector[NUM_FIXED_POSITIONS][3];
extern float gVBAPActiveVector[NUM_VBAP_TRACKS][3];
extern int gVBAPUpdateAzimuth[NUM_VBAP_TRACKS];
extern int gVBAPUpdateElevation[NUM_VBAP_TRACKS];
extern bool gHeardAState;
extern bool gHeardBState;
extern float gMainVol;
extern bool setupIMU(int sampleRate);
extern void pauseAudioFiles();
extern void startPlayback(int stream);
extern void changeAudioFiles(int oldTrack, int newTrack);

extern int gCurrentTargetSong;
extern int gTargetState;

int gOSCCounter=0;
float gTimeCounter=0;

Pipe oscPipe;

OscReceiver oscServer;
OscSender oscClient;
OscSender oscMonitor;

int gHRTF=0;		    // global variable to store HRTF set for binauralisation
bool gCalibrate=0;  // global variable to store headtracking calibration state
bool gCurrentSceneMode=false;	// global variable to store headtracking calibration state
bool gPreviousSceneMode=false;
bool LastSceneValue=false;
int CurrentSwipeValue[8]={};
int CurrentLocation=0;
int PreviousLocation=0;

bool handshakeReceived;

void sendCurrentStatusOSC();


// monitor messages received by OSC Server
void on_receive(oscpkt::Message* msg, void*)
{
	// we make a copy of the incoming message and we send it down the pipe to the real-time thread
	oscpkt::Message* incomingMsg = new oscpkt::Message(msg);
	oscPipe.writeNonRt(incomingMsg);

	// the real-time thread sends back to us the pointer once it is done with it
	oscpkt::Message* returnedMsg;
	while(oscPipe.readNonRt(returnedMsg) > 0)
	{
		delete returnedMsg;
	}
}


// parse messages received by OSC Server
void parseMessage(oscpkt::Message* msg){

	//OSC message receipt confirmation
  //rt_printf("received message to: %s\n", msg->addressPattern().c_str());

	// variables to store OSC input arguments
  int intArg;
  float floatArg;

	// if there is any activity on the swipe area:
	for(int buttonNo =1; buttonNo<=8; buttonNo++){
		std::string buttonID=to_string(buttonNo);
		if (msg->match("/two/1/"+buttonID).popFloat(floatArg).isOkNoMoreArgs()){
				CurrentSwipeValue[buttonNo-1]=floatArg;			// store button value
				if(floatArg==1.0) CurrentLocation=buttonNo;	// store button number
		}
	}

  // Channel-based controls (azimuth, elevation, volume)
  for(unsigned int stream=0; stream<NUM_SIM_3D_STREAMS; stream++){
    std::string number=to_string(stream);
    if (msg->match("/one/azimuth"+number).popInt32(intArg).isOkNoMoreArgs()){
      rt_printf("received azimuth command %i \n", intArg);
      gVBAPDefaultAzimuth[stream]=intArg;
      rt_printf("Source azimuth is %i \n", gVBAPDefaultAzimuth[stream]);
    }
    else if (msg->match("/one/elevation"+number).popInt32(intArg).isOkNoMoreArgs()){
      rt_printf("received elevation command %i \n", intArg);
      gVBAPDefaultElevation[stream]=intArg;
      rt_printf("Source elevation is %i \n", gVBAPDefaultElevation[stream]);
      gCurrentState=kPlaying;
    }
    else if (msg->match("/one/volume"+number).popFloat(floatArg).isOkNoMoreArgs()){
      rt_printf("received volume command %f \n", floatArg);
      gInputVolume[stream]=floatArg;
      rt_printf("Source volume is %f \n", gInputVolume[stream]);
    }
  }

  //Global controls (HRTF, head-tracker calibration, IMU reinitialisation)
  if (msg->match("/one/calibrate").popInt32(intArg).isOkNoMoreArgs()){
      rt_printf("received calibrate command %i \n", intArg);
      gCalibrate=intArg;
      rt_printf("Calibration is %i \n", gCalibrate);
  }
  else if (msg->match("/one/hrtf").popInt32(intArg).isOkNoMoreArgs()){
      rt_printf("received HRTF command %i \n", intArg);
      gHRTF=intArg;
      gCurrentState=kStopped;
      rt_printf("HRTF is %i \n", gHRTF);
  }
  else if (msg->match("/one/reinitIMU").popFloat(floatArg).isOkNoMoreArgs()){
    if(floatArg>0.0){
      setupIMU(44100);
    }
	}
	else if (msg->match("/two/pause").popFloat(floatArg).isOkNoMoreArgs()){
		if(floatArg==0.0){
			rt_printf("received pause command: %f \n", floatArg);
			pauseAudioFiles();
		}
	}
	else if (msg->match("/two/mainvol").popFloat(floatArg).isOkNoMoreArgs()){
		rt_printf("received main volume change: %f \n", floatArg);
		gMainVol=log10(floatArg/140) / log10(140) *-1;
	}


  // If in test mode and HRTF comparison index is within range
  if(gFixedTrajectory && gComparIndex<22){
    // Look for the following messages:

    // If "Play A" is tapped, set the correct playback text and playing states
    if (msg->match("/one/playAButton").popFloat(floatArg).isOkNoMoreArgs()){
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
				startPlayback(0);
      }
    }

    // If "Play B" is tapped, set the correct playback text and playing state
    else if (msg->match("/one/playBButton").popFloat(floatArg).isOkNoMoreArgs()){
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
				startPlayback(0);
      }
    }

    // If "Toggle A" is tapped, set the correct states and display values ...
    else if (msg->match("/one/chooseAToggle").popFloat(floatArg).isOkNoMoreArgs()){

      // When both A and B have been heard ...
      if (gHeardAState && gHeardBState){
        // if A is not already on, change state, toggle off B and and set text
        if(gChoiceState == kNoneSelected || gChoiceState == kBSelected) {
          gChoiceState=kASelected;
          oscClient.newMessage("/one/chooseBToggle").add(0.0f).send();
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
    else if (msg->match("/one/chooseBToggle").popFloat(floatArg).isOkNoMoreArgs()){

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
    else if (msg->match("/one/submitAnsButton").popFloat(floatArg).isOkNoMoreArgs()){
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
          // then reset counters
          gTimeCounter=0;
          gOSCCounter=0;
          // and if in range, increment comparison number ...
          if(++gComparIndex<=21){
            gPlaybackText=" ";
            gSubmitText=" ";
            gProgressText="Choice " + to_string(gComparIndex) + " of 21.";
          }
          // otherwise close task and write results and prep Localisation task
          else {
            gPlaybackText="END OF TASK";
            gSubmitText=" ";
            gProgressText="END";
            for(int i=0;i<7;i++){
              gHRTFResponseCount[i]=count(gHRTFResponses+1,gHRTFResponses+22,i);
              rt_printf("Score for HRTF %i is %i \n", i, gHRTFResponseCount[i]);
            }
            writeHRTFResponses();
            getWinnerAndLoser();
            rt_printf("Winning HRTF is %i \n", gWinningHRTF);
            rt_printf("Losing HRTF is %i \n", gLosingHRTF);
            // enable head tracking reading and reinitialise IMU for good measure
            gHeadLocked=0;
            setupIMU(44100);
          }
          gChoiceState=kNoneSelected;
          gCurrentState=kStopped;
        }
      }
    }
  }

  // Otherwise, if we have finished the HRTF comparison
  else {
    //Listen for the following messages
    // If using toggle calibration, rotate through states and notificatons
    if (msg->match("/one/calibrate").popFloat(floatArg).isOkNoMoreArgs()){
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
        // For the final state, also prep for the start of the Localisation task:
        else if(gCalibrateState==kElevation){
          gCalibrateState=kAhead;
          gCalibrateText="Look ahead";
          // update azi/ele and HRTF for source according to Location trial index
          gVBAPDefaultAzimuth[1]=gLocationTrials[gLocationIndex][0];
          gVBAPDefaultElevation[1]=gLocationTrials[gLocationIndex][1];
          gHRTF=gLocalisationHRTF[gLocationIndex];
          // silence HRTF audio and raise Location trial
          gInputVolume[0]=0.0;
          gInputVolume[1]=0.9;
          // switch looping on, reset counters and restart audio
          //gLooping=true;
          gTimeCounter=0;
          gOSCCounter=0;
          gCurrentState=kPlaying;
        }
      }
    }

    // If a location is submitted within range:
    else if (gFixedTrajectory && gLocationIndex<42){
      if (msg->match("/two/positionSubmit").popFloat(floatArg).isOkNoMoreArgs()){
        // When button is pressed ...
        if(floatArg>0.0){
          // log head-tracked azi/ele and time taken for current source ...
          gLocalisationResponses[gLocationIndex][0]=gVBAPUpdateAzimuth[1];
          gLocalisationResponses[gLocationIndex][1]=gVBAPUpdateElevation[1];
          gLocalisationResponseTimes[gLocationIndex]=gTimeCounter;
          // then stop audio.
          gCurrentState=kStopped;
        }
        // When button is released ...
        if(floatArg==0.0){
          // increment comparison number and azi/ele values ...
          gLocationIndex++;
          gVBAPDefaultAzimuth[1]=gLocationTrials[gLocationIndex][0];
          gVBAPDefaultElevation[1]=gLocationTrials[gLocationIndex][1];
          // then restart audio and counters.
          gCurrentState=kPlaying;
          gTimeCounter=0;
          gOSCCounter=0;
          // If an example task, just change the text ...
          if(gLocationIndex<2){
            gLocationText="Example " + to_string(gLocationIndex+1);
          }
          // otherwise change the text and check the required HRTF selection.
          else if(gLocationIndex>=2 && gLocationIndex<42){
            gLocationText="Target " + to_string(gLocationIndex-1) + " of 40.";
            if(gLocationIndex<22){
              if(HRTFComboOrd[0]==0){
                gHRTF=gLosingHRTF;
                gLocalisationHRTFState[gLocationIndex]=0;
              }
              else {
                gHRTF=gWinningHRTF;
                gLocalisationHRTFState[gLocationIndex]=1;
              }
            }
            else if (gLocationIndex >=22 && gLocationIndex<42){
              if(HRTFComboOrd[1]==0){
                gHRTF=gLosingHRTF;
                gLocalisationHRTFState[gLocationIndex]=0;
              }
              else {
                gHRTF=gWinningHRTF;
                gLocalisationHRTFState[gLocationIndex]=1;
              }
            }
            gLocalisationHRTF[gLocationIndex]=gHRTF;
          }
          // Otherwise close task and write results.
          else {
            gLocationText="END";
            writeLocationResponses();
            gCurrentState=kStopped;
          }
        }
      }
    }

  }
}

int localPort = 7562;
int remotePort = 7563;
const char* remoteIp = "192.168.1.4";
int monitorPort = 9001;
const char* monitorIp = "192.168.1.1";

bool setupOSC(){
    // setup OSC ports
	oscPipe.setup("incomingOsc");
    oscServer.setup(localPort, on_receive);
    oscClient.setup(remotePort, remoteIp);
    oscMonitor.setup(monitorPort, monitorIp);

    // the following code sends an OSC message to address /osc-setup
  	oscClient.newMessage("/osc-setup").send();

  	printf("Waiting for handshake ....\n");
  	// we want to stop our program and wait for a new message to come in.
  	// therefore, we set the pipe to blocking mode.
  	oscPipe.setBlockingNonRt(false);
  	oscPipe.setBlockingRt(true);
  	oscPipe.setTimeoutMsRt(1000);
  	oscpkt::Message* msg = nullptr;
  	int ret = oscPipe.readRt(msg);
  	bool ok = false;
  	if(ret > 0) {
  		if(msg && msg->match("/osc-setup-reply"))
  		{
  			printf("handshake received!\n");
  			ok = true;
  		}
  		delete msg;
  	}
  	if(!ok) {
  		fprintf(stderr, "No handshake received: %d\n", ret);
  		return false;
  	}
  	// in the remainder of the program, we will be calling readRt() from render(), and we want it
  	// to return immediately if there are no new messages available. We therefore set the
  	// pipe to non-blocking mode
  	oscPipe.setBlockingRt(false);

    oscClient.newMessage("/one/choiceText").add(gProgressText).send();
    oscClient.newMessage("/one/playAText").add(std::string("*Play A*")).send();
    oscClient.newMessage("/one/playBText").add(std::string("*Play B*")).send();
    oscClient.newMessage("/one/playbackText").add(gPlaybackText).send();
    oscClient.newMessage("/one/chooseAToggle").add(0.0f).send();
    oscClient.newMessage("/one/chooseBToggle").add(0.0f).send();
    oscClient.newMessage("/one/submitAnsText").add(gSubmitText).send();


	return true;
}

void checkOSC(){


  // send curret status by OSC
  if(++gOSCCounter>=8820){

    oscpkt::Message* msg;
  	// read incoming messages from the pipe

		// if the touch pad is being held, enable solo mode
		if(CurrentSwipeValue[0]==1 || CurrentSwipeValue[1]==1 || \
			CurrentSwipeValue[2]==1 || CurrentSwipeValue[3]==1 || \
			CurrentSwipeValue[4]==1 || CurrentSwipeValue[5]==1 || \
			CurrentSwipeValue[6]==1 || CurrentSwipeValue[7]==1){
				gCurrentSceneMode=true;
			}
		// otherwise set to concurrent mode
		else{
			gCurrentSceneMode=false;
		}
		// if solo mode has just been enabled
		if(gCurrentSceneMode==true && gPreviousSceneMode ==false){

			// load the corresponding voiceover file
			changeAudioFiles(5,gCurrentTargetSong+5);
			// switch to the corresponding target song state
			gTargetState=gCurrentTargetSong+5;
			// update the default location for the voiceover
			for(int i=0;i<3;i++){
				gVBAPActiveVector[5][i]=gVBAPDefaultVector[gCurrentTargetSong+5][i];
			}
			for(int song=0; song<5; song++){
				int position=0;
				// (treat all positions as positive)
				if(gVBAPUpdateAzimuth[song]<0) {
					position = gVBAPUpdateAzimuth[song]*-1;
				}
				else {
					position = gVBAPUpdateAzimuth[song];
				}
				gInputVolume[song]=0.0;
				if(position<=18) {
					gInputVolume[song]=1.0;
					// update the default location for the voiceover
				}
			}
		}
		// if touch pad has just been released
		if(gCurrentSceneMode==false && gPreviousSceneMode ==true){
			// check for an accept gesture
			if(CurrentLocation>PreviousLocation) {
				rt_printf("ACCEPT \n");
				startPlayback(7);

			}
			// check for a reject getsture
			if(CurrentLocation<PreviousLocation) {
				rt_printf("REJECT \n");
				startPlayback(8);
			}
		}
		// update scene modes and locations
		gPreviousSceneMode=gCurrentSceneMode;
		PreviousLocation=CurrentLocation;

  	while(oscPipe.readRt(msg) > 0)
  	{
      parseMessage(msg);
  		oscPipe.writeRt(msg); // return the pointer to the other thread, where it will be destroyed
  	}
		sendCurrentStatusOSC();
    oscPipe.setBlockingRt(false);
    gOSCCounter=0;
    gTimeCounter+=0.2;
  }
}

void sendCurrentStatusOSC(){
  oscClient.newMessage("/one/aziText").add(to_string(gVBAPUpdateAzimuth[1])).send();
  oscClient.newMessage("/one/eleText").add(to_string(gVBAPUpdateElevation[1])).send();
  oscClient.newMessage("/two/targetText").add(gLocationText).send();
  oscClient.newMessage("/one/choiceText").add(gProgressText).send();
  oscClient.newMessage("/one/playbackText").add(gPlaybackText).send();
  oscClient.newMessage("/one/submitAnsText").add(gSubmitText).send();
  oscClient.newMessage("/one/calibrateText").add(gCalibrateText).send();
  if(gHeardAState){
    oscClient.newMessage("/one/playAText").add(std::string("Play A")).send();
  }
  else {
    oscClient.newMessage("/one/playAText").add(std::string("*Play A*")).send();
  }
  if(gHeardBState){
    oscClient.newMessage("/one/playBText").add(std::string("Play B")).send();
  }
  else {
    oscClient.newMessage("/one/playBText").add(std::string("*Play B*")).send();
  }

  if(gChoiceState==kNoneSelected){
    oscClient.newMessage("/one/chooseAToggle").add(0.0f).send();
    oscClient.newMessage("/one/chooseBToggle").add(0.0f).send();
  }
  else if (gChoiceState==kASelected) {
    oscClient.newMessage("/one/chooseAToggle").add(1.0f).send();
    oscClient.newMessage("/one/chooseBToggle").add(0.0f).send();
  }
  else if (gChoiceState==kBSelected) {
    oscClient.newMessage("/one/chooseAToggle").add(0.0f).send();
    oscClient.newMessage("/one/chooseBToggle").add(1.0f).send();
  }

}

void cleanupOSC(){
	oscpkt::Message* returnedMsg;
	// drain the pipes, so that any objects trapped in there can be appropriately destroyed
	while(oscPipe.readRt(returnedMsg) > 0)
	{
		delete returnedMsg;
	}
	while(oscPipe.readNonRt(returnedMsg) > 0)
	{
		delete returnedMsg;
	}
}

#endif /* OSC_ */
