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
#include "spatialisation/SpatialSceneParams.h" // definition of audio context
#include "spatialisation/SampleStream.h"       // adapted audio streaming code
#include "spatialisation/ABRoutine.h"          // HRTF comparison trial
#include "spatialisation/SpatialFocus.h"       // head-tracked gain code
#include "spatialisation/TrackData.h"          // playlist interaction handling


//variables from VectorRotations.h
extern int gVBAPDefaultAzimuth[NUM_FIXED_POSITIONS];
extern int gVBAPDefaultElevation[NUM_FIXED_POSITIONS];
extern float gVBAPDefaultVector[NUM_FIXED_POSITIONS][3];
extern float gVBAPActiveVector[NUM_VBAP_TRACKS][3];
extern int gVBAPUpdateAzimuth[NUM_VBAP_TRACKS];
extern int gVBAPUpdateElevation[NUM_VBAP_TRACKS];


// variables from render.cpp
extern bool gFixedTrajectory;
extern int gPlaybackState;
extern bool gHeadLocked;
extern float gInputVolume[NUM_STREAMS];
extern float gMainVol;
extern bool setupIMU(int sampleRate);
extern void changeAudioFiles(int oldTrack, int newTrack, string filetype);
extern void pauseAllMusic(float fade);
extern void resumeAllMusic(float fade);
extern void pausePlayback(int stream);
extern void resumePlayback(int stream);
extern void startPlayback(int stream);


// OSC initialisation and communication handling
Pipe oscPipe;
OscReceiver oscServer;
OscSender oscClient;
OscSender oscMonitor;
bool handshakeReceived;

// helper functions for this code
void sendCurrentStatusOSC();
void checkChannelMessages(oscpkt::Message* msg);
void checkGlobalMessages(oscpkt::Message* msg);
void runHRTFComparisonChecks(oscpkt::Message* msg);
void runLocalisationTestChecks(oscpkt::Message* msg);
void sendHRTFLocMsgs();
void changeTrack(int notification);


// AB comparison states
bool gHeardAState = false;				// has example A been heard in entirety
bool gHeardBState = false;				// has example B been heard in entirety

// head-tracker interfacing
int gHRTF=0;		    							// HRTF set for binauralisation
bool gCalibrate=0;  							// headtracking calibration state


bool gCurrentSceneMode=false;			// current solo/concurrent stream mode
bool gPreviousSceneMode=false;		// previous solo/concurrent stream mode
int gCurrentSwipeValue[16]={};		// swipe interface states
int gCurrentLocation=0;						// current swipe interface contact position
int gPreviousLocation=0;					// previous swipe interface contact position
bool gTransitionContact=false;		// transition contact state
bool gCurrentSwipeActivity=false;
bool gPreviousSwipeActivity=false;



int gFS=44100;										// global sample rate
float gOSCRate=8820;							// OSC refresh rate
float gTimeInc=gOSCRate/gFS;			// time increment
int gOSCCounter=0;								// counter for OSC check interval
int gTaskCounter=1;								// current task number
float gTimeCounter=0;							// counter for time event logging
int gPlaylistCounter=5;						// counter for current playlist
int gRejectCounter=0;							// running tally of rejected songs
int gLastRemovedTrack=0;					// variable to store removed track ID
int gSceneTracks[5]={0,1,2,3,4};	// current tracks
int gEnd=gDynamicPlaylist[gTaskCounter-1].size(); // max count for playlist
bool gPauseState=true;
string gPauseText="START STUDY";




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


	// if not currently paused, record any activity on the swipe area:
	if(!gPauseState){
		for(int buttonNo=1; buttonNo<=16; buttonNo++){
			std::string buttonID=to_string(buttonNo);
			// store any button activation event and its location
			if (msg->match("/two/1/"+buttonID).popFloat(floatArg).isOkNoMoreArgs()){
				// store the button value in the array
				gCurrentSwipeValue[buttonNo-1]=floatArg;
				// if it is a movement off a button, update the new location
				if(floatArg==0.0)	{
					gCurrentLocation=buttonNo;
					gTransitionContact=true;
				}
				// if it is a movement on a button, update both locations
				if(floatArg==1.0)	{
					gPreviousLocation=gCurrentLocation;
					gCurrentLocation=buttonNo;
				}
				//rt_printf("received mode command %i, %f \n", buttonNo, floatArg);
				// also set the transition flag to true for release events
				gCurrentSwipeActivity=true;
			}
		}
	}


	checkChannelMessages(msg);
	checkGlobalMessages(msg);

	// If in test mode and HRTF comparison index is within range
	if(gFixedTrajectory && gComparIndex<22){
		// Run HRTF comparison routine checks
		runHRTFComparisonChecks(msg);
	}
	// If in test mode and HRTF comparison is complete
	else if (gFixedTrajectory && gComparIndex>=22){
		//Run localisation test routine checks
		runLocalisationTestChecks(msg);
	}
}

//variables for OSC ports
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
	}

	// in the remainder of the program, we will be calling readRt() from render(), and we want it
	// to return immediately if there are no new messages available. We therefore set the
	// pipe to non-blocking mode
	oscPipe.setBlockingRt(false);

	oscClient.newMessage("/two/mainvol").add(25.0f).send();
	oscClient.newMessage("/two/taskState").add(gPauseText).send();
	oscClient.newMessage("/one/choiceText").add(gProgressText).send();
	oscClient.newMessage("/one/playAText").add(std::string("*Play A*")).send();
	oscClient.newMessage("/one/playBText").add(std::string("*Play B*")).send();
	oscClient.newMessage("/one/playbackText").add(gPlaybackText).send();
	oscClient.newMessage("/one/chooseAToggle").add(0.0f).send();
	oscClient.newMessage("/one/chooseBToggle").add(0.0f).send();
	oscClient.newMessage("/one/submitAnsText").add(gSubmitText).send();

	// confirm setup
	return true;
}


void checkOSC() {
	// send/receive curret status by OSC every 1/5 sec
	if(++gOSCCounter>=gOSCRate){


		oscpkt::Message* msg;
		// read incoming messages from the pipe
		while(oscPipe.readRt(msg) > 0)
		{
			parseMessage(msg);
			oscPipe.writeRt(msg); // return the pointer to the other thread, where it will be destroyed
		}

		// if the touch pad is active ( held or in transition), enable solo mode
		for(int swipeVal=0; swipeVal<16; swipeVal++){
			if(gTransitionContact || gCurrentSwipeValue[swipeVal]==1){
				gCurrentSceneMode=true;
				break;
			}
			gCurrentSceneMode=false;
		}

		// if we are in solo mode
		if(gCurrentSceneMode){
			// increment the active listening time of the target song
			gActiveListen[gSceneTracks[gCurrentTargetSong]]+=gTimeInc;
		}

		// if solo mode has just been enabled
		if(gCurrentSceneMode && !gPreviousSceneMode){
			// switch to the corresponding target song state
			gTargetState=gCurrentTargetSong+5;
			// update the voiceover file
			changeAudioFiles(5,gSceneTracks[gCurrentTargetSong],"_VXO.wav");
			// update the default location for the voiceover
			for(int i=0;i<3;i++){
				gVBAPActiveVector[5][i]=gVBAPDefaultVector[gCurrentTargetSong+5][i];
			}
			// find the target song position and set playback volumes
			for(int song=0; song<5; song++){
				// set target song volume to maximum
				if(song==gCurrentTargetSong) gInputVolume[song]=1.0;
				// mute other songs
				else gInputVolume[song]=0.0;
			}
		}

		// if touch pad has just been released
		if(!gCurrentSceneMode && gPreviousSceneMode && gPreviousSwipeActivity){
			// check if we are paused
			if(gPauseState==true){
				resumeAllMusic(2.0);
				gPauseText="PAUSE STUDY";
				gPauseState=false;
			}

			// stop the voiceover
			pausePlayback(5);
			// if there was an accept gesture, change the track
			if(gCurrentLocation>gPreviousLocation) {
				changeTrack(7);
			}
			// if there was a reject getsture
			if(gCurrentLocation<gPreviousLocation) {
				// log the track as rejected
				gSelectionStatus[gSceneTracks[gCurrentTargetSong]]=false;
				gLastRemovedTrack=gSceneTracks[gCurrentTargetSong];
				// change the track
				changeTrack(8);
				// compress the playlist in a low priority thread
				Bela_scheduleAuxiliaryTask(gReorderPlaylist);
			}
			// otherwise exit solo mode gracefully
			//else{
				for(int song=0; song<5; song++){
					if(song!=gCurrentTargetSong){
						resumePlayback(song);
					}
				}
			//}
		}

		// update scene modes and locations
		gPreviousSceneMode=gCurrentSceneMode;
		gPreviousSwipeActivity=gCurrentSwipeActivity;
		if(!gPreviousSwipeActivity) gPreviousLocation=gCurrentLocation;
		gCurrentSwipeActivity=false;
		gTransitionContact=false;

		// send current status to UI
		sendCurrentStatusOSC();
		// close OSC pipe
		oscPipe.setBlockingRt(false);
		// update counters
		gOSCCounter=0;
		// if not paused, increment times
		if(gPauseState==false){
			// increment time counter
			gTimeCounter+=gTimeInc;
			// increment background listening counters
			for(int song=0; song<5; song++){
				gBackgroundListen[gSceneTracks[song]]+=gTimeInc;
			}
		}
	}
}

void changeTrack(int notification){
	// play notification
	startPlayback(notification);
	//rt_printf("Old = %i; ", gSceneTracks[gCurrentTargetSong]);


	// change song to the next in dynamic list not currenlty playing
	bool avoidDuplicate=true;
	while(avoidDuplicate){
		// assume the next song is not the same as any currently playing
		avoidDuplicate=false;
		// check if there is a duplicate
		for(int song=0; song<5; song++){
			// if any matches the next in the dynamic playlist
			if (gDynamicPlaylist[gTaskCounter-1][gPlaylistCounter]==gSceneTracks[song]){
				// avoid this one, increment the playlist and check again
				avoidDuplicate=true;
				if(++gPlaylistCounter==gEnd)gPlaylistCounter=0;
			}
		}
	}

	// if there's no match proceed with the next track
	gSceneTracks[gCurrentTargetSong]=gDynamicPlaylist[gTaskCounter-1][gPlaylistCounter];

	//rt_printf("New = %i; ", gSceneTracks[gCurrentTargetSong]);
		// move playlist counter and if we've reached the end
	if(++gPlaylistCounter==gEnd){
		// go back to the beginning
		gPlaylistCounter=0;
		// play the notification
		startPlayback(9);
	}
	//rt_printf("Counter = %i; Range = %i \n",	\
		gPlaylistCounter, gDynamicPlaylist[gTaskCounter-1].size());
	// load the new audio file
	changeAudioFiles(gCurrentTargetSong,gSceneTracks[gCurrentTargetSong],".wav");
	updatePlaylistLog(gSceneTracks[gCurrentTargetSong], gCurrentTargetSong, \
		gTimeCounter);
	// fade back all tracks
	resumeAllMusic(1.5);
	//rt_printf("Current Tracks: %i, %i, %i, %i, %i\n", \
		gSceneTracks[0], gSceneTracks[1], gSceneTracks[2], \
		gSceneTracks[3], gSceneTracks[4]);
}

void sendCurrentStatusOSC(){
	oscClient.newMessage("/one/aziText").add(to_string(gVBAPUpdateAzimuth[2])).send();
	oscClient.newMessage("/one/eleText").add(to_string(gVBAPUpdateElevation[2])).send();
	oscClient.newMessage("/one/calibrateText").add(gCalibrateText).send();
	oscClient.newMessage("/two/taskCount").add(to_string(gTaskCounter)).send();
	oscClient.newMessage("/two/taskLength").add(to_string(gTaskAnswers[gTaskCounter-1])).send();
	oscClient.newMessage("/two/taskDescriptor").add(gTaskText[gTaskCounter-1]).send();
	oscClient.newMessage("/two/taskProgress").add(to_string(gDynamicPlaylist[gTaskCounter-1].size())).send();
	oscClient.newMessage("/two/taskState").add(gPauseText).send();


	if(gFixedTrajectory) sendHRTFLocMsgs();



}

// helper function for checking channel OSC messages related source placement
void checkChannelMessages(oscpkt::Message* msg){
	int intArg;
	float floatArg;
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
		}
		else if (msg->match("/one/volume"+number).popFloat(floatArg).isOkNoMoreArgs()){
			rt_printf("received volume command %f \n", floatArg);
			gInputVolume[stream]=floatArg;
			rt_printf("Source volume is %f \n", gInputVolume[stream]);
		}
	}
}


// helper function for checking global OSC messages related to playback setup
void checkGlobalMessages(oscpkt::Message* msg){
	int intArg;
	float floatArg;
	//Global controls (HRTF, head-tracker calibration, IMU reinit, pause, mainVol)
	if (msg->match("/one/calibrate").popFloat(floatArg).isOkNoMoreArgs()){
		//rt_printf("received calibrate command %i \n", intArg);
		gCalibrate=floatArg;
		//rt_printf("Calibration is %i \n", gCalibrate);
	}
	else if (msg->match("/one/hrtf").popInt32(intArg).isOkNoMoreArgs()){
		rt_printf("received HRTF command %i \n", intArg);
		gHRTF=intArg;
		gPlaybackState=kStopped;
		rt_printf("HRTF is %i \n", gHRTF);
	}
	else if (msg->match("/one/reinitIMU").popFloat(floatArg).isOkNoMoreArgs()){
		if(floatArg>0.0){
			setupIMU(gFS);
		}
	}
	else if (msg->match("/two/pause").popFloat(floatArg).isOkNoMoreArgs()){
		if(floatArg==0.0){
			//rt_printf("received pause command: %f \n", floatArg);
			if(gPauseState==false){
				pauseAllMusic(2.0);
				gPauseText="RESUME STUDY";
				gPauseState=true;
			}
			else {
				resumeAllMusic(2.0);
				gPauseText="PAUSE STUDY";
				gPauseState=false;
			}
		}
	}
	else if (msg->match("/two/mainvol").popFloat(floatArg).isOkNoMoreArgs()){
		//rt_printf("received main volume change: %f \n", floatArg);
		gMainVol=log10(floatArg/140) / log10(140) *-2;
	}
}

// helper function for HRTF comparison routine
void runHRTFComparisonChecks(oscpkt::Message* msg){
	float floatArg;
	// If "Play A" is tapped, set the correct playback text and playing states
	if (msg->match("/one/playAButton").popFloat(floatArg).isOkNoMoreArgs()){
		if(floatArg>0.0){
			gPlaybackText="";
			gPlayingA=false;
			gPlayingB=false;
			gPlaybackState=kStopped;
			gHRTF=gComparMatches[gComparIndex][0];
		}
		if(floatArg==0.0){
			gPlaybackText="PLAYING A";
			gPlaybackState=kPlaying;
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
			gPlaybackState=kStopped;
			gHRTF=gComparMatches[gComparIndex][1];
		}
		if(floatArg==0.0){
			gPlaybackText="PLAYING B";
			gPlaybackState=kPlaying;
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
					setupIMU(gFS);
				}
				gChoiceState=kNoneSelected;
				gPlaybackState=kStopped;
			}
		}
	}
}

// helper function for localisation test routine
void runLocalisationTestChecks(oscpkt::Message* msg)	{
	float floatArg;
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
				// reset counters and restart audio
				gTimeCounter=0;
				gOSCCounter=0;
				gPlaybackState=kPlaying;
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
				gPlaybackState=kStopped;
			}
			// When button is released ...
			if(floatArg==0.0){
				// increment comparison number and azi/ele values ...
				gLocationIndex++;
				gVBAPDefaultAzimuth[1]=gLocationTrials[gLocationIndex][0];
				gVBAPDefaultElevation[1]=gLocationTrials[gLocationIndex][1];
				// then restart audio and counters.
				gPlaybackState=kPlaying;
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
					gPlaybackState=kStopped;
				}
			}
		}
	}
}

void sendHRTFLocMsgs(){
	oscClient.newMessage("/one/choiceText").add(gProgressText).send();
	oscClient.newMessage("/one/playbackText").add(gPlaybackText).send();
	oscClient.newMessage("/one/submitAnsText").add(gSubmitText).send();
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
	oscClient.newMessage("/two/targetText").add(gLocationText).send();
}




// close OSC messaging
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
