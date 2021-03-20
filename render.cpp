/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 */

// include files
#include <Bela.h>
#include <cmath>
#include <libraries/ne10/NE10.h>			         // neon library
#include "spatialisation/SampleStream.h"       // adapted audio streaming code
#include "spatialisation/SpatialSceneParams.h" // definition of audio context
#include "spatialisation/ImpulseLoader.h"      // code for loading HRTF IR files
#include "spatialisation/ImpulseData.h"        // struct file to store IR data
#include "spatialisation/VBAPData.h"           // VBAP speaker weighting lookup
#include "spatialisation/TrackData.h"          // Tranks filenames loopkup
#include "spatialisation/TestRoutine.h"        // defined trajectory testing
#include "OSC.h"                               // OSC interfacing
#include "VectorRotations.h"                   // point source vector rotation
#include "spatialisation/Spatialisation.h"     // spatialisation engine
#include "spatialisation/SpatialFocus.h"       // interaction engine

#include "reverb/Network.h"
SDN::Network *reverb;

// user controlled variables from main.cpp
extern bool gFixedTrajectory;
extern bool gTestMode;

int gPlaybackState=kPlaying;
bool gHeadLocked=0;

// volume level variables for individual streams and main output
float gInputVolume[NUM_STREAMS]={};
float gMainVol=0.7;

// instantiate the sampleStream class for each stream
SampleStream *sampleStream[NUM_STREAMS];

// instantialte auxiliary task to fill buffers
AuxiliaryTask gFillBuffersTask;

// helper functions for this code
void createEnvironment(int sampleRate);
void loadAudioFiles();
void setupLocalisationTests();
void reinitialiseAudioStreams();
void fillBuffers(void*);
void applyTestCoords();


// configure Bela environment for playback
bool setup(BelaContext *context, void *userData)
{
  createEnvironment(context->audioSampleRate);  // create AAR environment
  system("ifdown wlan0; ifup wlan0;");          // connect to wifi
  setupIMU(context->audioSampleRate);           // initialise IMU
  pinMode(context, 0, buttonPin, INPUT);        // initialise button for IMU
  loadImpulse(gHRIRLength);                     // load HRIRs
  getTaskTracks();                              // import track filenames
  loadAudioFiles();                             // load audio files
  prepFFTBuffers();                             // set up FFT
  transformHRIRs(gHRIRLength, gConvolutionSize);// convert HRIRs to Hz domain
  getVBAPMatrix();                              // import VBAP speaker gains
  createVectors();                              // create starting vectors
  setupOSC();                                   // setup OSC communication
  initFocusScene();                             // calculate focus gain values
  if(gFixedTrajectory)setupLocalisationTests();
  initFFTProcesses();                            // initialise FFT processing
  if((gFillBuffersTask = Bela_createAuxiliaryTask(&fillBuffers, 89, \
    "fill-buffer")) == 0) return false;          // fill buffers
  return true;
}

// render the audio scene
void render(BelaContext *context, void *userData){
  // check if buffers need filling using low priority auxiliary task
  Bela_scheduleAuxiliaryTask(gFillBuffersTask);



  // process the next audio frame
  for(unsigned int n = 0; n < context->audioFrames; n++) {

    // push/pop OSC messages
    checkOSC();


    // if playback is paused:
    if(gPlaybackState==kPaused){
      // kill the output
      context->audioOut[n * context->audioOutChannels + 0] = 0.0;
      context->audioOut[n * context->audioOutChannels + 1] = 0.0;
    }

    //if playback has just been stopped:
    else if(gPlaybackState==kStopped){
      // kill the output
      context->audioOut[n * context->audioOutChannels + 0] = 0.0;
      context->audioOut[n * context->audioOutChannels + 1] = 0.0;

      reinitialiseAudioStreams();

      // pause playback
      gPlaybackState=kPaused;
    }

    // if playback is active, proccess the audio
    else if(gPlaybackState==kPlaying){

      //use a defined 10 second trajectory for HRTF selection purposes
      if(gFixedTrajectory && !gTestMode)updatePositions();

      // run the spatialisation algorithm
      spatialiseAudio();

      //use a defined test routine for verifying HRTF convolution outputs
      if(gTestMode)applyTestCoords();

      // instantiate and initialise reverb send variable
      float reverbInput=0.0;

      // PROCESS ALL streams
      for(int stream=0; stream<NUM_STREAMS-1; stream++){
        sampleStream[stream]->processFrame();

      }

      // INPUTS TO AURALISATION SYSTEM:
      // process and read frames for each 3D stream  into input buffers
      for(int stream=0; stream<NUM_SIM_3D_STREAMS; stream++){

        // process stream for distance attenuation
        reverb->writeToDrySource(\
          sampleStream[gPlaybackStates[gTargetState][stream]]->getSample(0) \
          * gInputVolume[gPlaybackStates[gTargetState][stream]],stream);
        // feed 0.3 of stream to reverb generator
        reverbInput += sampleStream[gPlaybackStates[gTargetState][stream]] \
          ->getSample(0) * gInputVolume[gPlaybackStates[gTargetState][stream]] \
          * 0.4;
        // add stream with distance attenuation to input buffer
        gInputBuffer[stream][gInputBufferPointer] = \
          reverb->readFromDrySource(stream);
      }

      // NON-SPATIALISED OUTPUT SUMMING:
      // sum monoaural notification signals
      float monoOut=0.0;
      for(int notifs=NUM_VBAP_TRACKS; notifs<NUM_STREAMS-1; notifs++){
        monoOut += sampleStream[notifs]->getSample(0);
      }
      gOutputBufferL[gOutputBufferWritePointer]+=monoOut;
      gOutputBufferR[gOutputBufferWritePointer]+=monoOut;

      //REVERB RETURN:
      // add reverb output to spare stream
      gInputBuffer[NUM_STREAMS-1][gInputBufferPointer] =  \
        reverb->scatterMono(reverbInput);

      // copy output buffer L/R to audio output L/R
      for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
        if(channel == 0) {
          context->audioOut[n * context->audioOutChannels + channel] = \
            gOutputBufferL[gOutputBufferReadPointer] * gMainVol;
        }
        else if (channel == 1){
          context->audioOut[n * context->audioOutChannels + channel] = \
            gOutputBufferR[gOutputBufferReadPointer] * gMainVol;
        }
      }
    }
  }
}

// function to create virtual listening environment
void createEnvironment(int sampleRate){
  // set variables for room dimensons and source positions
  float roomWidth, roomLength, roomHeight, hrtfDepth, latCentre, longCentre, \
    earLevel, sourceDepth, sourceLat, sourceLong, sourceVertical;
  roomWidth = 3;
  roomLength = 3;
  roomHeight = 3;
  hrtfDepth = 1;
  latCentre = roomWidth/2;
  longCentre = roomLength/2;
  earLevel = 1.6;
  // create new SDN reverb network and initiate objects
  reverb = new SDN::Network(sampleRate, roomWidth, roomLength, roomHeight);
  reverb->setSourcePosition(latCentre, longCentre + hrtfDepth, earLevel);
  reverb->setMicPosition(latCentre, longCentre, earLevel);
  // assign positions of source signals within virtual environment
  for(int stream=0;stream<NUM_SIM_3D_STREAMS;stream++){
    sourceDepth = hrtfDepth*cos(gVBAPDefaultElevation[stream]*M_PI/180.0);
    sourceLat = sourceDepth*sin(gVBAPDefaultAzimuth[stream]*M_PI/180.0);
    sourceLong = sourceDepth*cos(gVBAPDefaultAzimuth[stream]*M_PI/180.0);
    sourceVertical = hrtfDepth*sin(gVBAPDefaultElevation[stream]*M_PI/180.0);
    // create delay lines for dry source signals
    float distance = reverb->addDrySource(sampleRate, \
      latCentre+sourceLat, longCentre+sourceLong, earLevel+sourceVertical, \
        stream);
      /*rt_printf("Distance %i: %f (X: %f; Y: %f; Z: %f) \n", stream, \
      distance, sourceLat, sourceLong, sourceVertical);*/
  }
}

// function to prepare audio files for playback
void loadAudioFiles(){
  // load music files into buffers 1-5 or a voiceover file into buffer 6
  for(int track=0; track<NUM_VBAP_TRACKS; track++) {
    if(track<=4) {
      std::string file= "./tracks/" + taskOne[track] + ".wav";
      const char * id = file.c_str();
      sampleStream[track] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE,true);
      gInputVolume[track]=1.0;
      updatePlaylistLog(track, track, true, 0.0, 0.0);
      sampleStream[track]->togglePlayback(1);
    }
    // load the first voiceover file into buffer 6
    else {
      std::string file= "./tracks/" + taskOne[0] + "_VXO.wav";
      const char * id = file.c_str();
      sampleStream[track] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE,false);
      gInputVolume[track] = 1.2;
      sampleStream[track]->togglePlayback(0);
    }
  }
  // load the sfx into other buffers
  for(int sfx=NUM_VBAP_TRACKS; sfx<NUM_STREAMS-1; sfx++) {
    std::string number=to_string(sfx-5);
    std::string file= "./sfx/sfx" + number + ".wav";
    const char * id = file.c_str();
    sampleStream[sfx] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE,false);
    gInputVolume[sfx]=1.5;
    sampleStream[sfx]->togglePlayback(0);
  }
}

// function to setup localisation tests
void setupLocalisationTests() {
  createPairs();                                // create HRTF comparisons
  createLocations();                            // create source locations
  std::string file1= "./tracks/trumpet1.wav";   // load HRTF comparison stimulus
  std::string file2= "./tracks/trumpet2.wav";   // load localisation stimulus
  const char * trajectoryStim = file1.c_str();
  const char * sourceStim = file2.c_str();
  sampleStream[0]->openFile(trajectoryStim,NUM_CHANNELS,BUFFER_SIZE,false);
  sampleStream[1]->openFile(sourceStim,NUM_CHANNELS,BUFFER_SIZE,true);
  // mute all other streams
  for(int stream=1; stream<NUM_STREAMS; stream++) gInputVolume[stream]=0.0;                         // mute second stream
}

// function to change audio files during playback
void changeAudioFiles(int scenePosition, int newTrack, string fileType){
    // stop the current track and delete the playback object
    sampleStream[scenePosition]->stopPlaying();
    // create a new playback object and load the replacement .wav file
    std::string file = "./tracks/" + taskOne[newTrack]+ fileType;
    const char * id = file.c_str();
    rt_printf("%s \n",id);

    sampleStream[scenePosition]->openFile(id,NUM_CHANNELS,BUFFER_SIZE,false);
    sampleStream[scenePosition]->togglePlayback(1);
}

// function to pause all music files during playback
void pauseAllMusic(float fade){
    for(int track=0; track<NUM_VBAP_TRACKS-1; track++){
      sampleStream[track]->togglePlayback(1);
      sampleStream[track]->togglePlaybackWithFade(-1,fade); // pause with fade
    }
}

// function to resume all music files during playback
void resumeAllMusic(float fade){
    for(int track=0; track<NUM_VBAP_TRACKS-1; track++){
      sampleStream[track]->togglePlayback(0);
      sampleStream[track]->togglePlaybackWithFade(1,fade);  // resume with fade
    }
}

// function to pause a specified audio file during playback
void pausePlayback(int stream){
    sampleStream[stream]->togglePlayback(1);
    sampleStream[stream]->togglePlaybackWithFade(-1,0.001);
}

// function to resume a specified audio file during playback
void resumePlayback(int stream){
  // start test trajectory track
    sampleStream[stream]->togglePlayback(0);
    sampleStream[stream]->togglePlaybackWithFade(1,0.5);
}

// function to start a specified file from the buffered stream
void startPlayback(int stream){
  // start test trajectory track
    sampleStream[stream]->togglePlayback(1);
}

void reinitialiseAudioStreams(){
  // stop all tracks and set to refill input buffers
  for(int stream=0; stream<NUM_STREAMS-1; stream++){
    sampleStream[stream]->stopPlaying();
  }
  Bela_scheduleAuxiliaryTask(gFillBuffersTask);
  // reset pointers and counters
  gOutputBufferReadPointer = 0;
  gOutputBufferWritePointer = gHopSize;
  gInputBufferPointer = 0;
  gFFTInputBufferPointer=0;
  gFFTOutputBufferPointer=0;
  gSampleCount=0;
  // clear output buffers
  for(int i=0; i<BUFFER_SIZE; i++){
    gOutputBufferL[i]=0;
    gOutputBufferR[i]=0;
  }
  // freeze the trajectory if in that mode
  if(gFixedTrajectory){
    gTrajectoryCount=0;
    gOrbitCount=0;
    gVBAPDefaultAzimuth[0]=0;
    gVBAPDefaultElevation[0]=0;
    gPlaybackText=" ";
  }
}

// function to fill buffers for specified number of streams on startup
void fillBuffers(void*) {
  for(int track=0; track<NUM_STREAMS-1; track++) {
    if(sampleStream[track]->bufferNeedsFilled())
    sampleStream[track]->fillBuffer();
  }
}


// function to apply test values to source 0 azimuth and elevation settings,
// then write these to an array
void applyTestCoords(){
  gVBAPUpdatePositions[0]=((gTestElevation+90)*361)+gTestAzimuth+180;
  writeOutput(gOutputBufferL[gOutputBufferReadPointer], \
    gOutputBufferR[gOutputBufferReadPointer]);
}


void cleanup(BelaContext *context, void *userData)
{
  // clear all input buffers
  for(int i=0;i<NUM_STREAMS;i++) {
    delete sampleStream[i];
  }
  // clear impulses
  clearImpulseFFTBuffers();
  clearSignalFFTBuffers();
  // write test file if in test mode
  if(gTestMode){
    writeDataFile(gConvolutionSize);
  }
}
