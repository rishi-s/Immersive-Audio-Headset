/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 */

// include files
#include <Bela.h>
#include <cmath>
#include <libraries/ne10/NE10.h>			         // neon library
#include "spatialisation/SampleStream.h"       // adapted code for streaming/processing audio
#include "spatialisation/SpatialSceneParams.h" // definition of audio sources and context
#include "spatialisation/ImpulseLoader.h"      // code for loading HRTF IR files
#include "spatialisation/ImpulseData.h"        // struct file to store IR data
#include "spatialisation/VBAPData.h"           // lookup tables for VBAP speaker weightings
#include "spatialisation/TestRoutine.h"        // code for testing defined trajectory
#include "OSC.h"                // OSC interfacing
#include "VectorRotations.h"    // bespoke code for point source vector rotation
#include "spatialisation/Spatialisation.h"     // spatialisation engine

#include "reverb/Network.h"
SDN::Network *reverb;

// user controlled variables from main.cpp
extern int gStreams;
extern bool gFixedTrajectory;
extern bool gTestMode;

int gCurrentState=kPlaying;
bool gHeadLocked=0;

// volume level variables for individual streams
float gInputVolume[NUM_STREAMS]={};

// instantiate the sampleStream class for each stream
SampleStream *sampleStream[NUM_STREAMS];

// instantialte auxiliary task to fill buffers
AuxiliaryTask gFillBuffersTask;

// helper functions for this code
void loadAudioFiles();
void reinitialiseAudioStreams();
void fillBuffers(void*);
void applyTestCoords();


// configure Bela environment for playback
bool setup(BelaContext *context, void *userData)
{
  // create listening environment
  float width, length, height, distance;
	width = 5;
	length = 5;
	height = 3;
	distance = 1;
  reverb = new SDN::Network(context->audioSampleRate, width, length, height);
	reverb->setSourcePosition(width/2 + distance, length/2, 1.5);
	reverb->setMicPosition(width/2, length/2, 1.6);
  system("ifdown wlan0; ifup wlan0;");
  setupIMU(context->audioSampleRate);
  // set up button pin for calibration, if used
  pinMode(context, 0, buttonPin, INPUT);
  loadImpulse(gHRIRLength);                      // load HRIRs
  loadAudioFiles();                              // load audio files
  prepFFTBuffers();                              // set up FFT
  transformHRIRs(gHRIRLength, gConvolutionSize); // convert HRIRs to Hz domain
  getVBAPMatrix();                               // import VBAP speaker gains
  createVectors(gStreams);
  setupOSC();                                    // setup OSC communication
  if(gFixedTrajectory){
    createPairs();                               // create HRTF tournament
    createLocations();
    gInputVolume[1]=0.0;                         // mute second stream
  }
  initFFTProcesses();                            // initialise FFT processing
  if((gFillBuffersTask = Bela_createAuxiliaryTask(&fillBuffers, 89, \
    "fill-buffer")) == 0) return false;          // fill buffers
  reverb->getNodeElevations(gVBAPDefaultElevation);
	reverb->getNodeAzimuths(gVBAPDefaultAzimuth);
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
    if(gCurrentState==kPaused){
      // kill the output
      context->audioOut[n * context->audioOutChannels + 0] = 0.0;
      context->audioOut[n * context->audioOutChannels + 1] = 0.0;
    }

    //if playback has just been stopped:
    else if(gCurrentState==kStopped){
      // kill the output
      context->audioOut[n * context->audioOutChannels + 0] = 0.0;
      context->audioOut[n * context->audioOutChannels + 1] = 0.0;

      reinitialiseAudioStreams();

      // pause playback
      gCurrentState=kPaused;
    }

    // if playback is active, proccess the audio
    else if(gCurrentState==kPlaying){

      //use a defined 10 second trajectory for HRTF selection purposes
      if(gFixedTrajectory && !gTestMode)updatePositions();

      //use a defined test routine for verifying HRTF convolution outputs
      if(gTestMode)applyTestCoords();

      // run the spatialisation algorithm
      spatialiseAudio();

      /* ----------
      CY TEST IMPLEMENTATION
      sampleStream[1]->togglePlayback(1);
      sampleStream[1]->processFrame();
      float in = sampleStream[1]->getSample(0) * gInputVolume[1];
      float reverbOutput[6] = {};
			reverb->process(in, reverbOutput);

      // process and read frames for each sampleStream object into input buffer
      for(int stream=0; stream<gStreams; stream++){
        gInputBuffer[stream][gInputBufferPointer] = reverbOutput[stream];
      }
      ---------- */


      float reverbInput=0.0;


      // process and read frames for each sampleStream object into input buffers
      for(int stream=0; stream<gStreams; stream++){
        sampleStream[stream]->togglePlayback(1);
        sampleStream[stream]->processFrame();
        // feed stream to reverb generator
        reverbInput += sampleStream[stream]->getSample(0) * gInputVolume[stream] * 0.3;
        // add stream to input buffer
        gInputBuffer[stream][gInputBufferPointer] = \
          sampleStream[stream]->getSample(0) * gInputVolume[stream];
      }

      // add reverb output to spare stream
      gInputBuffer[gStreams][gInputBufferPointer] = reverb->scatterMono(reverbInput);

      // copy output buffer L/R to audio output L/R
      for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
        if(channel == 0) {
          context->audioOut[n * context->audioOutChannels + channel] = \
            gOutputBufferL[gOutputBufferReadPointer];
        }
        else if (channel == 1){
          context->audioOut[n * context->audioOutChannels + channel] = \
            gOutputBufferR[gOutputBufferReadPointer];
        }
      }
    }
  }
}


// function to prepare audio streams for playback
void loadAudioFiles(){
  // load a playback .wav file into each stream buffer
  for(int stream=0;stream<gStreams;stream++) {
    std::string number=to_string(stream+1);
    std::string file= "./tracks/track" + number + ".wav";
    const char * id = file.c_str();
    sampleStream[stream] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE);
    gInputVolume[stream]=0.3;
  }
}


void reinitialiseAudioStreams(){
  // stop all streams and set to refill input buffers
  for(int stream=0; stream<gStreams; stream++){
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
  for(int stream=0;stream<gStreams;stream++) {
    if(sampleStream[stream]->bufferNeedsFilled())
    sampleStream[stream]->fillBuffer();
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
  for(int i=0;i<gStreams;i++) {
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
