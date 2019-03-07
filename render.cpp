/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 */

// include files
#include <Bela.h>
#include <cmath>
#include <SpatialSceneParams.h> // definition of audio sources and context
#include <ne10/NE10.h>			    // neon library
#include <SampleStream.h>       // adapted code for streaming/processing audio
#include <ImpulseLoader.h>      // code for loading HRTF IR files
#include <ImpulseData.h>        // struct file to store IR data
#include <VBAPData.h>           // lookup tables for VBAP speaker weightings
#include <TestRoutine.h>        // code for testing defined trajectory
#include <OSC.h>                // OSC interfacing
#include <VectorRotations.h>    // bespoke code for point source vector rotation
#include <Spatialisation.h>     // spatialisation engine

// user controlled variables from main.cpp
extern int gStreams;
extern bool gFixedTrajectory;
extern bool gTestMode;

// volume level variables for individual streams
float gInputVolume[NUM_STREAMS]={};

// instantiate the sampleStream class for each stream
SampleStream *sampleStream[NUM_STREAMS];

// instantialte auxiliary task to fill buffers
AuxiliaryTask gFillBuffersTask;

// helper functions for this code
void loadAudioFiles();
void fillBuffers(void*);
void applyTestCoords();


// configure Bela environment for playback
bool setup(BelaContext *context, void *userData)
{
  setupIMU(context->audioSampleRate);
  // set up button pin for calibration, if used
  pinMode(context, 0, buttonPin, INPUT);
  loadImpulse(gHRIRLength);                      // load HRIRs
  loadAudioFiles();                              // load audio files
  prepFFTBuffers();                              // set up FFT
  transformHRIRs(gHRIRLength, gConvolutionSize); // convert HRIRs to Hz domain
  getVBAPMatrix();                               // import VBAP speaker gains
  setupOSC();                                    // setup OSC communication
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

    //use a defined 10 second trajectory for HRTF selection purposes
    if(gFixedTrajectory && !gTestMode)updatePositions();

    //use a defined test routine for verifying HRTF convolution outputs
    if(gTestMode)applyTestCoords();

    // process and read frames for each sampleStream object into input buffer
    for(int stream=0; stream<gStreams; stream++){
      sampleStream[stream]->processFrame();
      gInputBuffer[stream][gInputBufferPointer] = sampleStream[stream]->getSample(0) \
        * gInputVolume[stream];
    }

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
    spatialiseAudio();
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
    gInputVolume[stream]=0.4;
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
