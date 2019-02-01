/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 *****  Code extended and adapted from RTDSP module overlap add solution  *****
 */

// include files
#include <Bela.h>
#include <cmath>
#include <ne10/NE10.h>			    // neon library
#include <SpatialSceneParams.h> // definition of audio sources and context
#include <SampleStream.h>       // adapted code for streaming/processing audio
#include <ImpulseLoader.h>       // adapted code for loading short audio files
#include <ImpulseData.h>        // struct file to identify impulse responses
#include <VBAPData.h>           // lookup tables for VBAP speaker weightings
#include <StreamGainData.h>     // table for audio track level balancing / muting
#include <TestRoutine.h>        // code for testing defined trajectory
#include <OSC.h>                // OSC interfacing
#include <VectorRotations.h>    // bespoke calculations for vector rotations
#include <FFT.h>                // time and frequency domain transforms


extern int gStreams;        // Number of streams defined on startup
extern int gHeadTracking;   // Head tracking status defined on startup



// global variables for stream playback code
int gStopThreads = 0;
int gTaskStopped = 0;
int gCount = 0;


// instantiate the sampleStream class
SampleStream *sampleStream[NUM_STREAMS];

// instantialte auxiliary task to fill buffers
AuxiliaryTask gFillBuffersTask;

// function to prepare audio streams for playback
void loadAudioFiles(){
  // load a playback .wav file into each stream buffer
  for(int k=0;k<gStreams;k++) {
    std::string number=to_string(k+1);
    std::string file= "track" + number + ".wav";
    const char * id = file.c_str();
    sampleStream[k] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE);
  }
}

// function to fill buffers for maximum number of streams on startup
void fillBuffers(void*) {
  for(int i=0;i<gStreams;i++) {
    if(sampleStream[i]->bufferNeedsFilled())
    sampleStream[i]->fillBuffer();
  }
}



// configure Bela environment for playback
bool setup(BelaContext *context, void *userData)
{
  setupIMU(context->audioSampleRate);

  // set up button pin for calibration, if used
  pinMode(context, 0, buttonPin, INPUT);
  loadImpulse(gHRIRLength);                      // load HRIRs
  loadAudioFiles();                               // load audio files
  prepFFTBuffers();                              // set up FFT
  transformHRIRs(gHRIRLength, gConvolutionSize); // convert HRIRs to Hz domain
  getVBAPMatrix();                               // import VBAP speaker gains
  createVectors(gStreams);
  setupOSC();
  initFFTProcesses();
  if((gFillBuffersTask = Bela_createAuxiliaryTask(&fillBuffers, 89, \
    "fill-buffer")) == 0) return false;
  return true;
}

void render(BelaContext *context, void *userData){
  // check if buffers need filling using low priority auxiliary task
  Bela_scheduleAuxiliaryTask(gFillBuffersTask);
  // process the next audio frame
  for(unsigned int n = 0; n < context->audioFrames; n++) {
    if(gHeadTracking)scheduleIMU();

    // process and read frames for each sampleStream object into input buffer
    for(int i=0; i<gStreams; i++){
      sampleStream[i]->processFrame();
      gInputBuffer[i][gInputBufferPointer] = sampleStream[i]->getSample(0);
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


void cleanup(BelaContext *context, void *userData)
{
  // Clear all input buffers
  for(int i=0;i<gStreams;i++) {
    delete sampleStream[i];
  }
  for(int i=0;i<NUM_SPEAKERS*NUM_HRTFS;i++) {
    NE10_FREE(impulseTimeDomainL[i]);
    NE10_FREE(impulseTimeDomainR[i]);
    NE10_FREE(impulseFrequencyDomainL[i]);
    NE10_FREE(impulseFrequencyDomainR[i]);
  }
  NE10_FREE(signalTimeDomainIn);
  NE10_FREE(signalFrequencyDomain);
  NE10_FREE(signalFrequencyDomainL);
  NE10_FREE(signalFrequencyDomainR);
  NE10_FREE(signalTimeDomainOutL);
  NE10_FREE(signalTimeDomainOutR);
  NE10_FREE(cfg);

  // write test file if tracking not active
  if(!gHeadTracking){
    std::ofstream OutL("testImpL.csv");
    std::ofstream OutR("testImpR.csv");
    for (int i = 0; i < 187; i++) {
      for (int j = 0; j < gConvolutionSize; j++){
        OutL << (float)gDataOutputL[i][j] << ',';
        OutR << (float)gDataOutputR[i][j] << ',';
      }
      OutL << '\n';
      OutR << '\n';
    }
  }
}
