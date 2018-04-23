/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 *****  Code extended and adapted from RTDSP module overlap add solution  *****
 */

// include files
#include <Bela.h>
#include <cmath>
#include <ne10/NE10.h>			// neon library
#include <SampleStream.h>   // adapted Bela code for streaming/processing audio
#include <SampleLoader.h>   // adapted Bela code for loading short audio files
#include <ImpulseData.h>    // distinct struct file to identify impulse responses
#include <VBAPData.h>       // lookup tables for VBAP speaker weightings
#include <StreamGainData.h> // table for audio track level balancing / muting

#define BUFFER_SIZE 32768   // BUFFER SIZE
#define NUM_CHANNELS 1      // NUMBER OF CHANNELS IN AUDIO STREAMS
#define NUM_STREAMS 10      // MAXIMUM NUMBER OF AUDIO STREAMS
#define NUM_SPEAKERS 8      // MAXIMUM NUMBER OF VIRTUAL SPEAKERS

extern int gSpeakers;       // Number of Speakers chosen by user
extern int gTracks;         // Concurrent tracks chosen by user
extern bool gVoiceMeta;     // Metadata playback on/off chosen by user

// instantiate the sampleStream class
SampleStream *sampleStream[NUM_STREAMS];

// instantiate binaural impulse response data buffers (left and right channels)
ImpulseData gImpulseData[NUM_SPEAKERS*2];

// global variables for stream playback code
int gStopThreads = 0;
int gTaskStopped = 0;
int gCount = 0;

// FFT overlap/add buffers and variables
float gInputBuffer[NUM_STREAMS][BUFFER_SIZE];
int gInputBufferPointer = 0;
float gOutputBufferL[BUFFER_SIZE];
float gOutputBufferR[BUFFER_SIZE];
int gOutputBufferReadPointer;
int gOutputBufferWritePointer;
int gFFTInputBufferPointer;
int gFFTOutputBufferPointer;
float *gWindowBuffer;
int gSampleCount = 0;
int gFFTSize = 2048;
int gHopSize = gFFTSize / 4;
float gFFTScaleFactor = 0;

// additional buffers for summing VBAP feeds to each virtual speaker
float gVBAPBuffers[NUM_SPEAKERS][BUFFER_SIZE];

// buffers and configuration for Neon FFT processing
ne10_fft_cpx_float32_t* impulseTimeDomainL[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseTimeDomainR[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainL[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainR[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* signalTimeDomainIn;
ne10_fft_cpx_float32_t* signalFrequencyDomain;  // (buffer to calculate L/R)
ne10_fft_cpx_float32_t* signalFrequencyDomainL;
ne10_fft_cpx_float32_t* signalFrequencyDomainR;
ne10_fft_cpx_float32_t* signalTimeDomainOutL;
ne10_fft_cpx_float32_t* signalTimeDomainOutR;
ne10_fft_cfg_float32_t cfg;

// instantialte auxiliary task to fill buffers
AuxiliaryTask gFillBuffersTask;
// instatiate auxiliary task to calculate FFTs
AuxiliaryTask gFFTTask;

//declare process_fft_backround method
void process_fft_background(void *);


// function to load HRIRs for either 4 or 8 virtual speakers declared on startup
void loadImpulse(){
  // load impulse .wav files from the relevant directory
  for(int i=0;i<gSpeakers;i++) {
    std::string speakers=to_string(gSpeakers);
    std::string number=to_string(i+1);
    std::string file= "./" + speakers + "speakers/impulse" + number + ".wav";
    const char * id = file.c_str();
    //determine the HRIR lengths (in samples) and populate the buffers
    for(int ch=0;ch<2;ch++) {
      int impulseChannel = (i*2)+ch;
      gImpulseData[impulseChannel].sampleLen = getImpulseNumFrames(id);
      gImpulseData[impulseChannel].samples = new float[gFFTSize];
      getImpulseSamples(id,gImpulseData[impulseChannel].samples,ch,0, \
        gImpulseData[impulseChannel].sampleLen);
    }

  }
}


// function to prepare maximum number of audio streams for playback
void loadStream(){
  // load a playback .wav file into each stream buffer
  for(int k=0;k<NUM_STREAMS;k++) {
    std::string number=to_string(k+1);
    std::string file= "track" + number + ".wav";
    const char * id = file.c_str();
    sampleStream[k] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE);
  }
}


// funciton to prepare FFT buffers for input signals and allocate memory
void prepFFT(){
  gFFTScaleFactor = 1.0f / (float)gFFTSize * 1000;
  signalTimeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalFrequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalFrequencyDomainL = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalFrequencyDomainR = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalTimeDomainOutL = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalTimeDomainOutR = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
    sizeof (ne10_fft_cpx_float32_t));
  cfg = ne10_fft_alloc_c2c_float32_neon (gFFTSize);
  memset(gInputBuffer, 0, BUFFER_SIZE * sizeof(float));
  memset(signalTimeDomainIn, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomain, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomainL, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomainR, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalTimeDomainOutL, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalTimeDomainOutR, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(gOutputBufferL, 0, BUFFER_SIZE * sizeof(float));
  memset(gOutputBufferR, 0, BUFFER_SIZE * sizeof(float));
}


// function to generate IR frequency domain values
void transformHRIRs(){
  // allocate memory to HRIR FFT buffers (left and right)
  for (int i = 0; i < gSpeakers; i++){
    int impulseL = i*2;
    int impulseR = impulseL+1;
    impulseTimeDomainL[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize \
      * sizeof (ne10_fft_cpx_float32_t));
    impulseTimeDomainR[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize \
      * sizeof (ne10_fft_cpx_float32_t));
    impulseFrequencyDomainL[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize \
       * sizeof (ne10_fft_cpx_float32_t));
    impulseFrequencyDomainR[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize \
      * sizeof (ne10_fft_cpx_float32_t));
    // assign real and imaginary components to each HRIR FFT buffer
    for (int n = 0; n < gFFTSize; n++)
    {
      impulseTimeDomainL[i][n].r = (ne10_float32_t) gImpulseData[impulseL].samples[n];
      impulseTimeDomainL[i][n].i = (ne10_float32_t) gImpulseData[impulseL].samples[n];
      impulseTimeDomainR[i][n].r = (ne10_float32_t) gImpulseData[impulseR].samples[n];
      impulseTimeDomainR[i][n].i = (ne10_float32_t) gImpulseData[impulseR].samples[n];
    }
    // transform to frequency domain (left and right)
    ne10_fft_c2c_1d_float32_neon(impulseFrequencyDomainL[i], impulseTimeDomainL[i], \
      cfg, 0);
    ne10_fft_c2c_1d_float32_neon(impulseFrequencyDomainR[i], impulseTimeDomainR[i], \
      cfg, 0);
  }
}


// function to fill buffers for maximum number of streams on startup
void fillBuffers(void*) {
  for(int i=0;i<NUM_STREAMS;i++) {
    if(sampleStream[i]->bufferNeedsFilled())
    sampleStream[i]->fillBuffer();
  }
}


// configure Bela environment for playback
bool setup(BelaContext *context, void *userData)
{
  // print user command line selections
  rt_printf("Speakers: %d\t Tracks: %d\t Voice Metadata: %d\t", \
    gSpeakers,gTracks,gVoiceMeta);
  loadImpulse();    // load HRIRs
  loadStream();     // load audio streams
  prepFFT();        // set up FFT
  transformHRIRs(); // convert HRIRs to frequency domain

  // initialise FFT auxiliary task
  if((gFFTTask = Bela_createAuxiliaryTask(&process_fft_background, 90, \
    "fft-calculation")) == 0)
  return false;

  // initialise main output buffer pointers
  gOutputBufferReadPointer = 0;
  gOutputBufferWritePointer = gHopSize;

  // allocate the window buffer based on the FFT size
  gWindowBuffer = (float *)malloc(gFFTSize * sizeof(float));
  if(gWindowBuffer == 0)
  	return false;

  // calculate a Hann window for overlap/add processing
  for(int n = 0; n < gFFTSize; n++) {
  	gWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFFTSize - 1)));
  }

  // silence voice metadata streams if switched off by user input
  if(gVoiceMeta==0){
    for(int i=NUM_STREAMS/2;i<NUM_STREAMS;i++){
      gStreamGains[gTracks-1][i]=0.0;
    }
  }

  // initialise streaming auxiliary task
  if((gFillBuffersTask = Bela_createAuxiliaryTask(&fillBuffers, 89, \
    "fill-buffer")) == 0)
  return false;

  return true;
}


void process_fft()
{
  // create the binaural signal for each speaker and sum to the L/R outputs
  for(int speaker=0; speaker<gSpeakers;speaker++){
    // copy individual streams into FFT buffer
    int pointer = (gFFTInputBufferPointer - gFFTSize + BUFFER_SIZE) % BUFFER_SIZE;
    for(int n = 0; n < gFFTSize; n++) {
      signalTimeDomainIn[n].r = 0.0;    // clear the FFT input buffers first
      signalTimeDomainIn[n].i = 0.0;
      // Add the value for each stream, taking into account VBAP speaker and
      // track gain weightings.
      for(int stream=0; stream<NUM_STREAMS;stream++){
        if(gSpeakers==4){               // check speakers numbers for gain lookup
          signalTimeDomainIn[n].r += (ne10_float32_t) gInputBuffer[stream][pointer] \
            * gStreamGains[gTracks-1][stream] * gVBAPGains4Speakers[stream][speaker];
        }
        else{
          signalTimeDomainIn[n].r += (ne10_float32_t) gInputBuffer[stream][pointer] \
            * gStreamGains[gTracks-1][stream] * gVBAPGains8Speakers[stream][speaker];
        }
      }
      // Update "pointer" each time and wrap it around to keep it within the
      // circular buffer.
      signalTimeDomainIn[n].r *= gWindowBuffer[n];
      pointer++;
      if (pointer >= BUFFER_SIZE)
      pointer = 0;
    }
    // convert speaker feed to frequency domian
    ne10_fft_c2c_1d_float32_neon (signalFrequencyDomain, signalTimeDomainIn, \
      cfg, 0);

    // convolve speaker feed to binaural signal with relevant HRIR
    for(int n=0;n<gFFTSize;n++){
      signalFrequencyDomainL[n].r = signalFrequencyDomain[n].r * \
        impulseFrequencyDomainL[speaker][n].r;
      signalFrequencyDomainL[n].i = signalFrequencyDomain[n].i * \
        impulseFrequencyDomainL[speaker][n].i;
      signalFrequencyDomainR[n].r = signalFrequencyDomain[n].r * \
        impulseFrequencyDomainR[speaker][n].r;
      signalFrequencyDomainR[n].i = signalFrequencyDomain[n].i * \
        impulseFrequencyDomainR[speaker][n].i;
    }

    // convert result back to time domain (left and right)
    ne10_fft_c2c_1d_float32_neon (signalTimeDomainOutL, signalFrequencyDomainL, \
      cfg, 1);
    ne10_fft_c2c_1d_float32_neon (signalTimeDomainOutR, signalFrequencyDomainR, \
        cfg, 1);

    // add results to left and outputs
    pointer = gFFTOutputBufferPointer;
    for(int n=0; n<gFFTSize; n++) {
      gOutputBufferL[pointer] += signalTimeDomainOutL[n].r * gFFTScaleFactor;
      gOutputBufferR[pointer] += signalTimeDomainOutR[n].r * gFFTScaleFactor;
      pointer++;
      if(pointer >= BUFFER_SIZE)
      pointer = 0;
    }
  }
}


// Function to process the FFT in a thread at lower priority
void process_fft_background(void *) {
  process_fft();
}


void render(BelaContext *context, void *userData){
  // check if buffers need filling using low priority auxiliary task
  Bela_scheduleAuxiliaryTask(gFillBuffersTask);

  // process the next audio frame
  for(unsigned int n = 0; n < context->audioFrames; n++) {

		// process and read frames for each sampleStream object into input buffer
    for(int i=0; i<NUM_STREAMS; i++){
      sampleStream[i]->processFrame();
      gInputBuffer[i][gInputBufferPointer] = sampleStream[i]->getSample(0);
    }
		// copy output buffer L/R to audio output L/R
		for(int channel = 0; channel < context->audioOutChannels; channel++) {
			if(channel == 0) {
        context->audioOut[n * context->audioOutChannels + channel] = \
          gOutputBufferL[gOutputBufferReadPointer];
      }
      else if (channel == 1){
        context->audioOut[n * context->audioOutChannels + channel] = \
          gOutputBufferR[gOutputBufferReadPointer];
      }
	  }

    // clear the output samples in the buffers so they're ready for the next ola
    gOutputBufferL[gOutputBufferReadPointer] = 0;
    gOutputBufferR[gOutputBufferReadPointer] = 0;

    // advance the output buffer pointer
    gOutputBufferReadPointer++;
    if(gOutputBufferReadPointer >= BUFFER_SIZE)
    	gOutputBufferReadPointer = 0;

    // advance the write pointer
    gOutputBufferWritePointer++;
    if(gOutputBufferWritePointer >= BUFFER_SIZE)
    	gOutputBufferWritePointer = 0;

    // advance the read pointer
    gInputBufferPointer++;
    if(gInputBufferPointer >= BUFFER_SIZE)
    	gInputBufferPointer = 0;

    // Increment gSampleCount and check if it reaches the hop size.
    // If so, reset buffer pointers, run the FFT and reset gSampleCount.
    gSampleCount++;
    if(gSampleCount >= gHopSize) {
    	gFFTInputBufferPointer = gInputBufferPointer;
    	gFFTOutputBufferPointer = gOutputBufferWritePointer;
      Bela_scheduleAuxiliaryTask(gFFTTask);
    	gSampleCount = 0;
    }
  }
}


// Clear all input buffers
void cleanup(BelaContext *context, void *userData)
{
  for(int i=0;i<NUM_STREAMS;i++) {
    delete sampleStream[i];
  }
  for(int i=0;i<NUM_SPEAKERS;i++) {
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
}
