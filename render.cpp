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
#include <TestRoutine.h>    // code for testing defined trajectory
#include <OSC.h>            // OSC interfacing
#include <VectorRotations.h>// bespoke calculations for vector rotations

#define BUFFER_SIZE 1536    // BUFFER SIZE
#define NUM_CHANNELS 1      // NUMBER OF CHANNELS IN AUDIO STREAMS
#define NUM_STREAMS 20      // MAXIMUM NUMBER OF AUDIO STREAMS
#define NUM_SPEAKERS 8      // NUMBER OF VIRTUAL SPEAKERS
#define NUM_HRTFS 7         // NUMBER OF SELECTABLE HRTF SETS

extern int gStreams;        // Number of streams defined on startup
extern int gTracks;         // Concurrent tracks chosen by user
extern int gHRTF;           // HRTF set chosen by user
extern bool gVoiceMeta;     // Metadata playback on/off chosen by user
extern bool gCalibrate;     // Headset calibration
extern int buttonPin;       // Pin used for hardware button calibration
extern int gVBAPDefaultAzimuth[10];   // Azimuth array
extern int gVBAPDefaultElevation[10]; // Elevation array

// instantiate the sampleStream class
SampleStream *sampleStream[NUM_STREAMS];

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
int gHRIRLength = 512;
int gFFTSize = 1024;
int gConvolutionSize = gFFTSize+gHRIRLength;
int gHopSize = gFFTSize/2;
float gFFTScaleFactor = 0;

// buffers and configuration for Neon FFT processing
ne10_fft_cpx_float32_t* signalTimeDomainIn;
ne10_fft_cpx_float32_t* signalFrequencyDomain;  // (buffer to calculate L/R)
ne10_fft_cpx_float32_t* signalFrequencyDomainL;
ne10_fft_cpx_float32_t* signalFrequencyDomainR;
ne10_fft_cpx_float32_t* signalTimeDomainOutL;
ne10_fft_cpx_float32_t* signalTimeDomainOutR;
extern ne10_fft_cfg_float32_t cfg;

// instantialte auxiliary task to fill buffers
AuxiliaryTask gFillBuffersTask;
// instatiate auxiliary task to calculate FFTs
AuxiliaryTask gFFTTask;


//declare process_fft_backround method
void process_fft_background(void *);

// function to prepare maximum number of audio streams for playback
void loadStream(){
  // load a playback .wav file into each stream buffer
  for(int k=0;k<gStreams;k++) {
    std::string number=to_string(k+1);
    std::string file= "track" + number + ".wav";
    const char * id = file.c_str();
    sampleStream[k] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE);
  }
}

// funciton to prepare FFT buffers for input signals and allocate memory
void prepFFT(){
  gFFTScaleFactor = 1.0f / (float)gConvolutionSize * 1000;
  signalTimeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalFrequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalFrequencyDomainL = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalFrequencyDomainR = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalTimeDomainOutL = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
    sizeof (ne10_fft_cpx_float32_t));
  signalTimeDomainOutR = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
    sizeof (ne10_fft_cpx_float32_t));
  cfg = ne10_fft_alloc_c2c_float32_neon (gConvolutionSize);
  memset(gInputBuffer, 0, BUFFER_SIZE * sizeof(float));
  memset(signalTimeDomainIn, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomain, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomainL, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomainR, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalTimeDomainOutL, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalTimeDomainOutR, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
  memset(gOutputBufferL, 0, BUFFER_SIZE * sizeof(float));
  memset(gOutputBufferR, 0, BUFFER_SIZE * sizeof(float));
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

  // set up button pin
  pinMode(context, 0, buttonPin, INPUT);

  // print user command line selections
  rt_printf("Streams: %d\t Tracks: %d\t Voice Metadata: %d\n", \
    gStreams,gTracks,gVoiceMeta);
  loadImpulse(gHRIRLength);    // load HRIRs
  loadStream();     // load audio streams
  prepFFT();        // set up FFT
  transformHRIRs(gHRIRLength, gConvolutionSize); // convert HRIRs to frequency domain
  getVBAPMatrix();  // import VBAP speaker gain data
  createVectors(gStreams);
  setupOSC();

  // initialise FFT auxiliary task
  if((gFFTTask = Bela_createAuxiliaryTask(&process_fft_background, 90, \
    "fft-calculation")) == 0) return false;

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


  // initialise streaming auxiliary task
  if((gFillBuffersTask = Bela_createAuxiliaryTask(&fillBuffers, 89, \
    "fill-buffer")) == 0) return false;

  return true;
}


void process_fft()
{
  // clear FFT buffers
  for(int n=0;n<gConvolutionSize;n++){
    signalFrequencyDomainL[n].r=0.0;
    signalFrequencyDomainL[n].i=0.0;
    signalFrequencyDomainR[n].r=0.0;
    signalFrequencyDomainR[n].i=0.0;
  }
  int pointer;
  // create the binaural signal for each speaker and sum to the L/R outputs
  for(int speaker=0; speaker<NUM_SPEAKERS;speaker++){
    // copy individual streams into FFT buffer
    pointer = (gFFTInputBufferPointer - gFFTSize + BUFFER_SIZE) % BUFFER_SIZE;
    for(int n = 0; n < gConvolutionSize; n++) {
      signalTimeDomainIn[n].r = 0.0;    // clear the FFT input buffers first
      signalTimeDomainIn[n].i = 0.0;
      // Add the value for each stream, taking into account VBAP speaker and
      // track gain weightings.
      if(n<gFFTSize){
        for(int stream=0; stream<gStreams;stream++){
          signalTimeDomainIn[n].r += (ne10_float32_t) \
          gInputBuffer[stream][pointer] \
          * gStreamGains[gTracks-1][gVoiceMeta][stream] \
          * gVBAPGains[gVBAPUpdatePositions[stream]][speaker] * gWindowBuffer[n];
        }
        // Update "pointer" each time and wrap it around to keep it within the
        // circular buffer.
        pointer++;
        if (pointer >= BUFFER_SIZE)
        pointer = 0;
      }
    }
    // convert speaker feed to frequency domian
    ne10_fft_c2c_1d_float32_neon (signalFrequencyDomain, signalTimeDomainIn, \
      cfg, 0);
    // convolve speaker feed to binaural signal with relevant HRIR
    for(int n=0;n<gConvolutionSize;n++){
      // left real
      signalFrequencyDomainL[n].r += (signalFrequencyDomain[n].r * \
        impulseFrequencyDomainL[speaker+(gHRTF*NUM_SPEAKERS)][n].r) \
        - (signalFrequencyDomain[n].i * \
          impulseFrequencyDomainL[speaker+(gHRTF*NUM_SPEAKERS)][n].i);
      // left imaginary
      signalFrequencyDomainL[n].i += (signalFrequencyDomain[n].i * \
        impulseFrequencyDomainL[speaker+(gHRTF*NUM_SPEAKERS)][n].r) \
        + (signalFrequencyDomain[n].r * \
          impulseFrequencyDomainL[speaker+(gHRTF*NUM_SPEAKERS)][n].i);
      // right real
      signalFrequencyDomainR[n].r += (signalFrequencyDomain[n].r * \
        impulseFrequencyDomainR[speaker+(gHRTF*NUM_SPEAKERS)][n].r) \
        - (signalFrequencyDomain[n].i * \
          impulseFrequencyDomainR[speaker+(gHRTF*NUM_SPEAKERS)][n].i);
      // right imaginary
      signalFrequencyDomainR[n].i += (signalFrequencyDomain[n].i * \
        impulseFrequencyDomainR[speaker+(gHRTF*NUM_SPEAKERS)][n].r) \
        + (signalFrequencyDomain[n].r * \
          impulseFrequencyDomainR[speaker+(gHRTF*NUM_SPEAKERS)][n].i);
    }
  }
  // convert results back to time domain (left and right)
  ne10_fft_c2c_1d_float32_neon (signalTimeDomainOutL, signalFrequencyDomainL, \
    cfg, 1);
  ne10_fft_c2c_1d_float32_neon (signalTimeDomainOutR, signalFrequencyDomainR, \
      cfg, 1);
  // add results to left and output buffers
  pointer = gFFTOutputBufferPointer;
  for(int n=0; n<gConvolutionSize; n++) {
    gOutputBufferL[pointer] += signalTimeDomainOutL[n].r * gFFTScaleFactor;
    gOutputBufferR[pointer] += signalTimeDomainOutR[n].r * gFFTScaleFactor;
    pointer++;
    if(pointer >= BUFFER_SIZE)
    pointer = 0;
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
    scheduleIMU();

		// process and read frames for each sampleStream object into input buffer
    for(int i=0; i<gStreams; i++){
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


    /*--- SCRIPT TO ENABLE TEST MODE ---
    gVBAPUpdatePositions[0]=((gTestElevation+90)*361)+gTestAzimuth+180;
    writeOutput(gOutputBufferL[gOutputBufferReadPointer], \
      gOutputBufferR[gOutputBufferReadPointer]);
    --- ---*/

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
      rotateVectors(gStreams);
      // calcuate the rotated position for each stream
      for(unsigned int i=0; i < gStreams; i++){
        gVBAPUpdatePositions[i]=((gVBAPUpdateElevation[i]+90)*361)+gVBAPUpdateAzimuth[i]+180;
      }
      Bela_scheduleAuxiliaryTask(gFFTTask);
    	gSampleCount = 0;
    }


  }
}


// Clear all input buffers
void cleanup(BelaContext *context, void *userData)
{
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
