/*
____  _____ _        _
| __ )| ____| |      / \
|  _ \|  _| | |     / _ \
| |_) | |___| |___ / ___ \
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

A project of the Augmented Instruments Laboratory within the
Centre for Digital Music at Queen Mary University of London.
http://www.eecs.qmul.ac.uk/~andrewm

(c) 2016 Augmented Instruments Laboratory: Andrew McPherson,
Astrid Bin, Liam Donovan, Christian Heinrichs, Robert Jack,
Giulio Moro, Laurel Pardue, Victor Zappi. All rights reserved.

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/

// include files
#include <Bela.h>
#include <ne10/NE10.h>					// neon library
#include <cmath>
#include <SampleStream.h>
#include <SampleLoader.h>
#include <VBAPData.h>
#include <ImpulseData.h>

#define NUM_CHANNELS 1    // NUMBER OF CHANNELS IN THE FILE
#define BUFFER_SIZE 32768   // BUFFER SIZE
#define NUM_STREAMS 8
#define NUM_SPEAKERS 8

// instantiate the sampleStream class for the required number of streams
SampleStream *sampleStream[NUM_STREAMS];
float gStreamGains[]={0.195,0.255,0.18,0.135,0.18,0.195,0.195,0.195,0.195};

ImpulseData gImpulseData[NUM_SPEAKERS*2];

// global variables for stream playback
int gStopThreads = 0;
int gTaskStopped = 0;
int gCount = 0;




// FFT buffer variables
float gInputBuffer[NUM_STREAMS][BUFFER_SIZE];
int gInputBufferPointer = 0;
float gVBAPBuffers[NUM_SPEAKERS][BUFFER_SIZE];
int gVBAPBufferPointer = 0;
float gOutputBufferL[BUFFER_SIZE];
float gOutputBufferR[BUFFER_SIZE];
int gOutputBufferReadPointer;
int gOutputBufferWritePointer;
int gFFTInputBufferPointer;
int gFFTOutputBufferPointer;
float *gWindowBuffer;
int gSampleCount = 0;

// These variables used internally in the example:
int gFFTSize = 2048;
int gHopSize = gFFTSize / 4;
float gFFTScaleFactor = 0;


// FFT vars
ne10_fft_cpx_float32_t* impulseTimeDomainL[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseTimeDomainR[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainL[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainR[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* signalTimeDomainIn;
ne10_fft_cpx_float32_t* signalFrequencyDomain;
ne10_fft_cpx_float32_t* signalFrequencyDomainL;
ne10_fft_cpx_float32_t* signalFrequencyDomainR;
ne10_fft_cpx_float32_t* signalTimeDomainOutL;
ne10_fft_cpx_float32_t* signalTimeDomainOutR;
ne10_fft_cfg_float32_t cfg;

// instantialte auxiliary task
AuxiliaryTask gFillBuffersTask;
// Auxiliary task for calculating FFT
AuxiliaryTask gFFTTask;

void process_fft_background(void *);



// function to fill buffers for each declared stream
void fillBuffers(void*) {
  for(int i=0;i<NUM_STREAMS;i++) {
    if(sampleStream[i]->bufferNeedsFilled())
    sampleStream[i]->fillBuffer();
  }
}


// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// Return true on success; returning false halts the program.


// This function handles the FFT processing in this example once the buffer has
// been assembled.

void loadImpulse(){
  // load an impulse .wav file into each stream
  for(int i=0;i<NUM_SPEAKERS;i++) {
    std::string number=to_string(i+1);
    std::string file= "impulse" + number + ".wav";
    const char * id = file.c_str();
    for(int ch=0;ch<2;ch++) {
      int impulseChannel = (i*2)+ch;
      gImpulseData[impulseChannel].sampleLen = getImpulseNumFrames(id);
      gImpulseData[impulseChannel].samples = new float[gFFTSize];
      getImpulseSamples(id,gImpulseData[impulseChannel].samples,ch,0, gImpulseData[impulseChannel].sampleLen);
      rt_printf("Impulse %d = %f\n",impulseChannel, gImpulseData[impulseChannel].samples[511]);
    }

  }
}

void loadStream(){

  // load a playback .wav file into each stream
  for(int k=0;k<NUM_STREAMS;k++) {
    std::string number=to_string(k+1);
    std::string file= "track" + number + ".wav";
    const char * id = file.c_str();
    sampleStream[k] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE);
  }
}

bool setup(BelaContext *context, void *userData)
{
  loadImpulse();
  loadStream();


  // set up FFT
  gFFTScaleFactor = 1.0f / (float)gFFTSize * 1000;
  rt_printf("scale factor: %f\n", gFFTScaleFactor);
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

  // Initialise auxiliary tasks
  if((gFFTTask = Bela_createAuxiliaryTask(&process_fft_background, 90, \
    "fft-calculation")) == 0)
  return false;


  memset(gInputBuffer, 0, BUFFER_SIZE * sizeof(float));
  memset(signalTimeDomainIn, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomain, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomainL, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalFrequencyDomainR, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalTimeDomainOutL, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(signalTimeDomainOutR, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
  memset(gOutputBufferL, 0, BUFFER_SIZE * sizeof(float));
  memset(gOutputBufferR, 0, BUFFER_SIZE * sizeof(float));

  gOutputBufferReadPointer = 0;
  gOutputBufferWritePointer = gHopSize;

  // Allocate the window buffer based on the FFT size
  gWindowBuffer = (float *)malloc(gFFTSize * sizeof(float));
  if(gWindowBuffer == 0)
  	return false;

  // Calculate a Hann window
  for(int n = 0; n < gFFTSize; n++) {
  	gWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFFTSize - 1)));
  }

  /*----------------------*/
  // Generate IR frequency domain values (with both real and imaginary components)
  for (int i = 0; i < NUM_SPEAKERS; i++){
    int impulseL = i*2;
    int impulseR = impulseL+1;
    impulseTimeDomainL[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
      sizeof (ne10_fft_cpx_float32_t));
    impulseTimeDomainR[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
      sizeof (ne10_fft_cpx_float32_t));
    impulseFrequencyDomainL[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
      sizeof (ne10_fft_cpx_float32_t));
    impulseFrequencyDomainR[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * \
      sizeof (ne10_fft_cpx_float32_t));
    for (int n = 0; n < gFFTSize; n++)
    {
      impulseTimeDomainL[i][n].r = (ne10_float32_t) gImpulseData[impulseL].samples[n];
      impulseTimeDomainL[i][n].i = (ne10_float32_t) gImpulseData[impulseL].samples[n];
      impulseTimeDomainR[i][n].r = (ne10_float32_t) gImpulseData[impulseR].samples[n];
      impulseTimeDomainR[i][n].i = (ne10_float32_t) gImpulseData[impulseR].samples[n];
    }
    ne10_fft_c2c_1d_float32_neon(impulseFrequencyDomainL[i], impulseTimeDomainL[i], \
      cfg, 0);
    ne10_fft_c2c_1d_float32_neon(impulseFrequencyDomainR[i], impulseTimeDomainR[i], \
      cfg, 0);
  }
  // Perform the FFT for the IR (left and right signals)

  /*----------------------*/

  // check if buffers need to be filled
  if((gFillBuffersTask = Bela_createAuxiliaryTask(&fillBuffers, 89, \
    "fill-buffer")) == 0)
  return false;

  rt_printf("Test: %f",gVBAPGains4Speakers[4][4]);

  return true;
}

void process_fft()
{
  // Copy buffer into FFT input.
  int pointer = (gFFTInputBufferPointer - gFFTSize + BUFFER_SIZE) % BUFFER_SIZE;
  for(int streams=0; streams<NUM_STREAMS;streams++){
    for(int speakers=0; speakers<NUM_SPEAKERS;speakers++){
      gVBAPBuffers[speakers][pointer]=+ gInputBuffer[streams][pointer] \
        * gVBAPGains8Speakers[streams][speakers];
    }
    pointer++;
    if (pointer >= BUFFER_SIZE)
    pointer = 0;
  }
  
  for(int speakers=0; speakers<NUM_SPEAKERS;speakers++){
    for(int n = 0; n < gFFTSize; n++) {
      signalTimeDomainIn[n].r = (ne10_float32_t) gInputBuffer[speakers][pointer] * gWindowBuffer[n];
      signalTimeDomainIn[n].i = 0.0;
      // Update "pointer" each time and wrap it around to keep it within the
      // circular buffer.
      pointer++;
      if (pointer >= BUFFER_SIZE)
      pointer = 0;
    }

    // Run the FFT
    ne10_fft_c2c_1d_float32_neon (signalFrequencyDomain, signalTimeDomainIn, \
      cfg, 0);

    for(int n=0;n<gFFTSize;n++){
      signalFrequencyDomainL[n].r = signalFrequencyDomain[n].r * \
        impulseFrequencyDomainL[speakers][n].r;
      signalFrequencyDomainL[n].i = signalFrequencyDomain[n].i * \
        impulseFrequencyDomainL[speakers][n].i;
      signalFrequencyDomainR[n].r = signalFrequencyDomain[n].r * \
        impulseFrequencyDomainR[speakers][n].r;
      signalFrequencyDomainR[n].i = signalFrequencyDomain[n].i * \
        impulseFrequencyDomainR[speakers][n].i;
    }

    // Run the inverse FFTs
    ne10_fft_c2c_1d_float32_neon (signalTimeDomainOutL, signalFrequencyDomainL, \
      cfg, 1);
    ne10_fft_c2c_1d_float32_neon (signalTimeDomainOutR, signalFrequencyDomainR, \
        cfg, 1);

    // Copy signalTimeDomainOut into the output buffer.
    int pointer = gFFTOutputBufferPointer;
    for(int n=0; n<gFFTSize; n++) {
      gOutputBufferL[pointer] += signalTimeDomainOutL[n].r * gFFTScaleFactor \
        * gStreamGains[speakers];
      gOutputBufferR[pointer] += signalTimeDomainOutR[n].r * gFFTScaleFactor \
        * gStreamGains[speakers];
      pointer++;
      if(pointer >= BUFFER_SIZE)
      pointer = 0;
    }
  }
}

// Function to process the FFT in a thread at lower priority
void process_fft_background(void *) {
  // TODO: call process_fft() using the pointer locations you saved in render()
  process_fft();
}

void render(BelaContext *context, void *userData){

// run check of buffers
Bela_scheduleAuxiliaryTask(gFillBuffersTask);


// apply processing to each stream
for(unsigned int n = 0; n < context->audioFrames; n++) {
  // fade in/out as necessary for each stream
  for(int i=0;i<NUM_STREAMS;i++) {
    //process frames for each sampleStream object (get samples per channel below)
    sampleStream[i]->processFrame();
  }


    // Prep the "input" to be the sound file played in a loop

    		// -------------------------------------------------------------------
    		// read sample data into input buffer
        for(int i=0; i<NUM_STREAMS; i++){
            gInputBuffer[i][gInputBufferPointer] = sampleStream[i]->getSample(0);
        }


    		// Copy output buffer to output
    		for(int channel = 0; channel < context->audioOutChannels; channel++) {
    			// Get the sample from the output buffer instead of sample buffer
    			if(channel == 0) {
            context->audioOut[n * context->audioOutChannels + channel] = \
              gOutputBufferL[gOutputBufferReadPointer];
          }
          else if (channel == 1){
            context->audioOut[n * context->audioOutChannels + channel] = \
              gOutputBufferR[gOutputBufferReadPointer];
          }

    		}

    		// Clear the output sample in the buffer so it is ready for the next overlap-add
    		gOutputBufferL[gOutputBufferReadPointer] = 0;
        gOutputBufferR[gOutputBufferReadPointer] = 0;

    		// Advance the output buffer pointer exactly the same way as the input buffer pointer
    		gOutputBufferReadPointer++;
    		if(gOutputBufferReadPointer >= BUFFER_SIZE)
    			gOutputBufferReadPointer = 0;

    		// Update write pointer
    		gOutputBufferWritePointer++;
    		if(gOutputBufferWritePointer >= BUFFER_SIZE)
    			gOutputBufferWritePointer = 0;

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
