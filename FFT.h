#ifndef FFT_H_
#define FFT_H_

#include <SpatialSceneParams.h> // definition of audio sources and context
#include <SampleStream.h>

extern int gStreams;        // Number of streams defined on startup
extern int gHeadTracking;
//extern SampleStream *sampleStream[NUM_STREAMS];

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


// instatiate auxiliary task to calculate FFTs
AuxiliaryTask gFFTTask;

//declare process_fft_backround method
void process_fft_background(void *);

// funciton to prepare FFT buffers for input signals and allocate memory
void prepFFTBuffers(){
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

bool initFFTProcesses(){
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
      // Add the value for each stream, taking into account VBAP speaker gains.
      if(n<gFFTSize){
        for(int stream=0; stream<gStreams;stream++){
          signalTimeDomainIn[n].r += (ne10_float32_t) \
          gInputBuffer[stream][pointer] * 0.5f \
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

void spatialiseAudio(){

  // script to run routine for unit impulse output test with fixed trajectory
  if(!gHeadTracking){
    gVBAPUpdatePositions[0]=((gTestElevation+90)*361)+gTestAzimuth+180;
    writeOutput(gOutputBufferL[gOutputBufferReadPointer], \
      gOutputBufferR[gOutputBufferReadPointer]);
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
    rotateVectors(gStreams);
    // calcuate the rotated position for each stream
    //for(unsigned int i=0; i < gStreams; i++){

    //}
    Bela_scheduleAuxiliaryTask(gFFTTask);
    gSampleCount = 0;
  }
}

#endif /* FFT_H_ */
