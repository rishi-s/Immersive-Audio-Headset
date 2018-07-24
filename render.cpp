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
#include <Scope.h>

/*----------*/
/*----------*/
/* IMU #includes*/
#include <rtdk.h>
#include "Bela_BNO055.h"
/*----------*/
/*----------*/

#define BUFFER_SIZE 4096   // BUFFER SIZE
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
int gHopSize = gFFTSize/4;
float gFFTScaleFactor = 0;


// BECKY - ADD AZIMUTHS HERE: range -180 (anti-clockwise) to 180 (clockwise)
int gVBAPDefaultAzimuth[NUM_STREAMS]{-144,-72,0,72,144,-144,-72,0,72,144};

// BECKY - ADD ELEVATIONS HERE: -90 (down) to 90 (up)
int gVBAPDefaultElevation[NUM_STREAMS]={-10,-10,-10,-10,-10,30,30,30,30,30};

//Rotation variables
float gVBAPDefaultVector[NUM_STREAMS][3];
float gVBAPRotatedVector[NUM_STREAMS][3];
int gVBAPUpdatePositions[NUM_STREAMS]={0};
int gVBAPUpdateAzimuth[NUM_STREAMS]={0};
int gVBAPUpdateElevation[NUM_STREAMS]={0};
int gVBAPTracking[3]={0};

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

// instantiate the scope
Scope scope;

//declare process_fft_backround method
void process_fft_background(void *);

/*----------*/
/*----------*/
/*IMU #variables*/

// Change this to change how often the BNO055 IMU is read (in Hz)
int readInterval = 100;

I2C_BNO055 bno; // IMU sensor object
int buttonPin = P2_02; // calibration button pin
int lastButtonValue = 0; // using a pulldown resistor

// Quaternions and Vectors
imu::Quaternion gCal, gCalLeft, gCalRight, gIdleConj = {1, 0, 0, 0};
imu::Quaternion qGravIdle, qGravCal, quat, steering, qRaw;

imu::Vector<3> gRaw;
imu::Vector<3> gGravIdle, gGravCal;
imu::Vector<3> ypr; //yaw pitch and roll angles


int calibrationState = 0; // state machine variable for calibration
int setForward = 0; // flag for setting forward orientation

// variables handling threading
AuxiliaryTask i2cTask;		// Auxiliary task to read I2C
AuxiliaryTask gravityNeutralTask;		// Auxiliary task to read gravity from I2C
AuxiliaryTask gravityDownTask;		// Auxiliary task to read gravity from I2C

int readCount = 0;			// How long until we read again...
int readIntervalSamples = 0; // How many samples between reads

int printThrottle = 0; // used to limit printing frequency

// function declarations
void readIMU(void*);
void getNeutralGravity(void*);
void getDownGravity(void*);
void calibrate();
void resetOrientation();
/*----------*/
/*----------*/


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
      rt_printf("Length %d = %d\n",impulseChannel, gImpulseData[impulseChannel].sampleLen);
      rt_printf("Impulse %d = %f\n",impulseChannel, gImpulseData[impulseChannel].samples[0]);
      rt_printf("Impulse %d = %f\n",impulseChannel, gImpulseData[impulseChannel].samples[511]);
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
    // zero pad
    for (int n = 0; n < gFFTSize; n++)
    {
      impulseTimeDomainL[i][n].r = 0;
      impulseTimeDomainL[i][n].i = 0;
      impulseTimeDomainR[i][n].r = 0;
      impulseTimeDomainR[i][n].i = 0;
    }
    //impulseTimeDomainL[i][0].r = (ne10_float32_t) 0.5;
    //impulseTimeDomainR[i][0].r = (ne10_float32_t) 0.5;
    // assign real and imaginary components to each HRIR FFT buffer
    for (int n = 0; n < gFFTSize/2; n++)
    {
      impulseTimeDomainL[i][n].r = (ne10_float32_t) gImpulseData[impulseL].samples[n];
      impulseTimeDomainR[i][n].r = (ne10_float32_t) gImpulseData[impulseR].samples[n];
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

// function to convert input stream point source locations to 3D vectors
void createVectors(){
  for(int i=0; i<NUM_STREAMS;i++){
    float aziRad=gVBAPDefaultAzimuth[i]*M_PI/180;
    float eleRad=gVBAPDefaultElevation[i]*M_PI/180;
   gVBAPDefaultVector[i][0]=cos(eleRad)*cos(aziRad);
   gVBAPDefaultVector[i][1]=cos(eleRad)*sin(aziRad);
   gVBAPDefaultVector[i][2]=sin(eleRad);
   rt_printf("\nSource %d – X: %f\t Y: %f\t Z: %f\n", i, \
    gVBAPDefaultVector[i][0],gVBAPDefaultVector[i][1],gVBAPDefaultVector[i][2]);
 }
}

void rotateVectors(){
  for(int i=0; i<NUM_STREAMS;i++){
    float yawRot[3]={0};
    float yawSin = sin(ypr[0]);
    float yawCos = cos(ypr[0]);
    yawRot[0] = yawCos*gVBAPDefaultVector[i][0] + -yawSin*gVBAPDefaultVector[i][1];
    yawRot[1] = yawSin*gVBAPDefaultVector[i][0] + yawCos*gVBAPDefaultVector[i][1];
    yawRot[2] = gVBAPDefaultVector[i][2];
    float pitchRot[3]={0};
    float pitchSin = sin(-ypr[1]);
    float pitchCos = cos(-ypr[1]);
    pitchRot[0] = pitchCos*yawRot[0] + pitchSin*yawRot[2];
    pitchRot[1] = yawRot[1];
    pitchRot[2] = -pitchSin*yawRot[0] + pitchCos*yawRot[2];
    float rollRot[3]={0};
    float rollSin = sin(ypr[2]);
    float rollCos = cos(ypr[2]);
    rollRot[0] = pitchRot[0];
    rollRot[1] = rollCos*pitchRot[1] + -rollSin*pitchRot[2];
    rollRot[2] = rollSin*pitchRot[1] + rollCos*pitchRot[2];
    gVBAPUpdateAzimuth[i]=(int)roundf(atan2(rollRot[1],rollRot[0])*180/M_PI);
    gVBAPUpdateElevation[i]=(int)roundf(asin(rollRot[2]/(sqrt(pow(rollRot[0],2) \
      +pow(rollRot[1],2)+pow(rollRot[2],2))))*180/M_PI);
  }
  rt_printf("Azimuth %d - Elevation %d\n",gVBAPUpdateAzimuth[0],gVBAPUpdateElevation[0]);
}

// configure Bela environment for playback
bool setup(BelaContext *context, void *userData)
{
  /*----------*/
  /*----------*/
  /*IMU #setup routine*/
  if(!bno.begin(2)) {
    rt_printf("Error initialising BNO055\n");
    return false;
  }

  rt_printf("Initialised BNO055\n");

  // use external crystal for better accuracy
    bno.setExtCrystalUse(true);

  // get the system status of the sensor to make sure everything is ok
  uint8_t sysStatus, selfTest, sysError;
    bno.getSystemStatus(&sysStatus, &selfTest, &sysError);
  rt_printf("System Status: %d (0 is Idle)   Self Test: %d (15 is all good)   System Error: %d (0 is no error)\n", sysStatus, selfTest, sysError);

  // set sensor reading in a separate thread
  // so it doesn't interfere with the audio processing
  i2cTask = Bela_createAuxiliaryTask(&readIMU, 5, "bela-bno");
  readIntervalSamples = context->audioSampleRate / readInterval;

  gravityNeutralTask = Bela_createAuxiliaryTask(&getNeutralGravity, 5, "bela-neu-gravity");
  gravityDownTask = Bela_createAuxiliaryTask(&getDownGravity, 5, "bela-down-gravity");

  // set up button pin
  pinMode(context, 0, buttonPin, INPUT);

  /*----------*/
  /*----------*/

  // print user command line selections
  rt_printf("Speakers: %d\t Tracks: %d\t Voice Metadata: %d\n", \
    gSpeakers,gTracks,gVoiceMeta);
  loadImpulse();    // load HRIRs
  loadStream();     // load audio streams
  prepFFT();        // set up FFT
  transformHRIRs(); // convert HRIRs to frequency domain
  getVBAPMatrix(); // import VBAP speaker gain data
  createVectors();

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

  // tell the scope how many channels and the sample rate
  scope.setup(2, context->audioSampleRate);

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
        signalTimeDomainIn[n].r += (ne10_float32_t) gInputBuffer[stream][pointer] \
           * gStreamGains[gTracks-1][stream] * gVBAPGains[gVBAPUpdatePositions[stream]][speaker];
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
      signalFrequencyDomainL[n].r = (signalFrequencyDomain[n].r * \
        impulseFrequencyDomainL[speaker][n].r) \
        - (signalFrequencyDomain[n].i * impulseFrequencyDomainL[speaker][n].i);
      signalFrequencyDomainL[n].i = (signalFrequencyDomain[n].i * \
        impulseFrequencyDomainL[speaker][n].r) \
        + (signalFrequencyDomain[n].r * impulseFrequencyDomainL[speaker][n].i);
      signalFrequencyDomainR[n].r = (signalFrequencyDomain[n].r * \
        impulseFrequencyDomainR[speaker][n].r) \
        - (signalFrequencyDomain[n].i * impulseFrequencyDomainR[speaker][n].i);
      signalFrequencyDomainR[n].i = (signalFrequencyDomain[n].i * \
        impulseFrequencyDomainR[speaker][n].r) \
        + (signalFrequencyDomain[n].r * impulseFrequencyDomainR[speaker][n].i);
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


    /*----------*/
    /*----------*/
    /*IMU #setup routine*/

    // this schedules the imu sensor readings
    if(++readCount >= readIntervalSamples) {
      readCount = 0;
      Bela_scheduleAuxiliaryTask(i2cTask);
    }

    // print IMU values, but not every sample
    printThrottle++;
    if(printThrottle >= 4100){
      //rt_printf("Tracker Value: %d %d %d \n",gVBAPTracking[0],gVBAPTracking[1],gVBAPTracking[2]); //print horizontal head-track value
      rt_printf("%f %f %f\n", ypr[0], ypr[1], ypr[2]);
      //rt_printf("Positions Update: %d %d\n",gVBAPUpdatePositions[0],gVBAPUpdatePositions[9]); //print horizontal head-track value
      imu::Vector<3> qForward = gIdleConj.toEuler();
      printThrottle = 0;
    }

    //read the value of the button
    int buttonValue = digitalRead(context, 0, buttonPin);

    // if button wasn't pressed before and is pressed now
    if( buttonValue != lastButtonValue && buttonValue == 1 ){
      // then run calibration to set looking forward (gGravIdle)
      // and looking down (gGravCal)
      switch(calibrationState) {
      case 0: // first time button was pressed
        setForward = 1;
        // run task to get gravity values when sensor in neutral position
        Bela_scheduleAuxiliaryTask(gravityNeutralTask);
        calibrationState = 1;	// progress calibration state
        break;
      case 1: // second time button was pressed
        // run task to get gravity values when sensor 'looking down' (for head-tracking)
        Bela_scheduleAuxiliaryTask(gravityDownTask);
        calibrationState = 0; // reset calibration state for next time
        break;
      }
    }
    lastButtonValue = buttonValue;
    /*----------*/
    /*----------*/

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
    // log the three oscillators to the scope
		scope.log(context->audioOut[0],context->audioOut[1]);

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
      // calcuate the rotated position for each stream
      for(unsigned int i=0; i < NUM_STREAMS; i++){
        gVBAPUpdatePositions[i]=((gVBAPUpdateElevation[i]+90)*361)+gVBAPUpdateAzimuth[i]+180;
      }
      rotateVectors();
    	gSampleCount = 0;
    }
  }
}



// Auxiliary task to read from the I2C board
void readIMU(void*)
{
	// get calibration status
	uint8_t sys, gyro, accel, mag;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	// status of 3 means fully calibrated
	//rt_printf("CALIBRATION STATUSES\n");
	//rt_printf("System: %d   Gyro: %d Accel: %d  Mag: %d\n", sys, gyro, accel, mag);

	// quaternion data routine from MrHeadTracker
  	imu::Quaternion qRaw = bno.getQuat(); //get sensor raw quaternion data

  	if( setForward ) {
  		gIdleConj = qRaw.conjugate(); // sets what is looking forward
  		setForward = 0; // reset flag so only happens once
  	}

  	steering = gIdleConj * qRaw; // calculate relative rotation data
  	quat = gCalLeft * steering; // transform it to calibrated coordinate system
  	quat = quat * gCalRight;

  	ypr = quat.toEuler(); // transform from quaternion to Euler
}

// Auxiliary task to read from the I2C board
void getNeutralGravity(void*) {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	gGravIdle = gravity;
}

// Auxiliary task to read from the I2C board
void getDownGravity(void*) {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	gGravCal = gravity;
  	// run calibration routine as we should have both gravity values
  	calibrate();
}

// calibration of coordinate system from MrHeadTracker
// see http://www.aes.org/e-lib/browse.cfm?elib=18567 for full paper
// describing algorithm
void calibrate() {
  	imu::Vector<3> g, gravCalTemp, x, y, z;
  	g = gGravIdle; // looking forward in neutral position

  	z = g.scale(-1);
  	z.normalize();

  	gravCalTemp = gGravCal; // looking down
  	y = gravCalTemp.cross(g);
  	y.normalize();

  	x = y.cross(z);
  	x.normalize();

  	imu::Matrix<3> rot;
  	rot.cell(0, 0) = x.x();
  	rot.cell(1, 0) = x.y();
  	rot.cell(2, 0) = x.z();
  	rot.cell(0, 1) = y.x();
  	rot.cell(1, 1) = y.y();
  	rot.cell(2, 1) = y.z();
  	rot.cell(0, 2) = z.x();
  	rot.cell(1, 2) = z.y();
  	rot.cell(2, 2) = z.z();

  	gCal.fromMatrix(rot);

  	resetOrientation();
}

// from MrHeadTracker
// resets values used for looking forward
void resetOrientation() {
  	gCalLeft = gCal.conjugate();
  	gCalRight = gCal;
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
