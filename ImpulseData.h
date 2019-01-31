/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 ****** Code adapted from Bela SampleLoader example  *****
 */

#ifndef IMPULSEDATA_H_
#define IMPULSEDATA_H_

#define NUM_SPEAKERS 8      // NUMBER OF VIRTUAL SPEAKERS
#define NUM_HRTFS 7         // NUMBER OF SELECTABLE HRTF SETS

// User defined structure to pass between main and render complex data retrieved from file
struct ImpulseData {
	float *samples;	// Samples in file
	int sampleLen;	// Total num of samples
};

// instantiate binaural impulse response data buffers (left and right channels)
ImpulseData gImpulseData[NUM_SPEAKERS*NUM_HRTFS*2];

// buffers and configuration for Neon FFT processing
ne10_fft_cpx_float32_t* impulseTimeDomainL[NUM_SPEAKERS*NUM_HRTFS];
ne10_fft_cpx_float32_t* impulseTimeDomainR[NUM_SPEAKERS*NUM_HRTFS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainL[NUM_SPEAKERS*NUM_HRTFS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainR[NUM_SPEAKERS*NUM_HRTFS];
ne10_fft_cfg_float32_t cfg;


// function to load 8 HRIRs for 7 HRTF sets on startup
void loadImpulse(int impulseLength){
  // for each HRTF set
  for(int i=0;i<NUM_HRTFS;i++){
    std::string hrtf=to_string(i);
    // load impulse .wav files from the relevant directory
    for(int j=0;j<NUM_SPEAKERS;j++) {
      std::string number=to_string(j+1);
      std::string file= "./set" + hrtf + "/impulse" + number + ".wav";
      const char * id = file.c_str();
      //determine the HRIR lengths (in samples) and populate the buffers
      for(int ch=0;ch<2;ch++) {
        //allocate impulse number by hrtf, speaker and channel
        int impulseChannel = (i*NUM_SPEAKERS*2)+(j*2)+ch;
        gImpulseData[impulseChannel].sampleLen = getImpulseNumFrames(id);
        gImpulseData[impulseChannel].samples = new float[getImpulseNumFrames(id)];
        getImpulseSamples(id,gImpulseData[impulseChannel].samples,ch,0, \
          gImpulseData[impulseChannel].sampleLen);
        //check buffer lengths and start/end values during setup
        rt_printf("Length %d = %d\n",impulseChannel, \
          gImpulseData[impulseChannel].sampleLen);
        rt_printf("Impulse %d = %f\n",impulseChannel, \
          gImpulseData[impulseChannel].samples[0]);
        rt_printf("Impulse %d = %f\n",impulseChannel, \
          gImpulseData[impulseChannel].samples[impulseLength-1]);
      }
    }
  }
}


// function to generate IR frequency domain values
void transformHRIRs(int impulseLength, int convSize){
  // allocate memory and add sample values to each HRTF set HRIR FFT buffer (L and R)
  for (int i = 0; i < NUM_SPEAKERS*NUM_HRTFS; i++){
    int impulseL = i*2;
    int impulseR = impulseL+1;
    impulseTimeDomainL[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (convSize \
      * sizeof (ne10_fft_cpx_float32_t));
    impulseTimeDomainR[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (convSize \
      * sizeof (ne10_fft_cpx_float32_t));
    impulseFrequencyDomainL[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (convSize \
       * sizeof (ne10_fft_cpx_float32_t));
    impulseFrequencyDomainR[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (convSize \
      * sizeof (ne10_fft_cpx_float32_t));

    // assign real component values from each impulse file
    for (int n = 0; n < impulseLength; n++)
    {
      impulseTimeDomainL[i][n].r = (ne10_float32_t) gImpulseData[impulseL].samples[n];
      impulseTimeDomainR[i][n].r = (ne10_float32_t) gImpulseData[impulseR].samples[n];
    }
    // transform to frequency domain (L and R)
    ne10_fft_c2c_1d_float32_neon(impulseFrequencyDomainL[i], impulseTimeDomainL[i], \
      cfg, 0);
    ne10_fft_c2c_1d_float32_neon(impulseFrequencyDomainR[i], impulseTimeDomainR[i], \
      cfg, 0);
  }
}





#endif /* IMPULSEDATA_H_ */
