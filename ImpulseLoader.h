/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 ***** Code extended and adapted from Bela SampleLoader example *****
 */

//include files
#include <Bela.h>
#include <sndfile.h>				// to load audio files
#include <string>
#include <iostream>
#include <cstdlib>
#include "ImpulseData.h"


// instantiate binaural impulse response data buffers (left and right channels)
ImpulseData gImpulseData[NUM_SPEAKERS*NUM_HRTFS*2];

// buffers and configuration for Neon FFT processing
ne10_fft_cpx_float32_t* impulseTimeDomainL[NUM_SPEAKERS*NUM_HRTFS];
ne10_fft_cpx_float32_t* impulseTimeDomainR[NUM_SPEAKERS*NUM_HRTFS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainL[NUM_SPEAKERS*NUM_HRTFS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainR[NUM_SPEAKERS*NUM_HRTFS];
ne10_fft_cfg_float32_t cfg;

using namespace std;

// Load samples from file
int getImpulseSamples(string file, float *buf, int channel, int startFrame, int endFrame)
{
	SNDFILE *sndfile ;
	SF_INFO sfinfo ;
	sfinfo.format = 0;
	if (!(sndfile = sf_open (file.c_str(), SFM_READ, &sfinfo))) {
		cout << "Couldn't open file " << file << ": " << sf_strerror(sndfile) << endl;
		return 1;
	}

	int numChannelsInFile = sfinfo.channels;
	if(numChannelsInFile < channel+1)
	{
		cout << "Error: " << file << " doesn't contain requested channel" << endl;
		return 1;
	}

    int frameLen = endFrame-startFrame;

    if(frameLen <= 0 || startFrame < 0 || endFrame <= 0 || endFrame > sfinfo.frames)
	{
	    //printf("framelen %d startframe %d endframe %d, sfinfo.frames %d\n",frameLen,startFrame,endFrame,sfinfo.frames);
		cout << "Error: " << file << " invalid frame range requested" << endl;
		return 1;
	}

    sf_seek(sndfile,startFrame,SEEK_SET);

    float* tempBuf = new float[frameLen*numChannelsInFile];

	int subformat = sfinfo.format & SF_FORMAT_SUBMASK;
	int readcount = sf_read_float(sndfile, tempBuf, frameLen*numChannelsInFile); //FIXME

	// Pad with zeros in case we couldn't read whole file
	for(int k = readcount; k <frameLen*numChannelsInFile; k++)
		tempBuf[k] = 0;

	if (subformat == SF_FORMAT_FLOAT || subformat == SF_FORMAT_DOUBLE) {
		double	scale ;
		int 	m ;

		sf_command (sndfile, SFC_CALC_SIGNAL_MAX, &scale, sizeof (scale)) ;
		if (scale < 1e-10)
			scale = 1.0 ;
		else
			scale = 32700.0 / scale ;
		cout << "File samples scale = " << scale << endl;

		for (m = 0; m < frameLen; m++)
			tempBuf[m] *= scale;
	}

	for(int n=0;n<frameLen;n++)
	    buf[n] = tempBuf[n*numChannelsInFile+channel];

	sf_close(sndfile);

	return 0;
}

int getImpulseNumChannels(string file) {

	SNDFILE *sndfile ;
	SF_INFO sfinfo ;
	sfinfo.format = 0;
	if (!(sndfile = sf_open (file.c_str(), SFM_READ, &sfinfo))) {
		cout << "Couldn't open file " << file << ": " << sf_strerror(sndfile) << endl;
		return -1;
	}

	return sfinfo.channels;
}

int getImpulseNumFrames(string file) {

	SNDFILE *sndfile ;
	SF_INFO sfinfo ;
	sfinfo.format = 0;
	if (!(sndfile = sf_open (file.c_str(), SFM_READ, &sfinfo))) {
		cout << "Couldn't open file " << file << ": " << sf_strerror(sndfile) << endl;
		return -1;
	}

	return sfinfo.frames;
}

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

// function to clear impulses from memory
void clearImpulseFFTBuffers(){
	for(int i=0;i<NUM_SPEAKERS*NUM_HRTFS;i++) {
		NE10_FREE(impulseTimeDomainL[i]);
		NE10_FREE(impulseTimeDomainR[i]);
		NE10_FREE(impulseFrequencyDomainL[i]);
		NE10_FREE(impulseFrequencyDomainR[i]);
	}
}
