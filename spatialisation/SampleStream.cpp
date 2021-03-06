/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 ***** Code extended and adapted from Bela SampleStream example *****
 */


#include "SampleStream.h" // adapted code for streaming/processing audio

// variable from render.cpp
extern int gPlaybackState;

//user controlled variable from main.cpp
extern bool gFixedTrajectory;

// variables from OSC.h
extern bool gHeardAState;
extern bool gHeardBState;

// variables from ABRoutine.h
extern bool gPlayingA;
extern bool gPlayingB;

// method to create new sample stream with file, number of channels, buffer size
SampleStream::SampleStream(const char* filename, int numChannels, int bufferLength, bool looping) {

    gSampleBuf[0] = NULL; //load buffer pointer
    gSampleBuf[1] = NULL; //playback buffer pointer

    openFile(filename,numChannels,bufferLength,looping); //open audio file

}

//close and clear the sample stream
SampleStream::~SampleStream() {

    sf_close(sndfile);
    for(int ch=0;ch<gNumChannels;ch++) {
        for(int i=0;i<2;i++) {
        if(gSampleBuf[i][ch].samples != NULL)
            delete[] gSampleBuf[i][ch].samples;
        }
    }
    free(gSampleBuf[0]);
    free(gSampleBuf[1]);

}

// method to load samples from audio file into buffer
int SampleStream::openFile(const char* filename, int numChannels, \
      int bufferLength, bool looping) {


    // set loading state to true
    gBusy = 1;

    // handle missing, incompatible or corrupt files
    sf_close(sndfile);
    sfinfo.format = 0;
    if (!(sndfile = sf_open (string(filename).c_str(), SFM_READ, &sfinfo))) {
		cout << "Couldn't open file " << filename << ": " << sf_strerror(sndfile) << endl;
		return 1;
	}

    // assign input arguments to variables
    gBufferLength = bufferLength;
    gNumChannels = numChannels;
    gFilename = filename;

    // clear and release all memory blocks for both buffers (and all channels)
    for(int i=0;i<2;i++) {
        if(gSampleBuf[i] != NULL) {
            cout << "I AM NOT A NULL" << endl;
            for(int ch=0;ch<numChannels;ch++) {
                if(gSampleBuf[i][ch].samples != NULL)
                    delete[] gSampleBuf[i][ch].samples;
            }
            free(gSampleBuf[i]);
        }
    }

    // reallocate memory blocks for both buffers
    gSampleBuf[0] = (SampleData*)calloc(bufferLength*numChannels,sizeof(SampleData));
    gSampleBuf[1] = (SampleData*)calloc(bufferLength*numChannels,sizeof(SampleData));

    // reset variables
    gReadPtr = bufferLength;
    gBufferReadPtr = 0;
    gActiveBuffer = 0;
    gDoneLoadingBuffer = 1;
    gBufferToBeFilled = 0;

    gPlaying = 0;
    gFadeAmount = 0;
    gFadeLengthInSeconds = 0.1;
    gFadeDirection = -1;

    gNumFramesInFile = getNumFrames(gFilename);

    gLooping = looping;

    // check length of file exceeds buffer size
    if(gNumFramesInFile <= gBufferLength) {
        printf("Sample needs to be longer than buffer size. \
          This example is intended to work with long samples.");
        return 1;
    }

    // set length and create array for each buffer (and all channels)
    for(int ch=0;ch<gNumChannels;ch++) {
        for(int i=0;i<2;i++) {
            gSampleBuf[i][ch].sampleLen = gBufferLength;
        	gSampleBuf[i][ch].samples = new float[gBufferLength];
            if(getSamples(gFilename,gSampleBuf[i][ch].samples,ch,0,gBufferLength)) {
                printf("error getting samples\n");
                return 1;
            }
        }
    }


    gBusy = 0;

    return 0;
}



void SampleStream::processFrame() {

    if(gFadeAmount<1 && gFadeAmount>0) {
        gFadeAmount += (gFadeDirection*((1.0/gFadeLengthInSeconds)/44100.0));
    }
    else if(gFadeAmount < 0)
        gPlaying = 0;

    if(gPlaying) {
        // Increment read pointer and reset to 0 when end of file is reached
        if(++gReadPtr >= gBufferLength) {
            // if(!gDoneLoadingBuffer)
            //     rt_printf("Couldn't load buffer in time :( -- try increasing buffer size!");
            gDoneLoadingBuffer = 0;
            gReadPtr = 0;
            gActiveBuffer = !gActiveBuffer;
            gBufferToBeFilled = 1;
            //ADDITION: if end of file, stop when not looping and reset file end
            if(gFileEnd) {
              if(!gLooping){
                stopPlaying();
                // stop all playback and update flags if using fixed trajectory
                if(gFixedTrajectory) {
                  gPlaybackState=kStopped;
                  if(gPlayingA)gHeardAState=true;
                  else if(gPlayingB)gHeardBState=true;
                }
              }
              gFileEnd=0;
            }
        }
    }

}

float SampleStream::getSample(int channel) {
    if(gPlaying) {
        // Wrap channel index in case there are more audio output channels than the file contains
        float out = gSampleBuf[gActiveBuffer][channel%gNumChannels].samples[gReadPtr];
    	return out * gFadeAmount * gFadeAmount;
    }
    return 0;

}

void SampleStream::fillBuffer() {

    if(!gBusy) {

        // increment buffer read pointer by buffer length
        gBufferReadPtr+=gBufferLength;

        // reset buffer pointer if it exceeds the number of frames in the file
        if(gBufferReadPtr>=gNumFramesInFile)
            gBufferReadPtr=0;

        int endFrame = gBufferReadPtr + gBufferLength;
        int zeroPad = 0;

        // if reaching the end of the file take note of the last frame index
        // so we can zero-pad the rest later
        if((gBufferReadPtr+gBufferLength)>=gNumFramesInFile-1) {
              endFrame = gNumFramesInFile-1;
              zeroPad = 1;
              // ADDITION: flag the end of the file
              gFileEnd=1;
        }

        for(int ch=0;ch<gNumChannels;ch++) {

            // fill (nonactive) buffer
            getSamples(gFilename,gSampleBuf[!gActiveBuffer][ch].samples,ch
                        ,gBufferReadPtr,endFrame);

            // zero-pad if necessary
            if(zeroPad) {
                int numFramesToPad = gBufferLength - (endFrame-gBufferReadPtr);
                for(int n=0;n<numFramesToPad;n++)
                    gSampleBuf[!gActiveBuffer][ch].samples[n+(gBufferLength-numFramesToPad)] = 0;
            }

        }

        gDoneLoadingBuffer = 1;
        gBufferToBeFilled = 0;

        // printf("done loading buffer!\n");

    }

}

int SampleStream::bufferNeedsFilled() {
    return gBufferToBeFilled;
}

// ADDITION: function to request buffer fill
void SampleStream::flagFillBuffer() {
  gBufferToBeFilled=1;
}

void SampleStream::togglePlayback() {
    gPlaying = !gPlaying;
    gFadeAmount = gPlaying;
}

void SampleStream::togglePlaybackWithFade(int toggle, float fadeLengthInSeconds) {

    gFadeDirection = toggle;
    if(fadeLengthInSeconds<=0)
        fadeLengthInSeconds = 0.00001;
    gFadeLengthInSeconds = fadeLengthInSeconds;
    gFadeAmount += (gFadeDirection*((1.0/gFadeLengthInSeconds)/44100.0));
    if(gFadeDirection)
        gPlaying = 1;
}

void SampleStream::togglePlayback(int toggle) {
    gPlaying = toggle;
    gFadeAmount = gPlaying;
}

void SampleStream::togglePlaybackWithFade(float fadeLengthInSeconds) {

    gFadeDirection = gFadeDirection*-1;
    if(fadeLengthInSeconds<=0)
        fadeLengthInSeconds = 0.00001;
    gFadeLengthInSeconds = fadeLengthInSeconds;
    gFadeAmount += (gFadeDirection*((1.0/gFadeLengthInSeconds)/44100.0));
    if(gFadeDirection)
        gPlaying = 1;
}


// ADDITION: function to stop stream
void SampleStream::stopPlaying() {
  // reset variables
  gPlaying = 0;
  gBufferToBeFilled = 0;
  gActiveBuffer = 1;
  gBufferReadPtr = 0;
  gReadPtr = BUFFER_SIZE;
}

void SampleStream::setActiveBuffer(bool activeBuffer){
  gActiveBuffer=activeBuffer;
};


//function to check if stream is playing
int SampleStream::isPlaying() {

    return gPlaying;
}


// private libsndfile wrappers (previously in SampleLoader.h)

int SampleStream::getSamples(const char* file, float *buf, int channel, int startFrame, int endFrame)
{

	int numChannelsInFile = sfinfo.channels;
	if(numChannelsInFile < channel+1)
	{
		cout << "Error: " << file << " doesn't contain requested channel" << endl;
		return 1;
	}

    int frameLen = endFrame-startFrame;

    if(frameLen <= 0 || startFrame < 0 || endFrame <= 0 || endFrame > sfinfo.frames)
	{
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

    delete[] tempBuf;

	return 0;
}

// funtion to return number of channels in stream
int SampleStream::getNumChannels(const char* file) {

	return sfinfo.channels;
}

//function to return number of frames in stream
int SampleStream::getNumFrames(const char* file) {

	return sfinfo.frames;
}
