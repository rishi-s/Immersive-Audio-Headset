/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 ***** Code extended and adapted from Bela SampleStream example *****
 */

#ifndef SAMPLESTREAM_H_
#define SAMPLESTREAM_H_

#include "SampleData.h"          // to store SampleStream data
#include <string>
#include <Bela.h>
#include <libraries/sndfile/sndfile.h>             // to load audio files
#include <iostream>
#include <cstdlib>
#include "SpatialSceneParams.h"  // definition of audio sources and context

using namespace std;



// state machine for A/B comparison playback audition procedures
enum States {kPlaying, kPaused, kStopped};




class SampleStream
{

public:

    SampleStream(const char* filename, int numChannels, int bufferLength, bool looping);
    ~SampleStream();
    int openFile(const char* filename, int numChannels, int bufferLength, bool looping);
    void fillBuffer();
    void processFrame();
    float getSample(int channel);
    int bufferNeedsFilled();
    void togglePlayback();
    void togglePlaybackWithFade(float fadeLengthInSeconds);
    void togglePlayback(int toggle);
    void togglePlaybackWithFade(int toggle, float fadeLengthInSeconds);
    int isPlaying();
    //ADDITIONS:
    void stopPlaying();
    void flagFillBuffer();
    void setActiveBuffer(bool activeBuffer);

private:

    // private libsndfile wrappers
    int getSamples(const char* file, float *buf, int channel, int startFrame, int endFrame);
    int getNumChannels(const char* file);
    int getNumFrames(const char* file);

    // Two buffers for each channel:
    // one of them loads the next chunk of audio while the other one is used for playback
    SampleData *gSampleBuf[2];
    // read pointer relative current buffer (range 0-BUFFER_LEN)
    // initialise at BUFFER_LEN to pre-load second buffer (see render())
    int gReadPtr;
    // read pointer relative to file, increments by BUFFER_LEN (see fillBuffer())
    int gBufferReadPtr;
    // keeps track of which buffer is currently active (switches between 0 and 1)
    int gActiveBuffer;
    // this variable will let us know if the buffer doesn't manage to load in time
    int gDoneLoadingBuffer;
    int gBufferLength;
    int gNumChannels;

    int gNumFramesInFile;
    const char* gFilename;
    int gBufferToBeFilled;
    int gPlaying;

    float gFadeAmount;
    float gFadeLengthInSeconds;
    int gFadeDirection;

    //ADDITION: variable to track reaching end of file
    bool gFileEnd;
    bool gLooping;

    int gBusy;

    SNDFILE *sndfile = NULL;
	SF_INFO sfinfo ;

};

#endif // SAMPLESTREAM_H_
