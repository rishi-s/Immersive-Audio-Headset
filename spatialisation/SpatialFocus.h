/*
 *  Created on: 20 July, 2020
 *      Author: Rishi Shukla
 *****   *****
 */

#ifndef SPATIALFOCUS_H_
#define SPATIALFOCUS_H_

#include <cmath>


extern int gStreams;
extern bool gHeadLocked;
extern float gInputVolume[NUM_STREAMS];
extern int gVBAPUpdateAzimuth[NUM_STREAMS];
extern bool gSceneMode;

int gFocusScene = 45;
float gFocusSceneValues[2][45]={};

int gainCheck=0;


// function to set logarithmic focus level values for concurrent scene state
void initFocusScene(){
  // loop through applicable focus positions and calculate log gain
  for(int i=gFocusScene; i>5; i--){
    if(i <= 25){
      // calculate gain for current position
      gFocusSceneValues[1][i]=log10((i-4.0)/(25-4.0)) / \
        log10(25-4.0) * -1.0;
      // overwrite first six positions with no attenuation
      gFocusSceneValues[1][i-6]=1.0;
    }
    if(i >= 31){
      gFocusSceneValues[0][i]=log10((i-19.0)/26) / log10(26) * -1.0;
      gFocusSceneValues[0][i-6]=0.5;
    }
    else{
      gFocusSceneValues[0][i]=log10((i+6.0)/121) / log10(121) * -1.0;
      gFocusSceneValues[0][i-6]=0.5;
    }
    rt_printf("Scene Level %i is: %f; Track Level %i is: %f\n",\
      i, gFocusSceneValues[0][i], i, gFocusSceneValues[1][i]);
  }
}

// function to update input gain according to source azimuth
void getFocusValues(){
  gainCheck++;
  // loop through each source
  for(int stream=0; stream<gStreams; stream++){
    // if the source is out of range, silence it
    if(gVBAPUpdateAzimuth[stream]>= gFocusScene || \
        gVBAPUpdateAzimuth[stream]<=(gFocusScene*-1)){
        gInputVolume[stream]=0.0;
      }
      // otherwise, pick the appropriate gain based on its azimuth
    else {
      int position;
      // treat all positions as positive
      if(gVBAPUpdateAzimuth[stream]<0) {
        position = gVBAPUpdateAzimuth[stream]*-1;
      }
      else{
        position = gVBAPUpdateAzimuth[stream];
      }
    gInputVolume[stream]=gFocusSceneValues[gSceneMode][position];
    }
    if(gainCheck>=8820){
      //rt_printf("Azimuth %i is: %i; level is: %f\n", stream, \
        gVBAPUpdateAzimuth[stream], gInputVolume[stream]);
      gainCheck=0;
    }
  }
}




#endif /* SPATIALFOCUS_H_ */
