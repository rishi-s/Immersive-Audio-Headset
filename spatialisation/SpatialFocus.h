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
extern bool gCurrentSceneMode;

int gFocusScene = 51;
float gFocusSceneValues[2][51]={};

int gainCheck=0;


// function to set logarithmic focus level values for concurrent scene state
void initFocusScene(){
  // loop through applicable focus positions and calculate log gain
  for(int i=gFocusScene-1; i>=0; i--){
    if(i < 18 && i >= 11){
      // calculate gain for current position
      gFocusSceneValues[1][i]=log10((i-8.0)/10) / log10(10) * -1.0;
      // overwrite first six positions with no attenuation
      //gFocusSceneValues[1][i-4]=1.0;
    }
    if (i < 11 && i >= 4){
      // calculate gain for current position
      gFocusSceneValues[1][i]=log10((i-2.5)/144) / log10(144) * -1.0;
      // overwrite first six positions with no attenuation
      gFocusSceneValues[1][i-4]=1.0;
    }
    if(i >=38){
      gFocusSceneValues[0][i]=log10((i-26.0)/25) / log10(25) * -1.0;
    }
    else if (i < 38 && i >=7 ){
      gFocusSceneValues[0][i]=log10((i+6.0)/144) / log10(144) * -1.0;
      // overwrite first eight positions
      gFocusSceneValues[0][i-7]=0.5;
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
    gInputVolume[stream]=gFocusSceneValues[gCurrentSceneMode][position];
    }
    if(gainCheck>=8820){
      /*rt_printf("Azimuth %i is: %i; level is: %f\n", stream,
        gVBAPUpdateAzimuth[stream], gInputVolume[stream]);
      gainCheck=0;*/
    }
  }
}




#endif /* SPATIALFOCUS_H_ */
