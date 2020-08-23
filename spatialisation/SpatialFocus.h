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

int gFocusLevels = 25;
float gFocusLevelValues[25]={};

int gainCheck=0;


// function to set logarithmic focus level values
void initFocusLevels(){
  // loop through applicable focus positions and calculate log gain
  for(int i=gFocusLevels; i>5; i--){
    gFocusLevelValues[i]=log10((i-4.0)/(gFocusLevels-4.0)) / log10(gFocusLevels-4.0) * -1.0;
    gFocusLevelValues[i-6]=1.0;
    rt_printf("Level %i is: %f\n", i, gFocusLevelValues[i]);
  }
}

// function to update input gain according to source azimuth
void getFocusValues(){
  gainCheck++;
  // loop through each source
  for(int stream=0; stream<gStreams; stream++){
    // if the source is out of range, silence it
    if(gVBAPUpdateAzimuth[stream]>= gFocusLevels || gVBAPUpdateAzimuth[stream]<=(gFocusLevels*-1)){
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
    gInputVolume[stream]=gFocusLevelValues[position];
    }
    if(gainCheck>=4410){
      rt_printf("Azimuth %i is: %i; level is: %f\n", stream, gVBAPUpdateAzimuth[stream],\
        gInputVolume[stream]);
      gainCheck=0;
    }
  }
}




#endif /* SPATIALFOCUS_H_ */
