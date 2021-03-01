/*
 *  Created on: 20 July, 2020
 *      Author: Rishi Shukla
 *****   *****
 */

#ifndef SPATIALFOCUS_H_
#define SPATIALFOCUS_H_

#include <cmath>


extern bool gHeadLocked;
extern float gInputVolume[NUM_STREAMS];
extern float gVBAPDefaultVector[NUM_VBAP_TRACKS][3];
extern int gVBAPUpdateAzimuth[NUM_VBAP_TRACKS];
extern bool gCurrentSceneMode;
extern bool gPreviousSceneMode;

int gFocusScene = 51;
float gFocusSceneValues[2][51]={};

int gTargetSong=0;
int gTargetState=0;
int gPlaybackStates[10][3]={
                          {0,1,2},{0,1,2},{1,2,3},{2,3,4},{2,3,4},
                          {0,1,5},{1,2,5},{2,3,5},{3,4,5},{4,0,5}
                          };


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
      gFocusSceneValues[1][i]=log10((i-2.5)/148) / log10(148) * -1.0;
      // overwrite first six positions with no attenuation
      gFocusSceneValues[1][i-4]=1.0;
    }
    if(i >=38){
      gFocusSceneValues[0][i]=log10((i-26.0)/25) / log10(25) * -1.0;
    }
    else if (i < 38 && i >=7 ){
      gFocusSceneValues[0][i]=log10((i+6.0)/148) / log10(148) * -1.0;
      // overwrite first eight positions
      gFocusSceneValues[0][i-7]=0.5;
    }
    rt_printf("Scene Level %i is: %f; Track Level %i is: %f\n",\
      i, gFocusSceneValues[0][i], i, gFocusSceneValues[1][i]);
  }
}

// function to update input gain according to source azimuth
void getFocusValues(){
  // loop through each source
  for(int song=0; song<5; song++){
    // if the source is out of range, silence it
    if(gVBAPUpdateAzimuth[song]>= gFocusScene || \
        gVBAPUpdateAzimuth[song]<=(gFocusScene*-1)){
        gInputVolume[song]=0.0;
      }
      // otherwise, pick the appropriate gain based on its azimuth
    else {
      int position;
      // treat all positions as positive
      if(gVBAPUpdateAzimuth[song]<0) {
        position = gVBAPUpdateAzimuth[song]*-1;
      }
      else{
        position = gVBAPUpdateAzimuth[song];
      }
      //if the position is within range, update target song and streams
      if(position<=18){
        gTargetSong=song;
        // if in concurrent mode, set the target state to the target song value
        if(gCurrentSceneMode==false){
          gTargetState=gTargetSong;
        }
        // if in solo mode, set the target state to the equivalent value
        else{
          gTargetState=gTargetSong+5;
          // update the default location of the voiceover
          gVBAPDefaultVector[5][0]=gVBAPDefaultVector[gTargetSong][0];
          gVBAPDefaultVector[5][1]=gVBAPDefaultVector[gTargetSong][1];
          gVBAPDefaultVector[5][2]=gVBAPDefaultVector[gTargetSong][2];
        }

        // Print the current target song and playback state
        //rt_printf("Target song is: %i (%i)\n", gTargetSong, position);
        /*rt_printf("Target state is: %i,%i,%i (%i)\n", \
          gVBAPDefaultAzimuth[gPlaybackStates[gTargetState][0]], \
          gVBAPDefaultAzimuth[gPlaybackStates[gTargetState][1]], \
          gVBAPDefaultAzimuth[gPlaybackStates[gTargetState][2]], position);*/
      }
    gInputVolume[song]=gFocusSceneValues[gCurrentSceneMode][position];
    }
  }
}




#endif /* SPATIALFOCUS_H_ */
