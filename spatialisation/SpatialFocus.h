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
extern float gVBAPDefaultVector[NUM_FIXED_POSITIONS][3];
extern float gVBAPActiveVector[NUM_VBAP_TRACKS][3];
extern int gVBAPUpdateAzimuth[NUM_VBAP_TRACKS];
extern bool gCurrentSceneMode;
extern bool gPreviousSceneMode;
extern void startPlayback(int stream);

int gFocusScene = 51;
float gFocusSceneValues[2][51]={};

int gPreviousTargetSong=0;
int gCurrentTargetSong=0;
int gTargetState=0;
int gPlaybackStates[10][3]={
                          {0,1,2},{0,1,2},{1,2,3},{2,3,4},{2,3,4},
                          {0,1,5},{1,2,5},{2,3,5},{3,4,5},{4,0,5}
                          };

// function to set logarithmic focus level values for concurrent scene state
void initFocusScene(){
  // loop through applicable focus positions and calculate log gain
  for(int i=gFocusScene-1; i>=0; i--){
    /*if(i < 18 && i >= 11){
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
    }*/
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
  int position;
  // if in concurrent mode, update each song's gain level
  if(gCurrentSceneMode==false){
    // loop through each source
    for(int song=0; song<5; song++){
      // (treat all song positions as positive)
      if(gVBAPUpdateAzimuth[song]<0) position = gVBAPUpdateAzimuth[song]*-1;
      else position = gVBAPUpdateAzimuth[song];
      // if the source is out of range, silence it
      if(gVBAPUpdateAzimuth[song]> gFocusScene || \
        gVBAPUpdateAzimuth[song]<(gFocusScene*-1)){
        gInputVolume[song]=0.0;
      }
      else gInputVolume[song]=gFocusSceneValues[0][position];
      // if the position is within target range and it is a different song
      if(position<=18){
        gCurrentTargetSong=song;
        // if the target song has changed
        if(gCurrentTargetSong!=gPreviousTargetSong){
          gTargetState=gCurrentTargetSong;
          startPlayback(6);
        }
        gPreviousTargetSong=gCurrentTargetSong;
      }
    }
  }

  // Print the current target song and playback state
  //rt_printf("Target song is: %i (%i)\n", gCurrentTargetSong, position);
  /*rt_printf("TARGET – song is: %i; position is: %i; state is\n", \
  gCurrentTargetSong, gVBAPUpdateAzimuth[gCurrentTargetSong], gTargetState);*/
}






#endif /* SPATIALFOCUS_H_ */
