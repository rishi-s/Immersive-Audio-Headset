/*
 *  Created on: 20 July, 2020
 *      Author: Rishi Shukla
 *****   *****
 */

#ifndef SPATIALFOCUS_H_
#define SPATIALFOCUS_H_

#include <cmath>

// variables from render.cpp
extern bool gHeadLocked;
extern float gInputVolume[NUM_STREAMS];
extern void startPlayback(int stream);

//variables from VectorRotations.h
extern float gVBAPDefaultVector[NUM_FIXED_POSITIONS][3];
extern float gVBAPActiveVector[NUM_VBAP_TRACKS][3];
extern int gVBAPUpdateAzimuth[NUM_VBAP_TRACKS];

//variables from OSC.h
extern bool gCurrentSceneMode;
extern bool gPreviousSceneMode;


// variables to define focus scene attenuation
int gFocusScene = 55;               // increments
float gFocusSceneValues[55]={};  // increment values

// variables for target song and accompanying target playback state
int gPreviousTargetSong=0;          // last song in focus
int gCurrentTargetSong=0;           // current song in focus
bool gNewTargetReached=false;       // focus transition status
int gTargetState=0;                 // state identifier and state makup
int gPlaybackStates[10][3]={
                          {4,0,1},{0,1,2},{1,2,3},{2,3,4},{3,4,0},
                          {0,1,5},{1,2,5},{2,3,5},{3,4,5},{4,0,5}
                          };

// function to set logarithmic focus level values for concurrent scene state
void initFocusScene(){
  // loop through applicable focus positions and calculate log gain
  for(int i=gFocusScene-1; i>=0; i--){
    if(i >=41){
      gFocusSceneValues[i]=log10((i-29.0)/26) / log10(26) * -1.0;
    }
    else if (i < 41 && i >=8 ){
      gFocusSceneValues[i]=log10((i+5.0)/165) / log10(165) * -1.0;
      // overwrite first eight positions
      gFocusSceneValues[i-8]=0.5;
    }
    /*rt_printf("Scene Level %i is: %f; Track Level %i is: %f\n",\
      i, gFocusSceneValues[i], i, gFocusSceneValues[i]);*/
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
        else gInputVolume[song]=gFocusSceneValues[position];
        // if the position is within target range and it is a different song
        if(position<=18){
          gCurrentTargetSong=song;
          // if the target song has changed, update the target state
          if(gCurrentTargetSong!=gPreviousTargetSong){
            gTargetState=gCurrentTargetSong;
            if(gPreviousSceneMode!=true)gNewTargetReached=true;
          }
          // update the previous state as a matter of course
          gPreviousTargetSong=gCurrentTargetSong;
        }
        // play a click notification any time a new target is clearly in focus

        //widen the notification trigger area for extreme (L&R) positions
        if(song == 0 || song == 4){
          if(position<10 && gNewTargetReached){
            startPlayback(6);
            gNewTargetReached=false;
          }
        }

        // keep the trigger area small for frontal positions
        else{
          if(position<3 && gNewTargetReached){
            startPlayback(6);
            gNewTargetReached=false;
          }
        }
      }
    }

    // Print the current target song and playback state
    //rt_printf("Target song is: %i (%i)\n", gCurrentTargetSong, position);
    /*rt_printf("TARGET – song is: %i; position is: %i; state is\n", \
    gCurrentTargetSong, gVBAPUpdateAzimuth[gCurrentTargetSong], gTargetState);*/
}






#endif /* SPATIALFOCUS_H_ */
