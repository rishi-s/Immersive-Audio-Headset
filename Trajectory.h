/*
 *  Created on: 8 February, 2019
 *      Author: Rishi Shukla
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_


// variables referenced from main render.cpp
extern int gVBAPDefaultAzimuth[NUM_STREAMS];
extern int gVBAPDefaultElevation[NUM_STREAMS];
extern int gHopSize;

// trajectory variables
int gTrajectoryCount=0;
int gTrajectoryLength=441000;
int gPauseLength=19294;
int gRotationLength=110250;
int gStopLength=66150;
int gTrajectoryStages[6]={gRotationLength,
                          gRotationLength+gStopLength,
                          gRotationLength+(2*gStopLength),
                          gRotationLength+(3*gStopLength),
                          gRotationLength+(4*gStopLength),
                          gRotationLength+(5*gStopLength)
                          };
int gOrbitCount=0;
int gOrbitLength=82688;

// function to generate trajectory fixed and repeating trajectory
void updatePositions(){
  //Update trajectory count
  gTrajectoryCount++;
  //orbit trajectory
  if (gTrajectoryCount>gPauseLength && gTrajectoryCount<=gTrajectoryStages[0]){
    if(gTrajectoryCount<=(gPauseLength+gOrbitLength)){
      if(++gOrbitCount>=230){
        if(++gVBAPDefaultAzimuth[0]==180)gVBAPDefaultAzimuth[0]=-180;
        gOrbitCount=0;
      }
    }
  }
  //position ahead
  else if (gTrajectoryCount>gTrajectoryStages[0] && \
    gTrajectoryCount<=gTrajectoryStages[1]){
      gVBAPDefaultAzimuth[0]=0;
      gVBAPDefaultElevation[0]=0;
  }
  //position lower front
  else if (gTrajectoryCount>gTrajectoryStages[1] && \
    gTrajectoryCount<=gTrajectoryStages[2]){
      gVBAPDefaultAzimuth[0]=0;
      gVBAPDefaultElevation[0]=-45;
  }
  //position lower back
  else if (gTrajectoryCount>gTrajectoryStages[2] && \
    gTrajectoryCount<=gTrajectoryStages[3]){
      gVBAPDefaultAzimuth[0]=180;
      gVBAPDefaultElevation[0]=-45;
  }
  //position ahead
  else if (gTrajectoryCount>gTrajectoryStages[3] && \
    gTrajectoryCount<=gTrajectoryStages[4]){
      gVBAPDefaultAzimuth[0]=0;
      gVBAPDefaultElevation[0]=0;
  }
  //position above
  else if (gTrajectoryCount>gTrajectoryStages[4] && \
    gTrajectoryCount<=gTrajectoryStages[5]){
      gVBAPDefaultAzimuth[0]=0;
      gVBAPDefaultElevation[0]=90;
  }
  //reset to initial status
  else if(gTrajectoryCount>=gTrajectoryLength){
    gTrajectoryCount=0;
    gOrbitCount=0;
    gVBAPDefaultAzimuth[0]=0;
    gVBAPDefaultElevation[0]=0;
  }
}

#endif /* TRAJECTORY_H_ */
