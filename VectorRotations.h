/*
 *  Created on: 30 January, 2019
 *      Author: Rishi Shukla
 */

#ifndef VECTOR_ROTATIONS_H_
#define VECTOR_ROTATIONS_H_

#include "spatialisation/SpatialSceneParams.h" // definition of audio sources and context
#include "belaOnUrHead/imuhandler.h"


// ADD AZIMUTHS HERE: range -180 (anti-clockwise) to 180 (clockwise)
int gVBAPDefaultAzimuth[NUM_FIXED_POSITIONS]={-74,-37,0,37,74,-74,-37,0,37,74};

// ADD ELEVATIONS HERE: -90 (down) to 90 (up)
int gVBAPDefaultElevation[NUM_FIXED_POSITIONS]={0,0,0,0,0,30,30,30,30,30};

//Rotation variables
float gVBAPDefaultVector[NUM_FIXED_POSITIONS][3];
float gVBAPActiveVector[NUM_VBAP_TRACKS][3];
int gVBAPUpdatePositions[NUM_VBAP_TRACKS]={0};
int gVBAPUpdateAzimuth[NUM_VBAP_TRACKS]={0};
int gVBAPUpdateElevation[NUM_VBAP_TRACKS]={0};

//Constants
float kDegToRad=M_PI/180;
float kRadToDeg=180/M_PI;

// function to convert input stream point source locations to 3D vectors
void createVectors(){
  for(int point=0; point<NUM_FIXED_POSITIONS; point++){
    // convert default azi and ele to radians
    float aziRad=gVBAPDefaultAzimuth[point]*kDegToRad;
    float eleRad=gVBAPDefaultElevation[point]*kDegToRad;
    // convert co-ordinates to 3D vector values
    gVBAPDefaultVector[point][0]=cos(eleRad)*cos(aziRad);
    gVBAPDefaultVector[point][1]=cos(eleRad)*sin(aziRad);
    gVBAPDefaultVector[point][2]=sin(eleRad);
 }
 for(int track=0; track<NUM_VBAP_TRACKS; track++){
   for(int i=0; i<3; i++){
     gVBAPActiveVector[track][i]=gVBAPDefaultVector[track][i];
   }
 }
}

// function to rotate input stream point source locations using head-tracker
// YPR input data
void rotateVectors(){
  //calculate yaw rotation matrix values
  float yawRot[3]={0};
  float yawSin = sin(ypr[0]);
  float yawCos = cos(ypr[0]);
  //calculate pitch rotation matrix values
  float pitchRot[3]={0};
  float pitchSin = sin(-ypr[1]);
  float pitchCos = cos(-ypr[1]);
  //calculate roll rotation matrix values
  float rollRot[3]={0};
  float rollSin = sin(ypr[2]);
  float rollCos = cos(ypr[2]);
  for(int track=0; track<NUM_VBAP_TRACKS; track++){
    //apply yaw rotation to source 3D vector locations
    yawRot[0] = yawCos*gVBAPActiveVector[track][0] + -yawSin*gVBAPActiveVector[track][1];
    yawRot[1] = yawSin*gVBAPActiveVector[track][0] + yawCos*gVBAPActiveVector[track][1];
    yawRot[2] = gVBAPActiveVector[track][2];
    //apply pitch rotation to yaw rotated locations
    pitchRot[0] = pitchCos*yawRot[0] + pitchSin*yawRot[2];
    pitchRot[1] = yawRot[1];
    pitchRot[2] = -pitchSin*yawRot[0] + pitchCos*yawRot[2];
    //apply roll rotation to yaw and pitch rotated locations
    rollRot[0] = pitchRot[0];
    rollRot[1] = rollCos*pitchRot[1] + -rollSin*pitchRot[2];
    rollRot[2] = rollSin*pitchRot[1] + rollCos*pitchRot[2];
    //convert 3DoF rotated 3D vector locations to azi and ele values
    gVBAPUpdateAzimuth[track]=(int)roundf(atan2(rollRot[1],rollRot[0])*kRadToDeg);
    gVBAPUpdateElevation[track]=(int)roundf(asin(rollRot[2]/(sqrt(pow(rollRot[0],2) \
      +pow(rollRot[1],2)+pow(rollRot[2],2))))*kRadToDeg);
    // calcuate the rotated position for each stream
    gVBAPUpdatePositions[track]=((gVBAPUpdateElevation[track]+90)*361) \
      +gVBAPUpdateAzimuth[track]+180;
  }
}

#endif /*VECTOR_ROTATIONS_H_*/
