/*
 *  Created on: 30 January, 2019
 *      Author: Rishi Shukla
 */

#ifndef VECTOR_ROTATIONS_H_
#define VECTOR_ROTATIONS_H_

#include "spatialisation/SpatialSceneParams.h" // definition of audio sources and context
#include "belaOnUrHead/imuhandler.h"


// ADD AZIMUTHS HERE: range -180 (anti-clockwise) to 180 (clockwise)
int gVBAPDefaultAzimuth[NUM_STREAMS]={-55,125,25,-175,85,-20,-110,60,-80,45,-140,120};

// ADD ELEVATIONS HERE: -90 (down) to 90 (up)
int gVBAPDefaultElevation[NUM_STREAMS]={20,0,-15,0,-10,0,-20,30,0,70,-5,35};

//Rotation variables
float gVBAPDefaultVector[NUM_STREAMS][3];
float gVBAPRotatedVector[NUM_STREAMS][3];
int gVBAPUpdatePositions[NUM_STREAMS]={0};
int gVBAPUpdateAzimuth[NUM_STREAMS]={0};
int gVBAPUpdateElevation[NUM_STREAMS]={0};
int gVBAPTracking[3]={0};

//Constants
float kDegToRad=M_PI/180;
float kRadToDeg=180/M_PI;

// function to convert input stream point source locations to 3D vectors
void createVectors(int streams){
  for(int i=0; i<streams;i++){
    // convert default azi and ele to radians
    float aziRad=gVBAPDefaultAzimuth[i]*kDegToRad;
    float eleRad=gVBAPDefaultElevation[i]*kDegToRad;
    // convert co-ordinates to 3D vector values
    gVBAPDefaultVector[i][0]=cos(eleRad)*cos(aziRad);
    gVBAPDefaultVector[i][1]=cos(eleRad)*sin(aziRad);
    gVBAPDefaultVector[i][2]=sin(eleRad);
 }
}

// function to rotate input stream point source locations using head-tracker
// YPR input data
void rotateVectors(int streams){
  createVectors(streams);
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
  for(int i=0; i<streams;i++){
    //apply yaw rotation to source 3D vector locations
    yawRot[0] = yawCos*gVBAPDefaultVector[i][0] + -yawSin*gVBAPDefaultVector[i][1];
    yawRot[1] = yawSin*gVBAPDefaultVector[i][0] + yawCos*gVBAPDefaultVector[i][1];
    yawRot[2] = gVBAPDefaultVector[i][2];
    //apply pitch rotation to yaw rotated locations
    pitchRot[0] = pitchCos*yawRot[0] + pitchSin*yawRot[2];
    pitchRot[1] = yawRot[1];
    pitchRot[2] = -pitchSin*yawRot[0] + pitchCos*yawRot[2];
    //apply roll rotation to yaw and pitch rotated locations
    rollRot[0] = pitchRot[0];
    rollRot[1] = rollCos*pitchRot[1] + -rollSin*pitchRot[2];
    rollRot[2] = rollSin*pitchRot[1] + rollCos*pitchRot[2];
    //convert 3DoF rotated 3D vector locations to azi and ele values
    gVBAPUpdateAzimuth[i]=(int)roundf(atan2(rollRot[1],rollRot[0])*kRadToDeg);
    gVBAPUpdateElevation[i]=(int)roundf(asin(rollRot[2]/(sqrt(pow(rollRot[0],2) \
      +pow(rollRot[1],2)+pow(rollRot[2],2))))*kRadToDeg);
    // calcuate the rotated position for each stream
    gVBAPUpdatePositions[i]=((gVBAPUpdateElevation[i]+90)*361) \
      +gVBAPUpdateAzimuth[i]+180;
  }
}

#endif /*VECTOR_ROTATIONS_H_*/
