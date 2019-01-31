#ifndef VECTOR_ROTATIONS_H_
#define VECTOR_ROTATIONS_H_

#include <Bela.h>
#include <imuhandler.h>

#define NUM_STREAMS 20      // MAXIMUM NUMBER OF AUDIO STREAMS

extern imu::Vector<3> ypr; //yaw pitch and roll angles

// BECKY - ADD AZIMUTHS HERE: range -180 (anti-clockwise) to 180 (clockwise)
int gVBAPDefaultAzimuth[10]={0,72,-72,144,-144,0,72,-72,144,-144};

// BECKY - ADD ELEVATIONS HERE: -90 (down) to 90 (up)
int gVBAPDefaultElevation[10]={-10,-10,-10,-10,-10,20,20,20,20,20};

//Rotation variables
float gVBAPDefaultVector[NUM_STREAMS][3];
float gVBAPRotatedVector[NUM_STREAMS][3];
int gVBAPUpdatePositions[NUM_STREAMS]={0};
int gVBAPUpdateAzimuth[NUM_STREAMS]={0};
int gVBAPUpdateElevation[NUM_STREAMS]={0};
int gVBAPTracking[3]={0};

// function to convert input stream point source locations to 3D vectors
void createVectors(int streams){
  for(int i=0; i<streams;i++){
    // convert default azi and ele to radians
    float aziRad=gVBAPDefaultAzimuth[i]*M_PI/180;
    float eleRad=gVBAPDefaultElevation[i]*M_PI/180;
    // convert co-ordinates to 3D vector values
    gVBAPDefaultVector[i][0]=cos(eleRad)*cos(aziRad);
    gVBAPDefaultVector[i][1]=cos(eleRad)*sin(aziRad);
    gVBAPDefaultVector[i][2]=sin(eleRad);
    // check default vector values on setup
    rt_printf("\nSource %d – X: %f\t Y: %f\t Z: %f\n", i, \
      gVBAPDefaultVector[i][0], \
      gVBAPDefaultVector[i][1], \
      gVBAPDefaultVector[i][2]);
 }
}

// function to rotate input stream point source locations using head-tracker
// YPR input data
void rotateVectors(int streams){
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
    gVBAPUpdateAzimuth[i]=(int)roundf(atan2(rollRot[1],rollRot[0])*180/M_PI);
    gVBAPUpdateElevation[i]=(int)roundf(asin(rollRot[2]/(sqrt(pow(rollRot[0],2) \
      +pow(rollRot[1],2)+pow(rollRot[2],2))))*180/M_PI);
  }
  //check revised azi and ele value of first input on each refresh
  //rt_printf("Azimuth %d - Elevation %d\n",gVBAPUpdateAzimuth[0],gVBAPUpdateElevation[0]);
}

#endif /*VECTOR_ROTATIONS_H_*/
