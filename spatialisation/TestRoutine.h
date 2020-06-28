/*
 *  Created on: 8 September, 2018
 *      Author: Rishi Shukla
 */

#ifndef TESTROUTINE_H_
#define TESTROUTINE_H_

/*
Can be used to run a unit impulse test at all HRTF measurement locations.
Best run with BUFFER_SIZE 512 and input file for source 0 of 1536 samples
containing unit impulse.
*/

// variables for test locations
int gTestAzimuth=0;
int gTestElevation =-45;

// varaiables for test data output
float gDataOutputL[187][1536];
float gDataOutputR[187][1536];
int gColumnCount=0;
int gRowCount=0;

// move to the next azimuth position at -45 degrees elevation
void resetIncrement(){
  gTestElevation= -45;
  if(gTestAzimuth>-180 && gTestAzimuth <= 165) gTestAzimuth-=15;
  else if(gTestAzimuth==-180) gTestAzimuth=165;
}

// increment location according to LISTEN database HRTF set measurement format
void incrementLocation(){
  gTestElevation+=15;
  if(gTestAzimuth%360==0){
    if(gTestElevation>90){
      resetIncrement();
    }
  }
  else if(gTestAzimuth%60==0){
    if(gTestElevation>75){
      resetIncrement();
    }
  }
  else if(gTestAzimuth%30==0){
    if(gTestElevation>60){
      resetIncrement();
    }
  }
  else if(gTestAzimuth%15==0){
    if(gTestElevation>45){
      resetIncrement();
    }
  }
}

// write L/R output values to an array
void writeOutput(float outL, float outR){
  if(gRowCount<187){
    gDataOutputL[gRowCount][gColumnCount] = outL;
    gDataOutputR[gRowCount][gColumnCount] = outR;
    if(++gColumnCount >=1536){
      incrementLocation();
      gColumnCount=0;
      gRowCount++;
    }
  }
}

// write test output data to .csv file
void writeDataFile(int convSize){
  std::ofstream OutL("testImpL.csv");
  std::ofstream OutR("testImpR.csv");
  for (int i = 0; i < 187; i++) {
    for (int j = 0; j < convSize; j++){
      OutL << (float)gDataOutputL[i][j] << ',';
      OutR << (float)gDataOutputR[i][j] << ',';
    }
    OutL << '\n';
    OutR << '\n';
  }
}

#endif /* TESTROUTINE_H_ */
