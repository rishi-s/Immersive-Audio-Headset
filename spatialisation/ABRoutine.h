/*
 *  Created on: 11 March, 2019
 *      Author: Rishi Shukla
 */

#ifndef ABROUTINE_
#define ABROUTINE_

#include <Bela.h>
#include <algorithm>
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include <unordered_map>

// state machines for current A/B selection and headtracking calibration
enum Selection {kNoneSelected, kASelected, kBSelected};
enum Calibration {kAhead,kAzimuth,kDown,kElevation};
std::string gProgressText="Example";
std::string gPlaybackText=" ";
std::string gSubmitText=" ";
std::string gCalibrateText="Look ahead";
std::string gLocationText="Example 1";

int gChoiceState=kNoneSelected;
int gCalibrateState=kAhead;
bool gPlayingA=false;
bool gPlayingB=false;


// all HRFT tournament match combinations
int HRTFCombos [21][2]= {
                          {0,1},{0,2},{0,3},{0,4},{0,5},{0,6},
                          {1,2},{1,3},{1,4},{1,5},{1,6},
                          {2,3},{2,4},{2,5},{2,6},
                          {3,4},{3,5},{3,6},
                          {4,5},{4,6},
                          {5,6}
                        };

// (randomisable) index for HRTF pair lookup
int HRTFComboIdx [21]= {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

// (randomisable) index for HRTF pair order
int HRTFComboOrd [2] = {0,1};

// index for tournament progress
int gComparIndex = 0;

// 2D array to store randomised HRTF tournament, generated on setup
int gComparMatches [22][2] = {{7,8}};

// variables to store tournament outcomes
int gHRTFResponses[22]={};
float gHRTFResponseTimes[22]={};
int gHRTFResponseCount[7]={};
int gWinningHRTF;
int gLosingHRTF;

// all HRFT tournament match combinations
int LocationCombos [20][2]= {
  {-153,-45},{-102,-45},{-51,-45},{0,-45},{51,-45},{102,-45},{153,-45},
  {-153,0},{-102,0},{-51,0},{51,0},{102,0},{153,0},
  {-153,45},{-102,45},{-51,45},{0,45},{51,45},{102,45},{153,45}
};

// (randomisable) index for location lookup
int LocationComboIdx [20]= {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};

// index for tournament progress
int gLocationIndex = 0;

// 2D array to store randomised HRTF tournament, generated on setup
int gLocationTrials [42][2] = {{120,0},{80,45}};

// variables to store localisation outcomes
int gLocalisationResponses[42][2]={};
int gLocalisationHRTFState[42]={7,7};
int gLocalisationHRTF[42]={7,7};
float gLocalisationResponseTimes[42]={};



// Random pair generator funciton
void createPairs(){
  //initialise random seed
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  //shuffle comparison combination order
  shuffle(&HRTFComboIdx[0], &HRTFComboIdx[21], std::default_random_engine(seed));
  //loop through shuffled comparison combination order and assign pairs to array
  for (int i = 1; i <= 21; i++) {
    //initialise random seed again ...
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // to shuffle pair order ...
    shuffle(&HRTFComboOrd[0], &HRTFComboOrd[2], std::default_random_engine(seed));
    // then assign each HRTF to relevant slot.
    gComparMatches[i][0] = HRTFCombos[HRTFComboIdx[i-1]][HRTFComboOrd[0]];
    gComparMatches[i][1] = HRTFCombos[HRTFComboIdx[i-1]][HRTFComboOrd[1]];
    // print for reference
    rt_printf("Pair %i is: %i and %i \n", i, gComparMatches[i][0],gComparMatches[i][1]);
  }
  rt_printf("\n");
}

// Random location generator funciton
void createLocations(){
  //initialise random seed
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  //shuffle location combination order
  shuffle(&LocationComboIdx[0], &LocationComboIdx[20], std::default_random_engine(seed));
  //loop through shuffled location combination order and azi/ele values to array
  for (int i = 2; i <= 21; i++) {
    gLocationTrials[i][0] = LocationCombos[LocationComboIdx[i-2]][0];
    gLocationTrials[i][1] = LocationCombos[LocationComboIdx[i-2]][1];
    // print for reference
    rt_printf("Location %i is: %i and %i \n", i, gLocationTrials[i][0], \
      gLocationTrials[i][1]);
  }
  //repeat for second half
  seed = std::chrono::system_clock::now().time_since_epoch().count();
  shuffle(&LocationComboIdx[0], &LocationComboIdx[20], std::default_random_engine(seed));
  for (int i = 22; i <= 41; i++) {
    gLocationTrials[i][0] = LocationCombos[LocationComboIdx[i-22]][0];
    gLocationTrials[i][1] = LocationCombos[LocationComboIdx[i-22]][1];
    // print for reference
    rt_printf("Location %i is: %i and %i \n", i, gLocationTrials[i][0], \
      gLocationTrials[i][1]);
  }
  rt_printf("\n");
}

// Pick the maximum and minimum scoring HRTF (randomly for either if tied)
void getWinnerAndLoser(){
  int maxScore=0;                     // stores max occuring score
  int minScore=6;                     // stores min occurring score
  int numMaxScores=0;                 // stores count of max occurring score
  int numMinScores=0;                 // stores count of min occurring score

  // loop through selections to find max/min score and increment counts of each
  for (int i=0; i<7; i++) {
      if(gHRTFResponseCount[i]==maxScore) numMaxScores++;
      else if(gHRTFResponseCount[i]>maxScore){
        maxScore=gHRTFResponseCount[i];
        numMaxScores=1;
      }
      if(gHRTFResponseCount[i]==minScore) numMinScores++;
      else if(gHRTFResponseCount[i]<minScore){
        minScore=gHRTFResponseCount[i];
        numMinScores=1;
      }
  }
  // initialise random function against time for max element selection
  srand (time(NULL));
  int maxIndex=rand() % numMaxScores + 1;
  // initialise random function against time for min element selection
  srand (time(NULL));
  int minIndex=rand() % numMinScores + 1;

  // loop through HRTFs and find the max/min scores of the specified index
  for (int i=0, maxCount=0, minCount=0; i<7; i++){
    if(gHRTFResponseCount[i]==maxScore) {
      maxCount++;
      if(maxCount==maxIndex) gWinningHRTF=i;
    }
    if(gHRTFResponseCount[i]==minScore) {
      minCount++;
      if(minCount==minIndex) gLosingHRTF=i;
    }
  }

}

// write HRTF test output data to .csv file
void writeHRTFResponses(){
  // Instantiate time class
  time_t thisTime;

  // Declare variable to store localtime()
  struct tm * currentTimeHere;

  // Applying time()
  time (&thisTime);

  // Using localtime()
  currentTimeHere = localtime(&thisTime);

  // write filename and data
  cout << "Current Day, Date and Time is = "
       << asctime(currentTimeHere);
  std::string timeStamp = asctime(currentTimeHere);
  std::replace(timeStamp.begin(), timeStamp.end(), ' ', '_');
  std::replace(timeStamp.begin(), timeStamp.end(), '\n', '_');
  std::ofstream HRTFAns("/root/Bela/projects/VBAP_STUDY_LOGS/HRTF_Responses_"+ \
    timeStamp+".csv");
  HRTFAns << "Trial,HRTF A,HRTF B,Response,Time" << '\n';
  for (int i = 0; i < 22; i++) {
    HRTFAns << i << ',';
    HRTFAns << gComparMatches[i][0]<<',';
    HRTFAns << gComparMatches[i][1]<<',';
    HRTFAns << gHRTFResponses[i]<<',';
    HRTFAns << gHRTFResponseTimes[i]<<','<<'\n';
  }
}

// write Localisation test output data to .csv file
void writeLocationResponses(){
  // Instantiate time class
  time_t thisTime;

  // Declare variable to store localtime()
  struct tm * currentTimeHere;

  // Applying time()
  time (&thisTime);

  // Using localtime()
  currentTimeHere = localtime(&thisTime);

  // write filename and data
  cout << "Current Day, Date and Time is = "
       << asctime(currentTimeHere);
  std::string timeStamp = asctime(currentTimeHere);
  std::replace(timeStamp.begin(), timeStamp.end(), ' ', '_');
  std::replace(timeStamp.begin(), timeStamp.end(), '\n', '_');
  std::ofstream LocAns("/root/Bela/projects/VBAP_STUDY_LOGS/Loc_Responses_"+ \
    timeStamp+".csv");
  LocAns << "Trial,Target Azi,Target Ele, Response Azi,Response Ele, \
    HRTFState, HRTFNum, Time" << '\n';
  for (int i = 0; i < 42; i++) {
    LocAns << i-1 << ',';
    LocAns << gLocationTrials[i][0]<<',';
    LocAns << gLocationTrials[i][1]<<',';
    LocAns << gLocalisationResponses[i][0]<<',';
    LocAns << gLocalisationResponses[i][1]<<',';
    LocAns << gLocalisationHRTFState[i]<<',';
    LocAns << gLocalisationHRTF[i]<<',';
    LocAns << gLocalisationResponseTimes[i]<<','<<'\n';
  }
}

#endif /* ABROUTINE_ */
