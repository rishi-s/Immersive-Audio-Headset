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

// state machine for current A/B selection
enum Selection {kNoneSelected, kASelected, kBSelected};
std::string gProgressText="Example";
std::string gPlaybackText=" ";
std::string gSubmitText=" ";
int gChoiceState=kNoneSelected;
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
int gComparMatches [22][2] = {};

int gHRTFResponses[22]={};

int gHRTFResponseCount[7]={};

// Random pair generator funciton
void createPairs(){


  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

  shuffle(&HRTFComboIdx[0], &HRTFComboIdx[21], std::default_random_engine(seed));
  for (int i = 1; i <= 21; i++) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    shuffle(&HRTFComboOrd[0], &HRTFComboOrd[2], std::default_random_engine(seed));
    gComparMatches[i][0] = HRTFCombos[HRTFComboIdx[i-1]][HRTFComboOrd[0]];
    gComparMatches[i][1] = HRTFCombos[HRTFComboIdx[i-1]][HRTFComboOrd[1]];
    rt_printf("Pair %i is: %i and %i \n", i, gComparMatches[i][0],gComparMatches[i][1]);
  }
  rt_printf("\n");
}

// write test output data to .csv file
void writeHRTFResponses(){
  // Instantiate time class
  time_t thisTime;

  // Declare variable to store localtime()
  struct tm * currentTimeHere;

  // Applying time()
  time (&thisTime);

  // Using localtime()
  currentTimeHere = localtime(&thisTime);

  cout << "Current Day, Date and Time is = "
       << asctime(currentTimeHere);
  std::string timeStamp = asctime(currentTimeHere);
  std::ofstream HRTFAns("HRTF_Responses_"+timeStamp+".csv");
  HRTFAns << "Trial,HRTF A,HRTF B,Response,Time" << '\n';
  for (int i = 0; i < 22; i++) {
    HRTFAns << i << ',';
    HRTFAns << gComparMatches[i][0]<<',';
    HRTFAns << gComparMatches[i][1]<<',';
    HRTFAns << gHRTFResponses[i]<<',';
    HRTFAns << "Not Known," << '\n';
  }
}


#endif /* ABROUTINE_ */
