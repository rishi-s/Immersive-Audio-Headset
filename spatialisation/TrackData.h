/*
*  Created on: 16 March, 2021
*      Author: Rishi Shukla
*  Code adapted from http://www.cplusplus.com/forum/unices/112048/
*/

#ifndef TRACKDATA_H_
#define TRACKDATA_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include <unordered_map>

#define NUM_TASKS 5


// variables from render.cpp
extern void loadAudioFiles();
extern void startPlayback(int stream);
extern int gPlaybackState;

// variables from OSC.h
extern int gTaskCounter;
extern float gTimeCounter;
extern int gPlaylistCounter;
extern int gRejectCounter;
extern int gLastRemovedTrack;
extern int gSceneTracks[];
extern int gEnd;
extern bool gPauseState;
extern string gPauseText;

// variables from SpatialFocus.h
extern int gCurrentTargetSong;

// arrays for task variables
int gTaskLengths[NUM_TASKS];          // number of tracks in each task playlist
int gTaskAnswers[NUM_TASKS];          // number of answers required of each task
std::string gTaskText[NUM_TASKS];     // text descriptions for task requirement


// vectors for playlist interaction
std::vector <string> gTaskList[NUM_TASKS];    // track titles
std::vector<int> gDynamicPlaylist[NUM_TASKS]; // dynamic playlist
std::vector<int> gDecisionList;               // responses
std::vector<int> gFirstPosition;              // track start position
std::vector<float> gFirstTime;                // track start time
std::vector<int> gLastPosition;               // track end position
std::vector<float> gLastTime;                 // track end time
std::vector<int> gAppearances;                // track appearances
std::vector<bool> gSelectionStatus;           // track status
std::vector<float> gActiveListen;             // active listening duration
std::vector<float> gBackgroundListen;         // background listening duration


// instantialte auxiliary task to reorder dynamic playlist
AuxiliaryTask gReorderPlaylist;
AuxiliaryTask gLoadPlaylist;
AuxiliaryTask gWriteResponses;

void removeTrack_background(void *);
void loadAudioFiles_background(void *);
void writePlaylistLog(void *);


// function to extract track names from .csv
void getTaskTracks(){
  std::ifstream list("tracks/TaskList.csv");
  std::string task;

  int question=0;

  if (list.good() ){
    // read consecutive lines in a loop
    while(getline(list, task)){
      //convert the next line to a readable string and store in a variable
      std::stringstream convertor(task);
      // add the strings to the array and increment index
      gTaskText[question]=task;
      rt_printf("%i = %s\n", question, gTaskText[question].c_str());
      question++;
    }
  }


  for (int i=0; i<NUM_TASKS; i++){
    // open the track list .csv
    std::ifstream file("tracks/Task" + to_string(i+1) + "Tracks.csv");

    // create a string object to read each line
    std::string line;

    int index=0;
    // check if file data is readable
    if (file.good() ){
      // read consecutive lines in a loop
      while(getline(file, line)){
        //convert the next line to a readable string and store in a variable
        std::stringstream convertor(line);
        string trackTitle;
        convertor>> trackTitle;
        // add the strings to the required vectors and increment index
        gTaskList[i].push_back(trackTitle);
        gDynamicPlaylist[i].push_back(index);
        rt_printf("%s = %i\n", gTaskList[i][index].c_str(), \
          gDynamicPlaylist[i][index]);
        index++;
      }
    }
    // log the task length and answer requirement
    gTaskLengths[i]=index+1;
    gTaskAnswers[i]=(index+1)/2;
  }
}

// function to update interaction log
void updatePlaylistLog(int track, int position, float timeStamp){
    // assume the track is a new entry to the log
    bool existingTrack=false;
    // if the track is already in the log, update statuses in the relevant row
    for(int i=0; i < gDecisionList.size(); i++){
      if(gDecisionList[i]==track){
        gLastPosition[i]=position;
        gLastTime[i]=timeStamp;
        gAppearances[i]++;
        /*rt_printf("AMENDED at %i: %i, %d, %i, %f, %f \n", i, gDecisionList[i], \
        gFirstPosition[i], gSelectionStatus[i], gActiveListen[i], \
        gBackgroundListen[i]);*/
        existingTrack=true;
        break;
      }
    }
    // otherwise, add a new line with default statuses
    if(existingTrack==false){
      gDecisionList.push_back(track);
      gFirstPosition.push_back(position);
      gFirstTime.push_back(timeStamp);
      gLastPosition.push_back(position);
      gLastTime.push_back(timeStamp);
      gAppearances.push_back(1);
      gSelectionStatus.push_back(true);
      gActiveListen.push_back(0.0);
      gBackgroundListen.push_back(0.0);
      //int j = gDecisionList.size()-1;
      /*rt_printf("ADDED at %i: %i, %i, %d, %f, %f \n", j, \
      gDecisionList[j], gFirstPosition[j], \
      gSelectionStatus[j], \
      gActiveListen[j], \
      gBackgroundListen[j]);*/
    }
}

// function to resize dynamic playlist
void removeTrack(int removedTrack){
  // find location of rejected track in the dynamic playlist and remove
  for(int track=0; track<gDynamicPlaylist[gTaskCounter-1].size(); track++){
      if(gDynamicPlaylist[gTaskCounter-1][track]==removedTrack){
        gDynamicPlaylist[gTaskCounter-1].erase(gDynamicPlaylist[gTaskCounter-1].begin()+track);
      }
      // print each track number in the dynamic playlist
      //rt_printf("%i,",gDynamicPlaylist[gTaskCounter-1][track]);
  }
  // revise the dynamic playlist size
  gEnd=gDynamicPlaylist[gTaskCounter-1].size();
  // if the task is complete
  if(gEnd==gTaskAnswers[gTaskCounter-1]){
    //rt_printf("** CHANGE! **\n");
    startPlayback(10);
    gPauseState=true;
    gPauseText="CONTINUE STUDY";
    // and if we still have tasks left to do
    if(++gTaskCounter<=NUM_TASKS) {
      // write the final responses
      Bela_scheduleAuxiliaryTask(gWriteResponses);
      // load the next set of audio
      Bela_scheduleAuxiliaryTask(gLoadPlaylist);
      //rt_printf("Load command issued.");
      // reset all task-related variables
      gTimeCounter=0;
      gPlaylistCounter=5;
      gRejectCounter=0;
      gLastRemovedTrack=0;
      for(int song=0;song<5;song++){gSceneTracks[song]=song;}
      gEnd=gDynamicPlaylist[gTaskCounter-1].size();

    }
    // otherwise stop the audio and write final responses
    else{
      gTaskCounter=NUM_TASKS+1;
      gPauseState=true;
      gPauseText="THANK YOU";
      //rt_printf("Stop Now");
      Bela_scheduleAuxiliaryTask(gWriteResponses);
      //rt_printf("Stopped");
    }
  }
  // decrement the playlist counter and if we've reached the beginning, wrap it
  if(--gPlaylistCounter<0)gPlaylistCounter=gEnd-1;
  //rt_printf("\nReset Counter = %i; Reset Range = %i \n",	\
    gPlaylistCounter, gDynamicPlaylist[gTaskCounter-1].size());
}


// function to resize dynamic playlist in background
void loadAudioFiles_background(void *){
  loadAudioFiles();
}


// function to resize dynamic playlist in background
void removeTrack_background(void *){
  removeTrack(gLastRemovedTrack);
}


// funciton to write the playlist interaction log
void writePlaylistLog(void *){

  // Instantiate time class
  time_t thisTime;

  // Declare variable to store localtime()
  struct tm * currentTimeHere;

  // Applying time()
  time (&thisTime);

  // Using localtime()
  currentTimeHere = localtime(&thisTime);

  // write filename and data
  //cout << "Current Day, Date and Time is = "
       //<< asctime(currentTimeHere);
  std::string timeStamp = asctime(currentTimeHere);
  std::replace(timeStamp.begin(), timeStamp.end(), ' ', '_');
  std::replace(timeStamp.begin(), timeStamp.end(), '\n', '_');
  std::ofstream Log("/root/Bela/projects/BROWSER_STUDY_LOGS/" + timeStamp + \
    "_PlaylistLog" + to_string(gTaskCounter-1) + ".csv");

  // write the log

  Log << "Track,Name,Pos1,Time1,Pos2,Time2,Appearances,Status,Active,Background" << '\n';
  for (int i = 0; i < gDecisionList.size(); i++) {
    Log << gDecisionList[i] << ',';
    Log << gTaskList[gTaskCounter-2][gDecisionList[i]] << ',';
    Log << gFirstPosition[i] << ',';
    Log << gFirstTime[i] << ',';
    Log << gLastPosition[i] << ',';
    Log << gLastTime[i] << ',';
    Log << gAppearances[i] << ',';
    Log << gSelectionStatus[i] << ',';
    Log << gActiveListen[i] << ',';
    Log << gBackgroundListen[i] << ',' << '\n';;
  }
  // clear the vectors
  gDecisionList.clear();
  gFirstPosition.clear();
  gFirstTime.clear();
  gLastPosition.clear();
  gLastTime.clear();
  gAppearances.clear();
  gSelectionStatus.clear();
  gActiveListen.clear();
  gBackgroundListen.clear();
}
#endif /* TRACKDATA_H_ */
