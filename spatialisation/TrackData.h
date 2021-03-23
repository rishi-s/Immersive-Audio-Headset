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


// variables from OSC.h
extern int gSceneTracks[];
extern int gPlaylistCounter;
extern int gEnd;
extern int gLastRemovedTrack;

// variables from SpatialFocus.h
extern int gCurrentTargetSong;

string taskOne[30];          // string array to hold track filenames
std::vector<int> gDynamicPlaylist (30);
std::vector<int> gDecisionList;
std::vector<int> gFirstPosition;
std::vector<float> gFirstTime;
std::vector<int> gLastPosition;
std::vector<float> gLastTime;
std::vector<int> gAppearances;
std::vector<bool> gSelectionStatus;
std::vector<float> gActiveListen;
std::vector<float> gBackgroundListen;


// instantialte auxiliary task to reorder dynamic playlist
AuxiliaryTask gReorderPlaylist;
void gRemoveTrack_background(void *);



// function to extraxt track names from .csv
void getTaskTracks(){
  // open the VBAP gain array .csv
  std::ifstream file("tracks/TaskTracks.csv");

  // loop through each row
  for(int row = 0; row <30; row++)
  {
    gDynamicPlaylist[row]=row;
    // check if file data is readable
    if ( !file.good() )
    break;
    // extract the line as a string and assign to array
    std::string line;
    std::getline(file, line);
    std::stringstream convertor(line);
    convertor>>taskOne[row];
    rt_printf("%i = %s; %i\n", row, taskOne[row].c_str(), gDynamicPlaylist[row]);
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


void gRemoveTrack(int removedTrack){
  // find location of rejected track in the dynamic playlist and remove
  for(int track=0; track<gDynamicPlaylist.size(); track++){
      if(gDynamicPlaylist[track]==removedTrack){
        gDynamicPlaylist.erase(gDynamicPlaylist.begin()+track);
      }
      // print each track number in the dynamic playlist
      rt_printf("%i,",gDynamicPlaylist[track]);
  }
  // revise the dynamic playlist size
  gEnd=gDynamicPlaylist.size();
  // decrement the playlist counter and if we've reached the beginning, wrap it
  if(--gPlaylistCounter<0)gPlaylistCounter=gEnd-1;
  rt_printf("\nReset Counter = %i; Reset Range = %i \n",	\
    gPlaylistCounter, gDynamicPlaylist.size());
}

// function to resize dynamic playlist in background
void gRemoveTrack_background(void *){
  gRemoveTrack(gLastRemovedTrack);
}


// funciton to write the playlist interaction log
void writePlaylistLog(){
  std::ofstream Log("PlaylistLog.csv");
  Log << "Track,Pos1,Time1,Pos2,Time2,Appearances,Status,Active,Background" << '\n';
  for (int i = 0; i < gDecisionList.size(); i++) {
    Log << gDecisionList[i] << ',';
    Log << gFirstPosition[i] << ',';
    Log << gFirstTime[i] << ',';
    Log << gLastPosition[i] << ',';
    Log << gLastTime[i] << ',';
    Log << gAppearances[i] << ',';
    Log << gSelectionStatus[i] << ',';
    Log << gActiveListen[i] << ',';
    Log << gBackgroundListen[i] << ',' << '\n';;
  }
  gDynamicPlaylist.clear();
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
