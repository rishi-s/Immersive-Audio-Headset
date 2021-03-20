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


string taskOne[30];          // string array to hold track filenames
std::vector<int> gDynamicPlaylist (sizeof(taskOne));
std::vector<int> gDecisionList;
std::vector<int> gScenePosition;
std::vector<bool> gSelectionStatus;
std::vector<float> gActiveListen;
std::vector<float> gBackgroundListen;


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
    rt_printf("%i = %s\n", row, taskOne[row].c_str());
  }
}

// function to update interaction log
void updatePlaylistLog(int track, int position, bool status, float actTime, \
  float bgTime){
    // assume the track is a new entry to the log
    bool existingTrack=false;
    // if the track is already in the log, update statuses in the relevant row
    for(int i=0; i < gDecisionList.size(); i++){
      if(gDecisionList[i]==track){
        gScenePosition[i]=position;
        gSelectionStatus[i]=status;
        gActiveListen[i]+=actTime;
        gBackgroundListen[i]+=bgTime;
        rt_printf("AMENDED at %i: %i, %d, %i, %f, %f \n", i, gDecisionList[i], \
        gScenePosition[i], gSelectionStatus[i], gActiveListen[i], \
        gBackgroundListen[i]);
        existingTrack=true;
        break;
      }
    }
    // otherwise, add a new line with default statuses
    if(existingTrack==false){
      gDecisionList.push_back(track);
      gScenePosition.push_back(position);
      gSelectionStatus.push_back(true);
      gActiveListen.push_back(0.0);
      gBackgroundListen.push_back(0.0);
      int j = gDecisionList.size()-1;
      rt_printf("ADDED at %i: %i, %i, %d, %f, %f \n", j, \
      gDecisionList[j], gScenePosition[j], \
      gSelectionStatus[j], \
      gActiveListen[j], \
      gBackgroundListen[j]);
    }
}

void writePlaylistLog(){
  std::ofstream Log("PlaylistLog.csv");
  Log << "Track,Position,Status,Active Listening,Background Listening" << '\n';
  for (int i = 0; i < gDecisionList.size(); i++) {
    Log << gDecisionList[i] << ',';
    Log << gScenePosition[i] << ',';
    Log << gSelectionStatus[i] << ',';
    Log << gActiveListen[i] << ',';
    Log << gBackgroundListen[i] << ',' << '\n';;
  }
  gDynamicPlaylist.clear();
  gDecisionList.clear();
  gScenePosition.clear();
  gSelectionStatus.clear();
  gActiveListen.clear();
  gBackgroundListen.clear();
}
#endif /* TRACKDATA_H_ */
