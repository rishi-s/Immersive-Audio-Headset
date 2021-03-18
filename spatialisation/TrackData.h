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


string taskOne[30];               // string array to hold track filenames

// function to extraxt track names from .csv
void getTaskTracks()
{
  // open the VBAP gain array .csv
  std::ifstream file("tracks/TaskTracks.csv");

  // loop through each row
  for(int row = 0; row <30; row++)
  {
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


#endif /* TRACKDATA_H_ */
