/*
*  Created on: 21 April, 2018
*      Author: Rishi Shukla
*  Code adapted from http://www.cplusplus.com/forum/unices/112048/
*/

#ifndef VBAPDATA_H_
#define VBAPDATA_H_


#include <iostream>
#include <fstream>
#include <sstream>

float gVBAPGains[65341][8]={};      //2D array to hold VBAP gains


// function to extraxt VBAP matrix from .csv
void getVBAPMatrix()
{
  // open the VBAP gain array .csv
  std::ifstream file("spatialisation/VBAPArray.csv");

  // loop through each row
  for(int row = 0; row < 65340; ++row)
  {
    // extract the line as a string and check it is valid
    std::string line;
    std::getline(file, line);
    if ( !file.good() )
    break;

    // interrogate the line as a string
    std::stringstream iss(line);

    // loop through the .csv columns to find integers
    for (int col = 0; col < 8; ++col)
    {
      std::string val;
      std::getline(iss, val, ',');
      if ( iss.bad() )
      break;

      // assign integers to the corresponding array location
      std::stringstream convertor(val);
      convertor >> gVBAPGains[row][col];
    }
  }
  // print a value to test the output
  std::cout << "Test output: " << gVBAPGains[32670][1] << "\n";
}


#endif /* VBAPDATA_H_ */
