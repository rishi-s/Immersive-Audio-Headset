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

float gVBAPGains[65341][8]={};

void getVBAPMatrix()
{

    std::ifstream file("VBAPArray.csv");

    for(int row = 0; row < 65340; ++row)
    {
        std::string line;
        std::getline(file, line);
        if ( !file.good() )
            break;

        std::stringstream iss(line);

        for (int col = 0; col < 8; ++col)
        {
            std::string val;
            std::getline(iss, val, ',');
            if ( iss.bad() )
                break;

            std::stringstream convertor(val);
            convertor >> gVBAPGains[row][col];
        }
    }
    std::cout << "Test output: " << gVBAPGains[32670][1];
}


#endif /* VBAPDATA_H_ */
