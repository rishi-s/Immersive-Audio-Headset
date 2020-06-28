/*
 *  Created on: 21 April, 2018
 *      Author: Victor Zappi
 ***** Code extended and adapted from Bela SampleLoader example *****
 */

#ifndef SAMPLEDATA_H
#define SAMPLEDATA_H

#include <Bela.h>

// User defined structure to pass between main and render complex data retrieved from file
struct SampleData {
	float *samples;	// Samples in file
	int sampleLen;	// Total num of samples
};

#endif /* SAMPLEDATA_H */
