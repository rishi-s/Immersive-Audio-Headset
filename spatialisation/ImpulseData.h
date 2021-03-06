/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 *****  Code extended and adapted from QMUL ECS732P module content *****
 */

#ifndef IMPULSEDATA_H_
#define IMPULSEDATA_H_

#include "SpatialSceneParams.h" // definition of audio sources and context

// User defined structure to pass between main and render complex data retrieved from file
struct ImpulseData {
	float *samples;	// Samples in file
	int sampleLen;	// Total num of samples
};







#endif /* IMPULSEDATA_H_ */
