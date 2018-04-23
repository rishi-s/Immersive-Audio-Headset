/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 */

#ifndef StreamGainDATA_H_
#define StreamGainDATA_H_

// A matrix of gains values to balance and mute audio streams for demo purposes
float gStreamGains[5][10]={
	{0.0,0.0,0.36,0.0,0.0,0.0,0.0,0.2,0.0,0.0}, // one track (centre)
	{0.0,0.51,0.0,0.27,0.0,0.0,0.2,0.0,0.2,0.0}, // two tracks (front L/R)
	{0.39,0.0,0.36,0.0,0.36,0.2,0.0,0.2,0.0,0.2}, // three tracks (centre & back L/R)
	{0.39,0.51,0.0,0.27,0.36,0.2,0.2,0.0,0.2,0.2}, // four tracks (no centre)
	{0.39,0.51,0.36,0.27,0.36,0.2,0.2,0.2,0.2,0.2} // five tracks (all)
};

#endif /* StreamGainDATA_H_ */
