/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 */

#ifndef StreamGainDATA_H_
#define StreamGainDATA_H_

// A matrix of gains values to balance and mute audio streams for demo purposes
float gStreamGains[5][2][10]={
	{{0.4,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}, \
		{0.4,0.0,0.0,0.0,0.0,0.75,0.0,0.0,0.0,0.0}}, // one track (centre)
	{{0.4,0.8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}, \
		{0.4,0.8,0.0,0.0,0.0,0.75,0.75,0.0,0.0,0.0}}, // two tracks (centre and R)
	{{0.4,0.8,0.48,0.0,0.0,0.0,0.0,0.0,0.0,0.0}, \
	 	{0.4,0.8,0.48,0.0,0.0,0.75,0.75,0.75,0.0,0.0}},// three tracks (centre and L/R)
	{{0.4,0.8,0.48,0.64,0.0,0.0,0.0,0.0,0.0,0.0}, \
	 	{0.4,0.8,0.48,0.64,0.0,0.75,0.75,0.75,0.75,0.0}},// four tracks (no centre)
	{{0.4,0.8,0.48,0.64,0.64,0.0,0.0,0.0,0.0,0.0}, \
		{0.4,0.8,0.48,0.64,0.64,0.75,0.75,0.75,0.75,0.75}} // five tracks (all)
};

#endif /* StreamGainDATA_H_ */
