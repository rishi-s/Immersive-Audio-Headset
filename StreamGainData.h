/*
 *
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 */

#ifndef StreamGainDATA_H_
#define StreamGainDATA_H_
// Rows 673, 688, 694, 700, 715, 819, 834, 840, 846, 861
float gStreamGains[5][10]={
	{0.0,0.0,0.18,0.0,0.0,0.0,0.0,0.135,0.0,0.0}, // one track (centre)
	{0.0,0.255,0.0,0.135,0.0,0.0,0.135,0.0,0.135,0.0}, // two tracks (front L/R)
	{0.195,0.0,0.18,0.0,0.18,0.135,0.0,0.135,0.0,0.135}, // three tracks (centre & back L/R)
	{0.195,0.255,0.0,0.135,0.18,0.135,0.135,0.0,0.135,0.135}, // four tracks (no centre)
	{0.195,0.255,0.18,0.135,0.18,0.135,0.135,0.135,0.135,0.135} // five tracks (all)
};


#endif /* StreamGainDATA_H_ */
