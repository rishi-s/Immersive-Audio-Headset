/*
  ==============================================================================

    ScatteringDelay.cpp
    Created: 11 Mar 2019 6:12:29pm
    Author:  Chris Yeoward

  ==============================================================================
*/

#include "reverb/Network.h"

namespace SDN
{
	Network::Network(float sampleRate) : Network(sampleRate, 5.0, 5.0, 3.0) {}

	Network::Network(float sampleRate, float width, float length, float height)
	{
		// define wall positions
		bounds[0] = SDN::Boundary(0.0, SDN::Plane::YZ);
		bounds[1] = SDN::Boundary(width, SDN::Plane::YZ);
		bounds[2] = SDN::Boundary(0.0, SDN::Plane::XZ);
		bounds[3] = SDN::Boundary(length, SDN::Plane::XZ);
		// bounds[4] = SDN::Boundary(0.0, SDN::Plane::XY);
		//bounds[5] = SDN::Boundary(height, SDN::Plane::XY);

		// calculate number of connections and initialise node positions
		for(int node = 0; node < nodeCount; node++)
		{
			connectionCount += node;
			nodes[node] = SDN::Node(bounds[node].getScatteringNodePosition(mic, source), delayOrder);
		}

		connections = new SDN::Connection[connectionCount];

		// initialise connections between nodes, defining lengths and providing terminals to the nodes to interact with
		int connection = 0;
		for(int node = 0; node < nodeCount - 1; node++)
		{
			for(int otherNode = node + 1; otherNode < nodeCount; otherNode++)
			{
				connections[connection] = SDN::Connection(nodes[node], nodes[otherNode], sampleRate);
				nodes[node].addTerminal(connections[connection].getStartTerminal());
				nodes[otherNode].addTerminal(connections[connection].getEndTerminal());
				connection++;
			}
		}

		// initialised the source to node and node to mic delay lines
		for(int node = 0; node < nodeCount; node++)
		{
			sourceToNodeDelays[node] = *Delay::fromDistance(sampleRate, source.distanceTo(nodes[node].getPosition()));
			nodeToMicDelays[node] = *Delay::fromDistance(sampleRate, mic.distanceTo(nodes[node].getPosition()));
		}

		//AMEND: sourceMicDelay is overriden by drySourcesToMicDelays
		//sourceMicDelay = Delay::fromDistance(sampleRate, source.distanceTo(mic));

//		setAbsorptionAmount(0.01);
		// small room
		nodes[0].setAbsorption(0.2);
		nodes[1].setAbsorption(0.2);
		nodes[2].setAbsorption(0.4);
		nodes[3].setAbsorption(0.3);
		//nodes[4].setAbsorption(0.4); //floor
		//nodes[5].setAbsorption(0.7); // ceiling
//		nodes[5].setAbsorption(0.4); // ceiling
	}

	// returns a stereo output of the direct line between the source and mic, based on their relative positions
	StereoOutput Network::positionSource(float sourceInput)
	{
		sourceMicDelay->write(sourceInput);

		StereoOutput out;

		float fromSource = sourceMicDelay->read() / source.distanceTo(mic); // get value from delay line and attenuate by 1/r
		float sourceAzimuth = source.azimuthFrom(mic);
		float sinAzimuth = sin(sourceAzimuth);
		float denom = sqrt(2 * (1 + pow(sinAzimuth, 2)));
		float sourceGainLeft = (1 - sinAzimuth) / denom;
		float sourceGainRight = (1 + sinAzimuth) / denom;

		out.L = fromSource * sourceGainLeft;
		out.R = fromSource * sourceGainRight;

		return out;
	}


	// returns a stereo signal of a the reverberation.
	StereoOutput Network::scatterStereo(float in)
	{
		scatter(in);

		StereoOutput out;
		out.L = 0.0;
		out.R = 0.0;

		for(int node = 0; node < nodeCount; node++)
		{
			float fromNode = nodeToMicDelays[node].readWithDistanceAttenuation();
//			fromNode /= (mic.distanceTo(nodes[node].getPosition())); // get value from delay line and attenuate by 1/r
			float nodeAzimuth = nodes[node].getPosition().azimuthFrom(mic);
			float sinAzimuth = sin(nodeAzimuth);
			float denom = 1 / sqrt(2 * (1 + pow(sinAzimuth, 2)));

			float nodeGainLeft = (1 - sinAzimuth) * denom; // calculate left gain
			float nodeGainRight = (1 + sinAzimuth) * denom; // calculate right gain

			out.L += fromNode * nodeGainLeft;
			out.R += fromNode * nodeGainRight;
		}

		return out;
	}


	// method for scattering only in mono
	// AMEND: Using a combined input signal from several sources
	float Network::scatterMono(float in)
	{
		scatter(in);

		/*AMEND: Do not create a source-to-mic delay of the combined input signal*/
		auto out = 0.0; /*sourceMicDelay->readWithDistanceAttenuation(); // get value from delay line and attenuate by 1/r*/

		for(int node = 0; node < nodeCount; node++)
		{
			out += nodeToMicDelays[node].readWithDistanceAttenuation();
		}
		return out;
	}

	void Network::process(float in, float* output) {
		scatter(in);

		output[0] = sourceMicDelay->readWithDistanceAttenuation(); // get value from delay line and attenuate by 1/r
		for(int node = 0; node < nodeCount; node++)
		{
			output[node + 1] = sourceToNodeDelays[node].getDelayDistance() * nodeToMicDelays[node].readWithDistanceAttenuation(sourceToNodeDelays[node].getDelayDistance());
		}
	}

	// main reverberation method
	void Network::scatter(float in)
	{
		//AMEND: sourceMicDelay not active; input is combined source
		//sourceMicDelay->write(in);

		// for each node, gather inputs
		for(int node = 0; node < nodeCount; node++)
		{
			sourceToNodeDelays[node].write(in);
			nodes[node].scatter(sourceToNodeDelays[node].readWithDistanceAttenuation()); // add distance attenuation
			float nodeOut = nodes[node].getNodeOutput();
			nodeToMicDelays[node].write(nodeOut);
		}
	}

	void Network::setSourcePosition(float x, float y, float z) {
		source.setX(x);
		source.setY(y);
		source.setZ(z);

		updateNodePositions();
	}

	float Network::addDrySource(int sampleRate, float x, float y, float z, int sourceIndex) {
		drySources[sourceIndex].setX(x);
		drySources[sourceIndex].setY(y);
		drySources[sourceIndex].setZ(z);
		drySourceToMicDelays[sourceIndex] = *Delay::fromDistance(sampleRate, drySources[sourceIndex].distanceTo(mic));
		return drySourceToMicDelays[sourceIndex].getDelayDistance();
	}

	void Network::writeToDrySource(float in, int sourceIndex){
		drySourceToMicDelays[sourceIndex].write(in);
	}

	float Network::readFromDrySource(int sourceIndex){
		float out=drySourceToMicDelays[sourceIndex].readWithDistanceAttenuation();
		return out;
	}

	void Network::setMicPosition(float x, float y, float z) {
		mic.setX(x);
		mic.setY(y);
		mic.setZ(z);

		updateNodePositions();
	}


	// update lengths if mic or source position have changed
	void Network::updateNodePositions()
	{
		for(int node = 0; node < nodeCount; node++)
		{
			Point pos = bounds[node].getScatteringNodePosition(mic, source);
			nodes[node].setPosition(bounds[node].getScatteringNodePosition(mic, source));
		}

		int connection = 0;
		for(int node = 0; node < nodeCount - 1; node++)
		{
			for(int otherNode = node + 1; otherNode < nodeCount; otherNode++)
			{
				connections[connection].setLength(nodes[node].getPosition().distanceTo(nodes[otherNode].getPosition()));
				connection++;
			}
		}

		for(int node = 0; node < nodeCount; node++)
		{
			sourceToNodeDelays[node].setDelayLengthFromDistance(source.distanceTo(nodes[node].getPosition()));
			nodeToMicDelays[node].setDelayLengthFromDistance(mic.distanceTo(nodes[node].getPosition()));
		}
		//AMEND: sourceMicDelay not active; input is combined source
		//sourceMicDelay->setDelayLengthFromDistance(source.distanceTo(mic));
	}

	void Network::setAbsorptionAmount(const float amount) {
		for(int node = 0; node < nodeCount; node++)
		{
			nodes[node].setAbsorption(amount);
		}
	}

	void Network::getNodeAzimuths(int* azimuths) {
		azimuths[0] = getSourceAzimuth();

		for (int node = 0; node < nodeCount; node++) {
			azimuths[node + 1] = getNodeAzimuth(node);
		}
	}

	void Network::getNodeElevations(int* elevations) {
		elevations[0] = getSourceElevation();

		for (int node = 0; node < nodeCount; node++) {

			elevations[node + 1] = getNodeElevation(node);
		}
	}

	// gets the node azimuths
	float Network::getNodeAzimuth(int node) {
		return nodes[node].getPosition().azimuthFrom(mic) * (180 / M_PI);
	}

	float Network::getSourceAzimuth() {
		return source.azimuthFrom(mic) * (180 / M_PI);
	}

	float Network::getNodeElevation(int node) {
		return nodes[node].getPosition().elevationFrom(mic) * (180 / M_PI);
	}

	float Network::getSourceElevation() {
		return source.elevationFrom(mic) * (180 / M_PI);
	}
}
