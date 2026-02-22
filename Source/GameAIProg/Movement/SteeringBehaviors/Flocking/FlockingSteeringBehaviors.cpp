#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput out{};
	
	out.LinearVelocity = pFlock->GetAverageNeighborPos() - pAgent.GetPosition();
	
	return out;
}

//*********************
//SEPARATION (FLOCKING)

SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput out{};
	
	auto neighbours {pFlock->GetNeighbours()};
	
	for (auto neighbour : neighbours) {
		out.LinearVelocity += pAgent.GetPosition() - neighbour->GetPosition();
	}
	
	return out;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput Alignmment::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput out{};

	out.LinearVelocity = pFlock->GetAverageNeighborVelocity();

	return out;
}