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
	out.LinearVelocity.Normalize();
	
	return out;
}

//*********************
//SEPARATION (FLOCKING)

SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput out{};
	
	auto neighbours {pFlock->GetNeighbors()};
	
	for (auto& neighbour : neighbours) {
		if (neighbour) 
			out.LinearVelocity += pAgent.GetPosition() - neighbour->GetPosition();
	}
	
	out.LinearVelocity.Normalize();
	
	return out;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput Alignmment::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput out{};

	out.LinearVelocity = pFlock->GetAverageNeighborVelocity();
	out.LinearVelocity.Normalize();

	return out;
}