#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput SteeringBehaviourSeek::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	// Add debug stuff
	
	return steering;
}


SteeringOutput SteeringBehaviourFlee::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	steering.LinearVelocity = -(Target.Position - Agent.GetPosition());
	
	// Add debug stuff
	
	return steering;
}
