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

SteeringBehaviourArrive::SteeringBehaviourArrive(float slowRadius, float targetRadius) :
	SlowRadius(slowRadius),
	TargetRadius((targetRadius))
{
}

SteeringOutput SteeringBehaviourArrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	double distance { (Target.Position - Agent.GetPosition()).Length() };
	double speed = std::clamp((distance - TargetRadius)/(SlowRadius-TargetRadius), 0.0, 1.0) * m_MaxSpeed;
	Agent.SetMaxLinearSpeed(speed);
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	
	// Add debug stuff
	
	return steering;
}

SteeringOutput SteeringBehaviourPursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	double distance { (Target.Position - Agent.GetPosition()).Length() };
	FVector2D predictedPosition = (distance/Agent.GetMaxLinearSpeed()) * Target.LinearVelocity + Target.Position;
	
	steering.LinearVelocity = predictedPosition - Agent.GetPosition();
	
	
	// Add debug stuff
	
	return steering;
}
