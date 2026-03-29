#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"
#include "Math/UnrealMathUtility.h"

// --- SEEK ---
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	steering.LinearVelocity.Normalize();

	if (RenderDebug)
	{
		FVector Start(Agent.GetPosition().X, Agent.GetPosition().Y, 90.0f);
		FVector End = Start + FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.0f) * 200.0f;

		DrawDebugLine(Agent.GetWorld(), Start, End, FColor::Green, false, 0.0f, 0, 2.0f);
	}
	
	return steering;
}

// --- FLEE ---
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	steering.LinearVelocity = -(Target.Position - Agent.GetPosition());
	steering.LinearVelocity.Normalize();

	if (RenderDebug)
	{
		FVector Start(Agent.GetPosition().X, Agent.GetPosition().Y, 90.0f);
		FVector End = Start + FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.0f) * 200.0f;

		DrawDebugLine(Agent.GetWorld(), Start, End, FColor::Red, false, 0.0f, 0, 2.0f);
	}
	
	return steering;
}

// --- ARRIVE ---
Arrive::Arrive(float slowRadius, float targetRadius) :
	SlowRadius(slowRadius),
	TargetRadius(targetRadius)
{
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	double distance = (Target.Position - Agent.GetPosition()).Length();
	double speed = FMath::Clamp((distance - TargetRadius)/(SlowRadius-TargetRadius), 0.0, 1.0) * Agent.GetMaxLinearSpeed();
	double oldSpeed = Agent.GetMaxLinearSpeed();
	Agent.SetMaxLinearSpeed(speed);
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	steering.LinearVelocity.Normalize();
	
	Agent.SetMaxLinearSpeed(oldSpeed);

	if (RenderDebug)
	{
		FVector Start(Agent.GetPosition().X, Agent.GetPosition().Y, 90.0f);
		FVector End = Start + FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.0f) * 200.0f;
		DrawDebugLine(Agent.GetWorld(), Start, End, FColor::Blue, false, 0.0f, 0, 2.0f);
	}

	return steering;
}

void Arrive::SetTargetRadius(float NewRadius) {
	TargetRadius = NewRadius;	
}	

// --- PURSUIT ---
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	double distance = (Target.Position - Agent.GetPosition()).Length();
	FVector2D predictedPosition = Target.Position + (distance / Agent.GetMaxLinearSpeed()) * Target.LinearVelocity;
	
	steering.LinearVelocity = predictedPosition - Agent.GetPosition();
	steering.LinearVelocity.Normalize();

	if (RenderDebug)
	{
		FVector Start(Agent.GetPosition().X, Agent.GetPosition().Y, 90.0f);
		FVector End = Start + FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.0f) * 200.0f;
		DrawDebugLine(Agent.GetWorld(), Start, End, FColor::Cyan, false, 0.0f, 0, 2.0f);

		// Draw predicted target
		FVector PredictedFVec(predictedPosition.X, predictedPosition.Y, 90.0f);
		DrawDebugSphere(Agent.GetWorld(), PredictedFVec, 20.0f, 12, FColor::Cyan, false, 0.0f);
	}

	return steering;
}

// --- EVADE ---
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	double distance = (Target.Position - Agent.GetPosition()).Length();
	FVector2D predictedPosition = Target.Position + (distance / Agent.GetMaxLinearSpeed()) * Target.LinearVelocity;
	
	steering.LinearVelocity = -(predictedPosition - Agent.GetPosition());
	steering.LinearVelocity.Normalize();
	
	if (distance > MinDistance) 
		steering.IsValid = false;

	if (RenderDebug)
	{
		FVector Start(Agent.GetPosition().X, Agent.GetPosition().Y, 90.0f);
		FVector End = Start + FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.0f) * 200.0f;
		DrawDebugLine(Agent.GetWorld(), Start, End, FColor::Orange, false, 0.0f, 0, 2.0f);

		FVector PredictedFVec(predictedPosition.X, predictedPosition.Y, 90.0f);
		DrawDebugSphere(Agent.GetWorld(), PredictedFVec, 20.0f, 12, FColor::Orange, false, 0.0f);
	}

	return steering;
}

// --- WANDER ---
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent) {
	SteeringOutput steering{};
	
	m_Offset += FMath::RandRange(-MaxDeviation, MaxDeviation);
	FVector2D wanderTarget{ cos(m_Offset) * Radius, sin(m_Offset) * Radius};
	wanderTarget += Agent.GetPosition();
	auto forwardVec = Agent.GetActorForwardVector();
	wanderTarget += WanderDistance * FVector2D{forwardVec.X, forwardVec.Y};

	steering.LinearVelocity = wanderTarget - Agent.GetPosition();
	steering.LinearVelocity.Normalize();

	if (RenderDebug)
	{
		FVector Start(Agent.GetPosition().X, Agent.GetPosition().Y, 90.0f);
		FVector End = Start + FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.0f) * 200.0f;
		DrawDebugLine(Agent.GetWorld(), Start, End, FColor::Purple, false, 0.0f, 0, 2.0f);

		FVector TargetFVec(wanderTarget.X, wanderTarget.Y, 90.0f);
		DrawDebugSphere(Agent.GetWorld(), TargetFVec, 15.0f, 8, FColor::Purple, false, 0.0f);
	}

	return steering;
}