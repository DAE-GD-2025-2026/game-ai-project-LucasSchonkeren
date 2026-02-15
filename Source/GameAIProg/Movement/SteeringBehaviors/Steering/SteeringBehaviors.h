#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }

	template <class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{
		return static_cast<T*>(this);
	}

protected:
	FTargetData Target;
};

class SteeringBehaviourSeek final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	SteeringBehaviourSeek() = default;

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class SteeringBehaviourFlee final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	SteeringBehaviourFlee() = default;

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class SteeringBehaviourArrive final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	SteeringBehaviourArrive(float slowRadius, float targetRadius);

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
private: //-------------- Fields ----------------------------

const float BaseMaxSpeed{500};
const float SlowRadius{20};
const float TargetRadius{20};

float m_MaxSpeed{BaseMaxSpeed};

};

class SteeringBehaviourPursuit final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	SteeringBehaviourPursuit() = default;

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class SteeringBehaviourEvade final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	SteeringBehaviourEvade() = default;

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};