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

	
	bool RenderDebug{true};
protected:
	FTargetData Target;
};

class Seek : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	Seek() = default;

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
};

class Flee final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	Flee() = default;

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	
};

class Arrive final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	Arrive(float slowRadius = 40.f, float targetRadius = 10.f);

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	void SetTargetRadius(float NewRadius);
	
private: //-------------- Fields ----------------------------

 float BaseMaxSpeed{500};
 float SlowRadius{20};
 float TargetRadius{20};

float m_MaxSpeed{BaseMaxSpeed};

};

class Pursuit final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	Pursuit() = default;

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class Evade final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	Evade() = default;

public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	
	
private:
	float MinDistance{200.f};
};

class Wander final : public ISteeringBehavior
{
public: //------------ Constructor/Destructor --------------
	Wander() = default;
 
public: //--------------- Behaviour methods ----------------
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	
	
private: //-------------- Fields ----------------------------

	const double Radius{20};
	const double MaxDeviation{0.05};
	const double WanderDistance{5};
	
	double m_Offset{};
};