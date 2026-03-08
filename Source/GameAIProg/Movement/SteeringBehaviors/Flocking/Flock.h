#pragma once

// Toggle this define to enable/disable spatial partitioning
#define GAMEAI_USE_SPACE_PARTITIONING

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"
#ifdef GAMEAI_USE_SPACE_PARTITIONING
//#include "../SpacePartitioning/SpacePartitioning.h"
class CellSpace;
#endif

class Flock final
{
public:
	Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize = 10, 
	float WorldSize = 100.f, 
	ASteeringAgent* const pAgentToEvade = nullptr, 
	bool bTrimWorld = false);

	~Flock();

	void Tick(float DeltaTime);
	void RenderDebug();
	void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);
	
	static constexpr size_t maxNeighbours{100};
	
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	std::unique_ptr<CellSpace> pPartitionedSpace{};
	const std::array<ASteeringAgent*, maxNeighbours>& GetNeighbors() const;
	int GetNrOfNeighbors() const;
	TArray<FVector2D> OldPositions;
#else // No space partitioning
	void RegisterNeighbors(ASteeringAgent* const Agent);
	int GetNrOfNeighbors() const { return NrOfNeighbors; }
	//const TArray<ASteeringAgent*>& GetNeighbors() const { return Neighbors; }
	const std::array<ASteeringAgent*, maxNeighbours>& GetNeighbours() const { return neighbours; }
	
#endif // USE_SPACE_PARTITIONING

	FVector2D GetAverageNeighborPos() const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const & Target);

private:
	// For debug rendering purposes
	UWorld* pWorld{nullptr};
	
	int FlockSize{0};
	TArray<ASteeringAgent*> Agents{};
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	//std::unique_ptr<CellSpace> pPartitionedSpace{};
	//int NrOfCellsX{ 10 };
	//TArray<FVector2D> OldPositions{};
#else // No space partitioning
	//TArray<ASteeringAgent*> Neighbors{};
	std::array<ASteeringAgent*, maxNeighbours> neighbours{};
#endif // USE_SPACE_PARTITIONING
	
	float NeighborhoodRadius{100.f};
	int NrOfNeighbors{0};

	ASteeringAgent* pAgentToEvade{nullptr};
	
	//Steering Behaviors
	std::unique_ptr<Separation> pSeparationBehavior{std::make_unique<Separation>(this)};
	std::unique_ptr<Cohesion> pCohesionBehavior{std::make_unique<Cohesion>(this)};
	std::unique_ptr<Alignmment> pVelMatchBehavior{std::make_unique<Alignmment>(this)};
	std::unique_ptr<SteeringBehaviourSeek> pSeekBehavior{std::make_unique<SteeringBehaviourSeek>()};
	std::unique_ptr<SteeringBehaviourWander> pWanderBehavior{std::make_unique<SteeringBehaviourWander>()};
	std::unique_ptr<SteeringBehaviourEvade> pEvadeBehavior{std::make_unique<SteeringBehaviourEvade>()};
	
	std::unique_ptr<BlendedSteering> pBlendedSteering{std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>({
		BlendedSteering::WeightedBehavior{pCohesionBehavior.get(), 0}, 
		BlendedSteering::WeightedBehavior{pSeparationBehavior.get(), 0},
		BlendedSteering::WeightedBehavior{pVelMatchBehavior.get(), 0},
		BlendedSteering::WeightedBehavior{pSeekBehavior.get(), 0},
		BlendedSteering::WeightedBehavior{pWanderBehavior.get(), 0}
	}))};
	std::unique_ptr<PrioritySteering> pPrioritySteering{std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>{
		pEvadeBehavior.get(), pBlendedSteering.get()
	})};

	float CohesionWeight{0.35f};
	float SeparationWeight{0.25f};
	float AlignmentWeight{0.10f};
	float SeekWeight{0.20f};
	float WanderWeight{0.10f};

	// UI and rendering
	bool DebugRenderSteering{true};
	bool DebugRenderNeighborhood{true};
	bool DebugRenderPartitions{true};

	void RenderNeighborhood();
};
