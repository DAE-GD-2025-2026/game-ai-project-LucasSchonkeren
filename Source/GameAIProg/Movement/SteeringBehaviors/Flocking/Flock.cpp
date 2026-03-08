#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Misc/LowLevelTestAdapter.h"
#include "Shared/ImGuiHelpers.h"

#include <memory>

#include "Movement/SteeringBehaviors/SpacePartitioning/SpacePartitioning.h"

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);
	OldPositions.SetNum(FlockSize);

	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, 2000.f, 2000.f, 10, 10, 1000);
	
	for (int i = 0; i < Agents.Num(); ++i)
	{
		while (Agents[i] == nullptr)
		{
		double x = FMath::FRandRange(-1000.0, 1000.0);
		double y = FMath::FRandRange(-1000.0, 1000.0);
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector{x, y,90}, FRotator::ZeroRotator);
		}
		pPartitionedSpace->AddAgent(*Agents[i]);
		OldPositions[i] = Agents[i]->GetPosition();
	}
	
	
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
 // TODO: update the flock
 // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world
  
	pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = CohesionWeight;
	pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = SeparationWeight;
	pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight = AlignmentWeight;
	pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight = SeekWeight;
	pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight = WanderWeight;
	
	pBlendedSteering->GetWeightedBehaviorsRef()[0].pBehavior->RenderDebug = DebugRenderSteering;
	pBlendedSteering->GetWeightedBehaviorsRef()[1].pBehavior->RenderDebug = DebugRenderSteering;
	pBlendedSteering->GetWeightedBehaviorsRef()[2].pBehavior->RenderDebug = DebugRenderSteering;
	pBlendedSteering->GetWeightedBehaviorsRef()[3].pBehavior->RenderDebug = DebugRenderSteering;
	pBlendedSteering->GetWeightedBehaviorsRef()[4].pBehavior->RenderDebug = DebugRenderSteering;
	pEvadeBehavior->RenderDebug = DebugRenderSteering;
  
	pEvadeBehavior->SetTarget(FTargetData{pAgentToEvade->GetPosition()});
	pAgentToEvade->SetSteeringBehavior(pWanderBehavior.get());
  
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	for (int i = 0; i < Agents.Num(); i++)
	{
		OldPositions[i] = Agents[i]->GetPosition();
	}

	for (int i = 0; i < Agents.Num(); i++)
	{
		ASteeringAgent* agent = Agents[i];

		auto steering = pPrioritySteering->CalculateSteering(DeltaTime, *agent);

		agent->AddMovementInput(FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.f));

		pPartitionedSpace->UpdateAgentCell(*agent, OldPositions[i]);

		pPartitionedSpace->RegisterNeighbors(*agent, NeighborhoodRadius);
	}

	if (DebugRenderPartitions)
		pPartitionedSpace->RenderCells();

#else
// fallback without partitioning
	for (auto agent : Agents)
	{
		RegisterNeighbors(agent);
		auto steering = pPrioritySteering->CalculateSteering(DeltaTime, *agent);
		agent->AddMovementInput(FVector(steering.LinearVelocity.X, steering.LinearVelocity.Y, 0.f));
	}
#endif
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
 
	if (DebugRenderNeighborhood) RenderNeighborhood();
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (DebugRenderPartitions)
	{
		pPartitionedSpace->RenderCells();
	}
#endif
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		ImGui::Checkbox("Render Neighbourhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Render Steering", &DebugRenderSteering);
		ImGui::Checkbox("Render Partitions", &DebugRenderPartitions);

  // TODO: implement ImGUI sliders for steering behavior weights here
  
		ImGui::SliderFloat("Cohesion weight", &CohesionWeight, 0.f, 1.f);
		ImGui::SliderFloat("Alignment weight", &AlignmentWeight, 0.f, 1.f);
		ImGui::SliderFloat("Separation weight", &SeparationWeight, 0.f, 1.f);
		ImGui::SliderFloat("Seek weight", &SeekWeight, 0.f, 1.f);
		ImGui::SliderFloat("Wander weight", &WanderWeight, 0.f, 1.f);
		
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
	DrawDebugCircle(pWorld, FVector{Agents[0]->GetPosition().X, Agents[0]->GetPosition().Y, 0}, NeighborhoodRadius, 64, FColor::Green, false, -1.f, 0, 2.f, FVector(1,0,0), FVector(0,1,0), false);
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{ 
	NrOfNeighbors = 0;
	for (auto agent : Agents) {
		if (agent == pAgent) continue;
		if ((agent->GetPosition() - pAgent->GetPosition()).SizeSquared() < NeighborhoodRadius * NeighborhoodRadius) {
			neighbours[NrOfNeighbors] = agent;
			NrOfNeighbors += 1;
			if (NrOfNeighbors >= maxNeighbours) break;
		}
	}
}
#endif

const std::array<ASteeringAgent*, Flock::maxNeighbours>& Flock::GetNeighbors() const
{
	return pPartitionedSpace->GetNeighbors(); 
}

int Flock::GetNrOfNeighbors() const
{
	return pPartitionedSpace->GetNrOfNeighbors();
}

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	int count = GetNrOfNeighbors();
	if (count == 0) return FVector2D::ZeroVector;

	for (int i = 0; i < count; ++i) {
		avgPosition += GetNeighbors()[i]->GetPosition();
	}
	
	avgPosition /= count;
	
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	int count = GetNrOfNeighbors();
	if (count == 0) return FVector2D::ZeroVector;

	for (int i = 0; i < count; ++i) {
		avgVelocity += GetNeighbors()[i]->GetLinearVelocity();
	}
	
	avgVelocity /= count;

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	pSeekBehavior->SetTarget(Target);
}

