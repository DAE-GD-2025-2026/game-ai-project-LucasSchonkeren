#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


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

	for (int i = 0; i < Agents.Num(); ++i)
	{
		while (Agents[i] == nullptr)
		{
		double x = FMath::FRandRange(-1000.0, 1000.0);
		double y = FMath::FRandRange(-1000.0, 1000.0);
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector{x, y,90}, FRotator::ZeroRotator);
		}
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
  
	for (auto agent : Agents) {
		RegisterNeighbors(agent);
		auto steering {pBlendedSteering->CalculateSteering(DeltaTime, *agent)};
		agent->AddMovementInput(FVector{steering.LinearVelocity, 0.f});
	}
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
 
	RenderNeighborhood();
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

  // TODO: implement ImGUI sliders for steering behavior weights here
  
		ImGui::SliderFloat("Cohesion weight", &CohesionWeight, 0.f, 100.f);
		ImGui::SliderFloat("Alignment weight", &AlignmentWeight, 0.f, 100.f);
		ImGui::SliderFloat("Separation weight", &SeparationWeight, 0.f, 100.f);
		ImGui::SliderFloat("Seek weight", &SeekWeight, 0.f, 100.f);
		ImGui::SliderFloat("Wander weight", &WanderWeight, 0.f, 100.f);
		
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

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	for (int i{}; i < NrOfNeighbors; ++i) {
		avgPosition += neighbours.at(i)->GetPosition();
	}
	
	avgPosition /= NrOfNeighbors;
	
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	for (int i{}; i < NrOfNeighbors; ++i) {
		auto pos = neighbours.at(i)->ActorToWorld().GetLocation();
		avgVelocity += FVector2D(pos.X, pos.Z);
	}
	
	avgVelocity /= NrOfNeighbors;

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	pSeekBehavior->SetTarget(Target);
}

