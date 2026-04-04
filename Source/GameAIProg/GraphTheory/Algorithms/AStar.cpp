#include "AStar.h"

#include <Queue>
#include <Set>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> nodes {pGraph->GetActiveNodes()};

	std::vector<Node*> path{};
	std::set<NodeRecord> openNodes{};
	std::set<NodeRecord> closedNodes{};
	
	NodeRecord startRecord{pStartNode, nullptr, 0, GetHeuristicCost(pStartNode, pGoalNode)};
	NodeRecord currentNodeRecord{};
	
	openNodes.emplace(startRecord);

	while (!openNodes.empty()) {
		currentNodeRecord = *openNodes.begin();
		
		if (currentNodeRecord.pNode == pGoalNode)
			break;
			
		for (auto connection : pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId())) {
			Node* pNextNode{nodes.at(connection->GetToId())};
			
			float gCostSoFar{currentNodeRecord.costSoFar + connection->GetWeight()};
			
			// Check closedlist
			if (auto existingRecord {std::ranges::find_if(closedNodes, [pNextNode](const NodeRecord& record){
				return record.pNode == pNextNode; 
			})};
			existingRecord != closedNodes.end()) {
				if (existingRecord->costSoFar <= gCostSoFar)
					continue;
				else
					closedNodes.erase(existingRecord);
			}
			
			// Check Openlist
			if (auto existingRecord {std::ranges::find_if(openNodes, [pNextNode](const NodeRecord& record){
				return record.pNode == pNextNode; 
			})};
			existingRecord != openNodes.end()) {
				if (existingRecord->costSoFar <= gCostSoFar)
					continue;
				else
					openNodes.erase(existingRecord);
			}
			openNodes.emplace(NodeRecord{pNextNode, connection, gCostSoFar, gCostSoFar + GetHeuristicCost(pNextNode, pGoalNode)});
		}
		
		openNodes.erase(currentNodeRecord);
		closedNodes.emplace(currentNodeRecord);
	}
	
	while (currentNodeRecord.pConnection != nullptr)
	{
		path.emplace(path.begin(), currentNodeRecord.pNode);
		currentNodeRecord = *std::ranges::find_if(closedNodes, [currentNodeRecord](const NodeRecord& record){
			return record.pNode->GetId() == currentNodeRecord.pConnection->GetFromId();
		});
	}
		
	path.emplace(path.begin(), startRecord.pNode);
	
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}