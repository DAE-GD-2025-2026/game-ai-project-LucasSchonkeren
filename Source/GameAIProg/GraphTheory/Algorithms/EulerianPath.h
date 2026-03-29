#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline std::vector<int> FindOddDegreeNodes(Graph& graph) 
	{
		std::vector<Node*> Nodes = graph.GetActiveNodes();
		
		std::vector<int> oddIndices{};
		
		for (int i{}; i < Nodes.size(); ++i) {
			if (graph.FindConnectionsFrom(i).size() % 2 != 0)
				oddIndices.emplace_back(i);
		}
		
		return oddIndices;
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		if (not IsConnected())
			return Eulerianity::notEulerian;

		auto oddIndices {FindOddDegreeNodes(*m_pGraph)};
		
		switch (oddIndices.size())
		{
		case 0:
			return Eulerianity::eulerian;
		case 2:
			return Eulerianity::semiEulerian;
		default:
			return Eulerianity::notEulerian;
		}
	}


	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		if (Nodes.size() < 1)
			return Path;
		
		// Check if there can be an Euler path
		// If this graph is not eulerian, return the empty path
		
		switch (eulerianity) {
		case Eulerianity::notEulerian:
			return Path;
		case Eulerianity::semiEulerian:
			currentNodeId = FindOddDegreeNodes(graphCopy).at(0);
			break;
		case Eulerianity::eulerian:
			currentNodeId = Nodes.at(0)->GetId();
			break;
		}
		
		// Start algorithm loop
		std::stack<int> nodeStack{};
		std::vector<Connection*> connections {graphCopy.FindConnectionsFrom(currentNodeId)};
		
		while (not connections.empty())
		{
			nodeStack.emplace(currentNodeId);
			currentNodeId = connections.at(0)->GetToId();
			graphCopy.RemoveConnection(connections.at(0));
			connections = graphCopy.FindConnectionsFrom(currentNodeId);
		}
		
		nodeStack.emplace(currentNodeId);
		
		auto realNodes = m_pGraph->GetActiveNodes();
		
		while (not nodeStack.empty()) 
		{
			Path.emplace_back(realNodes.at(nodeStack.top()));
			nodeStack.pop();
		}

		std::ranges::reverse(Path);
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// Mark the visited node
		
		visited[startIndex] = true;

		// Ask the graph for the connections from that node
		
		std::vector<Connection*> connections {m_pGraph->FindConnectionsFrom(startIndex) };
		
		// recursively visit any valid connected nodes that were not visited before
		
		for (auto connectionPtr : connections) 
		{
			int index{connectionPtr->GetToId()};
			if (visited.at(index))
				continue;
			VisitAllNodesDFS(Nodes, visited, index);
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;

		// choose a starting node
		
		std::vector<Node*> nodesToSearch;
		nodesToSearch.emplace_back(Nodes[0]);
		
		// start a depth-first-search traversal from the node that has at least one connection
		
		std::vector<Node*> foundNodes;
		
		while (not nodesToSearch.empty())
		{
			Node* currentNodePtr {nodesToSearch.back()};
			nodesToSearch.pop_back();
			
			if (std::ranges::find(foundNodes, currentNodePtr) != foundNodes.end())
				continue;

			foundNodes.emplace_back(currentNodePtr);
			
			auto connections {m_pGraph->FindConnectionsFrom(currentNodePtr->GetId())};
			
			for (auto connection : connections)
			{
				nodesToSearch.emplace_back(Nodes.at(connection->GetToId()));
			}
		} 
		
		//  if a node was never visited, this graph is not connected
		return foundNodes.size() == Nodes.size();
	}
}