#pragma once

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class AStar
	{
	public:
		AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	AStar<T_NodeType, T_ConnectionType>::AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> AStar<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		vector<T_NodeType*> path;
		vector<NodeRecord> openList;
		vector<NodeRecord> closedList;
		NodeRecord currentRecord;

		// Create NodeRecord to kickstart loop
		currentRecord.pNode = pStartNode;
		currentRecord.pConnection = nullptr;
		currentRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
		openList.push_back(currentRecord);
		
		// Loop over openlist
		while (!openList.empty())
		{
			// Get NodeRecord with lowest cost from openList
			currentRecord = openList[0];
			for (auto record : openList)
			{
				if (record < currentRecord)
					currentRecord = record;
			}
			if (currentRecord.pConnection != nullptr && currentRecord.pConnection->GetTo() == pGoalNode->GetIndex())
				break;

			for (auto connection: m_pGraph->GetNodeConnections(currentRecord.pNode))
			{
				auto GCost = currentRecord.costSoFar + connection->GetCost();
				int currentNode = connection->GetTo();
				auto LCost = GCost + GetHeuristicCost(m_pGraph->GetNode(currentNode), pGoalNode);
				auto lambda = [currentNode](const NodeRecord& n)-> bool {return currentNode == n.pNode->GetIndex(); };
				auto clFound = find_if(closedList.begin(), closedList.end(), lambda);
				auto olFound = find_if(openList.begin(), openList.end(), lambda);

				if (clFound != closedList.end())
				{
					if (GCost >= closedList[clFound - closedList.begin()].costSoFar)
						continue;
					else
						closedList.erase(clFound);
				}
				else if (olFound != openList.end())
				{
					if (GCost >= openList[olFound - openList.begin()].costSoFar)
						continue;
					else
						openList.erase(olFound);
				}
				NodeRecord temp;
				temp.pNode = m_pGraph->GetNode(connection->GetTo());
				temp.costSoFar = GCost;
				temp.pConnection = connection;
				temp.estimatedTotalCost = LCost;
				openList.push_back(temp);
			}
			auto olRecord = find(openList.begin(), openList.end(), currentRecord);
			openList.erase(olRecord);
			closedList.push_back(currentRecord);
		}

		auto idx = currentRecord.pConnection->GetFrom();
		path.push_back(pGoalNode);
		while (idx != pStartNode->GetIndex())
		{
			for (auto item : closedList)
			{
				if (item.pNode->GetIndex() == idx)
				{
					path.push_back(item.pNode);
					idx = item.pConnection->GetFrom();
					break;
				}
			}
		}
		path.push_back(pStartNode);
		std::reverse(path.begin(), path.end());

		return path;
	}

	template <class T_NodeType, class T_ConnectionType>
	float Elite::AStar<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}
}