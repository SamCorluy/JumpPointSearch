#pragma once

template <class T_NodeType>
struct NodeRecord
{
	T_NodeType* pNode = nullptr;
	T_NodeType* pParent = nullptr;
	float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
	float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

	bool operator==(const NodeRecord& other) const
	{
		return pNode == other.pNode
			&& pParent == other.pParent
			&& costSoFar == other.costSoFar
			&& estimatedTotalCost == other.estimatedTotalCost;
	};

	bool operator<(const NodeRecord& other) const
	{
		return estimatedTotalCost < other.estimatedTotalCost;
	};
};

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class JPS
	{
	public:
		JPS(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
		void IdentifySuccessors(NodeRecord<T_NodeType> current, T_NodeType* start, T_NodeType* end, std::vector<NodeRecord<T_NodeType>>* openList, std::vector<NodeRecord<T_NodeType>>* closedList);
		std::vector<T_NodeType*> GetNodeNeighbors(T_NodeType* node);
		bool HasForcedNeighbor(T_NodeType* current, T_NodeType* nextPoint, int dirX, int dirY);
		NodeRecord<T_NodeType> Jump(NodeRecord<T_NodeType> current, int dirX, int dirY, T_NodeType* start, T_NodeType* end);
	};

	template <class T_NodeType, class T_ConnectionType>
	JPS<T_NodeType, T_ConnectionType>::JPS(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> JPS<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{

		vector<T_NodeType*> path;
		vector<NodeRecord<T_NodeType>> openList;
		vector<NodeRecord<T_NodeType>> closedList;
		NodeRecord<T_NodeType> currentRecord;

		// Create NodeRecord to kickstart loop
		currentRecord.pNode = pStartNode;
		currentRecord.costSoFar = 0;
		currentRecord.estimatedTotalCost = 0;
		openList.push_back(currentRecord);
		while (!openList.empty())
		{
			// Get NodeRecord with lowest cost from openList
			currentRecord = openList[0];
			for (auto record : openList)
			{
				if (record < currentRecord)
					currentRecord = record;
			}
			openList.pop_back();
			auto lambda = [currentRecord](const NodeRecord<T_NodeType> n)-> bool {return currentRecord.pNode == n.pNode; };
			auto olFound = find_if(openList.begin(), openList.end(), lambda);
			auto clFound = find_if(closedList.begin(), closedList.end(), lambda);
			if (olFound != openList.end())
				openList.erase(olFound);
			if (clFound == closedList.end())
			{
				closedList.push_back(currentRecord);
			}
			if (currentRecord.pNode == pGoalNode)
				break;

			IdentifySuccessors(currentRecord, pStartNode, pGoalNode, &openList, &closedList);
		}

		path.push_back(pStartNode);
		std::reverse(path.begin(), path.end());

		return path;
	}

	template <class T_NodeType, class T_ConnectionType>
	float Elite::JPS<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}

	/*template<class T_NodeType, class T_ConnectionType>
	inline std::vector<T_NodeType*> JPS<T_NodeType, T_ConnectionType>::IdentifySuccessors(T_NodeType* current, T_NodeType* start, T_NodeType* end)
	{
		std::vector<T_NodeType*> successors{};
		std::vector<T_NodeType*> neighbors = GetNodeNeighbors(current);

		for (auto neighbor : neighbors)
		{
			int dX = Clamp(m_pGraph->GetNodePos(neighbor->GetIndex()).x - m_pGraph->GetNodePos(current->GetIndex()).x, -1.f, 1.f);
			int dY = Clamp(m_pGraph->GetNodePos(neighbor->GetIndex()).y - m_pGraph->GetNodePos(current->GetIndex()).y, -1.f, 1.f);

			T_NodeType* jumpPoint = Jump(m_pGraph->GetNodePos(current->GetIndex()).x, m_pGraph->GetNodePos(current->GetIndex()).y, dX, dY, start, end);

			if (jumpPoint)
				successors.push_back(jumpPoint);
		}

		return successors;
	}*/

	template<class T_NodeType, class T_ConnectionType>
	inline void JPS<T_NodeType, T_ConnectionType>::IdentifySuccessors(NodeRecord<T_NodeType> current, T_NodeType* start, T_NodeType* end, std::vector<NodeRecord<T_NodeType>>* openList, std::vector<NodeRecord<T_NodeType>>* closedList)
	{
		NodeRecord<T_NodeType> jumpNode{};
		std::vector<T_NodeType*> neighbors = GetNodeNeighbors(current.pNode);

		float g = 0;
		float dist;

		for (auto neighbor : neighbors)
		{
			int dX = int(Clamp(m_pGraph->GetNodePos(neighbor->GetIndex()).x - m_pGraph->GetNodePos(current.pNode->GetIndex()).x, -1.f, 1.f));
			int dY = int(Clamp(m_pGraph->GetNodePos(neighbor->GetIndex()).y - m_pGraph->GetNodePos(current.pNode->GetIndex()).y, -1.f, 1.f));

			auto result = Jump(current, dX, dY, start, end);

			if (!result.pNode)
				continue;

			jumpNode = result;
			auto lambda = [jumpNode](const NodeRecord<T_NodeType> n)-> bool {return jumpNode.pNode == n.pNode; };
			auto clFound = find_if(closedList->begin(), closedList->end(), lambda);
			auto olFound = find_if(openList->begin(), openList->end(), lambda);
			if (clFound != closedList->end())
				continue;

			dist = Elite::Distance(m_pGraph->GetNodePos(current.pNode), m_pGraph->GetNodePos(jumpNode.pNode));
			g = current.costSoFar + dist;
			if (olFound == openList->end() || g < jumpNode.costSoFar)
			{
				jumpNode.costSoFar = g;
				if (jumpNode.estimatedTotalCost == 0)
					jumpNode.estimatedTotalCost = GetHeuristicCost(jumpNode.pNode, end);
				jumpNode.pParent = current.pNode;
				auto olFound = find_if(openList->begin(), openList->end(), lambda);
				if (olFound == openList->end())
				{
					openList->push_back(jumpNode);
				}
			}
		}
	}

	template<class T_NodeType, class T_ConnectionType>
	inline std::vector<T_NodeType*> JPS<T_NodeType, T_ConnectionType>::GetNodeNeighbors(T_NodeType* node)
	{
		auto list = m_pGraph->GetNodeConnections(node->GetIndex());
		std::vector<T_NodeType*> nodes{};
		for (auto item : list)
		{
			nodes.push_back(m_pGraph->GetNode(item->GetTo()));
		}

		return nodes;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline bool JPS<T_NodeType, T_ConnectionType>::HasForcedNeighbor(T_NodeType* current, T_NodeType* nextPoint, int dirX, int dirY)
	{
		auto x = m_pGraph->GetNodePos(current).x + dirY * 5.f;
		auto y = m_pGraph->GetNodePos(current).y + dirX * 5.f;
		auto neighbor1 = m_pGraph->GetNodeAtWorldPos(Elite::Vector2(x, y));

		x = m_pGraph->GetNodePos(current).x - dirY * 5.f;
		y = m_pGraph->GetNodePos(current).y - dirX * 5.f;
		auto neighbor2 = m_pGraph->GetNodeAtWorldPos(Elite::Vector2(x, y));

		x = m_pGraph->GetNodePos(nextPoint).x + dirY * 5.f;
		y = m_pGraph->GetNodePos(nextPoint).y + dirX * 5.f;
		auto neighbor3 = m_pGraph->GetNodeAtWorldPos(Elite::Vector2(x, y));

		x = m_pGraph->GetNodePos(nextPoint).x - dirY * 5.f;
		y = m_pGraph->GetNodePos(nextPoint).y - dirX * 5.f;
		auto neighbor4 = m_pGraph->GetNodeAtWorldPos(Elite::Vector2(x, y));

		bool a = neighbor1 && neighbor1->GetTerrainType() == TerrainType::Water;
		bool b = neighbor2 && neighbor2->GetTerrainType() == TerrainType::Water;

		if (a != b)
			return true;

		a = neighbor3 && neighbor3->GetTerrainType() == TerrainType::Water;
		b = neighbor4 && neighbor4->GetTerrainType() == TerrainType::Water;
		return a != b;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline NodeRecord<T_NodeType> JPS<T_NodeType, T_ConnectionType>::Jump(NodeRecord<T_NodeType> current, int dirX, int dirY, T_NodeType* start, T_NodeType* end)
	{
		float cellSize = 15.f;
		float nextX = m_pGraph->GetNodeWorldPos(current.pNode).x + dirX * cellSize;
		float nextY = m_pGraph->GetNodeWorldPos(current.pNode).y + dirY * cellSize;

		auto node = m_pGraph->GetNodeAtWorldPos(Elite::Vector2(nextX, nextY));
		NodeRecord<T_NodeType> nodeRecord;
		nodeRecord.pNode = node;

		if (!node || node->GetTerrainType() == TerrainType::Water)
		{
			nodeRecord.pNode = nullptr;
			return nodeRecord;
		}
		if (node == end)
		{
			return nodeRecord;
		}

		if (dirX != 0 && dirY != 0) {
			if (HasForcedNeighbor(current.pNode, node, dirX, dirY)) {
				return nodeRecord;
			}

			// Check in horizontal and vertical directions for forced neighbors
			// This is a special case for diagonal direction
			if (Jump(nodeRecord, dirX, 0, start, end).pNode != nullptr ||
				Jump(nodeRecord, 0, dirY, start, end).pNode != nullptr)
			{
				return nodeRecord;
			}
		}
		else {
			// Horizontal case
			if (dirX != 0) {
				if (HasForcedNeighbor(current.pNode, node, dirX, dirY)) {
					return nodeRecord;
				}
				/// Vertical case
			}
			else {
				if (HasForcedNeighbor(current.pNode, node, dirX, dirY)) {
					return nodeRecord;
				}
			}
		}
		return Jump(nodeRecord, dirX, dirY, start, end);
	}
}
