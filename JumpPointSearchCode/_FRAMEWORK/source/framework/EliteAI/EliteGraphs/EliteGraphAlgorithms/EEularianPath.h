#pragma once
#include <stack>

namespace Elite
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	template <class T_NodeType, class T_ConnectionType>
	class EulerianPath
	{
	public:

		EulerianPath(IGraph<T_NodeType, T_ConnectionType>* pGraph);

		Eulerianity IsEulerian() const;
		vector<T_NodeType*> FindPath(Eulerianity& eulerianity) const;
		void GraphColouring();
		vector<T_ConnectionType*> FindMST() const;

	private:
		void VisitAllNodesDFS(int startIdx, vector<bool>& visited) const;
		bool IsConnected() const;

		Color translateNumberToColor(int amountOfNodes, int color) const;
		bool FormsCycle(int currentNodeIdx, vector<int>& visitedIdx, int indexToReach, IGraph<T_NodeType, T_ConnectionType>* pGraph) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
	};

	template<class T_NodeType, class T_ConnectionType>
	inline EulerianPath<T_NodeType, T_ConnectionType>::EulerianPath(IGraph<T_NodeType, T_ConnectionType>* pGraph)
		: m_pGraph(pGraph)
	{
	}

	template<class T_NodeType, class T_ConnectionType>
	inline Eulerianity EulerianPath<T_NodeType, T_ConnectionType>::IsEulerian() const
	{
		// If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
			return Eulerianity::notEulerian;

		// Count nodes with odd degree 
		auto activeNodes = m_pGraph->GetAllActiveNodes();
		int oddCount = 0;
		for (auto node : activeNodes)
		{
			auto connections = m_pGraph->GetNodeConnections(node);
			if (connections.size() & 1)
				++oddCount;
		}

		// A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (oddCount > 2)
			return Eulerianity::notEulerian;

		// A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// An Euler trail can be made, but only starting and ending in these 2 nodes
		else if (oddCount == 2)
			return Eulerianity::semiEulerian;

		// A connected graph with no odd nodes is Eulerian
		else
			return Eulerianity::eulerian;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline vector<T_NodeType*> EulerianPath<T_NodeType, T_ConnectionType>::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		auto graphCopy = m_pGraph->Clone();
		vector<T_NodeType*> path{};
		//int nrOfNodes = graphCopy->GetNrOfNodes();

		// Check if there can be an Euler path
		// If this graph is not eulerian, return the empty path
		// Else we need to find a valid starting index for the algorithm
		T_NodeType* currentNode{nullptr};
		auto activeNodes = m_pGraph->GetAllActiveNodes();

		if (eulerianity == Eulerianity::notEulerian)
			return path;
		else if (eulerianity == Eulerianity::eulerian)
		{
			for (auto node : activeNodes)
			{
				currentNode = node;
				break;
			}
			
		}
		else if(eulerianity == Eulerianity::semiEulerian)
		{
			for (auto node : activeNodes)
			{
				if (m_pGraph->IsNodeValid(node->GetIndex()) && m_pGraph->GetNodeConnections(node->GetIndex()).size() & 1)
				{
					currentNode = node;
					break;
				}
			}
		}
		

		// Start algorithm loop
		stack<T_NodeType*> nodeStack;
		while (nodeStack.size() > 0 || graphCopy->GetNodeConnections(currentNode).size() > 0)
		{
			if (graphCopy->GetNodeConnections(currentNode).size() == 0)
			{
				path.push_back(currentNode);
				currentNode = nodeStack.top();
				nodeStack.pop();
			}
			else
			{
				nodeStack.push(currentNode);
				currentNode = m_pGraph->GetNode(graphCopy->GetNodeConnections(currentNode).front()->GetTo());
				graphCopy->RemoveConnection(nodeStack.top()->GetIndex(), currentNode->GetIndex());
			}
		}
		path.push_back(currentNode);
		auto tempPath = path;

		for (unsigned int i{ 0 }; i < tempPath.size(); ++i)
		{
			path[i] = tempPath[tempPath.size() - 1 - i];
		}

		return path;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline void EulerianPath<T_NodeType, T_ConnectionType>::GraphColouring()
	{
		unsigned int mostColors{ 2 };
		unsigned int amountOfNodes{ m_pGraph->GetAllActiveNodes().size() };
		vector<bool> adjacencyMatrix;
		adjacencyMatrix.reserve(amountOfNodes * amountOfNodes);
		for (unsigned int i{ 0 }; i < amountOfNodes; ++i)
		{
			for (unsigned int j{ 0 }; j < amountOfNodes; ++j)
			{
				auto node = m_pGraph->GetAllActiveNodes()[i];
				auto compareNode = m_pGraph->GetAllActiveNodes()[j];
				bool isNeighbour{ node == compareNode || m_pGraph->GetConnection(node->GetIndex(), compareNode->GetIndex()) != nullptr };
				adjacencyMatrix.push_back(isNeighbour);
			}
		}

		bool success{ false };
		vector<int> nodeColors;
		nodeColors.reserve(amountOfNodes);

		nodeColors.push_back(1);

		while (!success)
		{
			for (unsigned int i{ 1 }; i < amountOfNodes; ++i)
			{
				for (unsigned int j{ 1 }; j <= mostColors; ++j)
				{
					bool safeToColor{ false };
					for (unsigned int k{ 0 }; k < i; ++k)
					{
						//std::cout << k << " " << nodeColors.size() << std::endl;
						safeToColor = (adjacencyMatrix[i + (k * amountOfNodes)] == 0 || nodeColors[k] != j);
						if (!safeToColor)
							break;
					}
					success = safeToColor;
					if (safeToColor)
					{
						nodeColors.push_back(j);
						break;
					}
				}
				if (!success)
				{
					nodeColors.clear();
					nodeColors.push_back(1);
					++mostColors;
					break;
				}
			}
		}

		for (unsigned int i{ 0 }; i < m_pGraph->GetAllActiveNodes().size(); ++i)
		{
			m_pGraph->GetAllActiveNodes()[i]->SetColor(translateNumberToColor(mostColors, nodeColors[i]));
		}
	}

	template<class T_NodeType, class T_ConnectionType>
	inline vector<T_ConnectionType*> EulerianPath<T_NodeType, T_ConnectionType>::FindMST() const
	{
		auto graphCopy = m_pGraph->Clone();
		vector<T_ConnectionType*> mstPath{};
		//int nrOfNodes = graphCopy->GetNrOfNodes();

		// Check if there can be an Euler path
		// If this graph is not eulerian, return the empty path
		// Else we need to find a valid starting index for the algorithm
		T_NodeType* currentNode{ nullptr };
		auto activeNodes = m_pGraph->GetAllActiveNodes();

		struct link {
			int idxFrom;
			int idxTo;
			float weight;
		};

		vector<link> links{};

		for (auto node : activeNodes)
		{
			for (auto connection : m_pGraph->GetNodeConnections(node->GetIndex()))
			{
				if (graphCopy->GetConnection(connection->GetFrom(), connection->GetTo()))
				{
					connection->GetFrom();
					links.push_back(link{ connection->GetFrom(), connection->GetTo(), connection->GetCost() });
					for (int i{ 0 }; i < int(links.size()) - 1; ++i)
					{
						if (links[i].weight > links[links.size() - 1].weight)
						{
							for (int j{ int(links.size()) - 2 }; j >= i; --j)
							{
								links[j + 1] = links[j];
							}
							links[i] = link{ connection->GetFrom(), connection->GetTo(), connection->GetCost() };
						}
					}
					graphCopy->RemoveConnection(connection->GetFrom(), connection->GetTo());
				}
			}
		}

		for (unsigned int i{0}; i < links.size(); ++i)
		{
			vector<int> pathsIdx{};
			pathsIdx.push_back(links[0].idxFrom);
			pathsIdx.push_back(links[0].idxTo);
			vector<int> idxChecked{};


			int prevSize{};
			int nodeIndexToCheck{ links[i].idxFrom };

			bool allChecked{false};

			vector<int> visited;
			if (!FormsCycle(links[i].idxFrom, visited, links[i].idxTo, graphCopy.get()))
			{
				mstPath.push_back(m_pGraph->GetConnection(links[i].idxFrom, links[i].idxTo));
				mstPath.push_back(m_pGraph->GetConnection(links[i].idxTo, links[i].idxFrom));
				graphCopy->AddConnection(new GraphConnection2D{ links[i].idxFrom, links[i].idxTo, links[i].weight });
			}
		}
		return mstPath;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline void EulerianPath<T_NodeType, T_ConnectionType>::VisitAllNodesDFS(int startIdx, vector<bool>& visited) const
	{
		// mark the visited node
		visited[startIdx] = true;

		// recursively visit any valid connected nodes that were not visited before
		for (T_ConnectionType* pConnection : m_pGraph->GetNodeConnections(startIdx))
			if (!visited[pConnection->GetTo()])
				VisitAllNodesDFS(pConnection->GetTo(), visited);

	}

	template<class T_NodeType, class T_ConnectionType>
	inline bool EulerianPath<T_NodeType, T_ConnectionType>::IsConnected() const
	{
		auto activeNodes = m_pGraph->GetAllActiveNodes();
		vector<bool> visited(m_pGraph->GetNrOfNodes(), false);

		// find a valid starting node that has connections
		int connectedIdx = invalid_node_index;
		for (auto node : activeNodes)
		{
			auto connections = m_pGraph->GetNodeConnections(node);
			if (connections.size() > 0)
			{
				connectedIdx = node->GetIndex();
				break;
			}
		}
		
		// if no valid node could be found, return false
		if (connectedIdx == invalid_node_index)
			return false;

		// start a depth-first-search traversal from the node that has at least one connection
		VisitAllNodesDFS(connectedIdx, visited);

		// if a node was never visited, this graph is not connected
		for (auto node : activeNodes)
			if (!visited[node->GetIndex()])
				return false;

		return true;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline Color EulerianPath<T_NodeType, T_ConnectionType>::translateNumberToColor(int amountOfColors, int color) const
	{
		Color translatedColor{};
		float tint{ (ceil(float(amountOfColors) / 6.f) - ceil(float(color) / 6.f) + 1.f) / ceil(float(amountOfColors) / 6.f)};
		//std::cout << tint << std::endl;
		switch (color % 6)
		{
		case 1:
			translatedColor.r = tint;
			break;
		case 2:
			translatedColor.g = tint;
			break;
		case 3:
			translatedColor.b = tint;
			break;
		case 4:
			translatedColor.r = tint;
			translatedColor.g = tint;
			break;
		case 5:
			translatedColor.r = tint;
			translatedColor.b = tint;
			break;
		case 0:
			translatedColor.g = tint;
			translatedColor.b = tint;
			break;
		}
		return translatedColor;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline bool EulerianPath<T_NodeType, T_ConnectionType>::FormsCycle(int currentNodeIdx, vector<int>& visitedIdx, int indexToReach, IGraph<T_NodeType, T_ConnectionType>* pGraph) const
	{
		visitedIdx.push_back(currentNodeIdx);
		for (auto neighbor : pGraph->GetNodeConnections(currentNodeIdx))
		{
			bool isVisited{ false };
			for (auto visited : visitedIdx)
			{
				isVisited = (visited == neighbor->GetTo());
				if (isVisited)
					break;
			}
			if (!isVisited)
			{
				if (neighbor->GetTo() == indexToReach || FormsCycle(neighbor->GetTo(), visitedIdx, indexToReach, pGraph))
					return true;
			}
			else if (neighbor->GetTo() == indexToReach)
				return true;
		}
		return false;
	}

}