#pragma once

namespace Elite 
{
	template <class T_NodeType, class T_ConnectionType>
	class BFS
	{
	public:
		BFS(IGraph<T_NodeType, T_ConnectionType>* pGraph);

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);
	private:
		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
	};

	template <class T_NodeType, class T_ConnectionType>
	BFS<T_NodeType, T_ConnectionType>::BFS(IGraph<T_NodeType, T_ConnectionType>* pGraph)
		: m_pGraph(pGraph)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> BFS<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode)
	{
		std::queue<T_NodeType*> openList; // Frontier - Expanding edge
		std::map<T_NodeType*, T_NodeType*> closedList; // Already checked nodes

		openList.push(pStartNode); // Kickstarting the loop

		while (!openList.empty())
		{
			T_NodeType* pCurrentNode = openList.front();
			openList.pop();

			if (pCurrentNode == pDestinationNode)
				break; // EXIT WHILE LOOP

			// Looping over all connections from current node
			for (auto connections : m_pGraph->GetNodeConnections(pCurrentNode))
			{
				T_NodeType* nextNode = m_pGraph->GetNode(connections->GetTo());
				if (closedList.find(nextNode) == closedList.end())
				{
					openList.push(nextNode);
					closedList[nextNode] = pCurrentNode;
				}
			}
		}

		// Goal node found
		// Track back from goal node to start node to create the path
		vector<T_NodeType*> path;
		T_NodeType* pCurrentNode = pDestinationNode; // Start tracking back from end node
		while (pCurrentNode != pStartNode)
		{
			path.push_back(pCurrentNode);
			pCurrentNode = closedList[pCurrentNode];
		}
		path.push_back(pCurrentNode);

		std::reverse(path.begin(), path.end());

		return path;
	}
}

