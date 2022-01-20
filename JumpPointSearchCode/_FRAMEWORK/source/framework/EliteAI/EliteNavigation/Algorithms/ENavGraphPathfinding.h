#pragma once
#include <vector>
#include <iostream>
#include "framework/EliteMath/EMath.h"
#include "framework\EliteAI\EliteGraphs\ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EBFS.h"

namespace Elite
{
	class NavMeshPathfinding
	{
	public:
		static std::vector<Elite::Vector2> FindPath(Elite::Vector2 startPos, Elite::Vector2 endPos, Elite::NavGraph* pNavGraph, std::vector<Elite::Vector2>& debugNodePositions, std::vector<Elite::Portal>& debugPortals)
		{
			//Create the path to return
			std::vector<Elite::Vector2> finalPath{};

			//Get the start and endTriangle
			auto startTriangle = pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(startPos);
			auto endTriangle = pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(endPos);
		
			//We have valid start/end triangles and they are not the same
			if (!(startTriangle && endTriangle))
				return finalPath;
			if (startTriangle == endTriangle)
			{
				finalPath.push_back(endPos);
				return finalPath;
			}
			//=> Start looking for a path
			//Copy the graph
			auto graphCopy = pNavGraph->Clone();
			
			//Create extra node for the Start Node (Agent's position
			NavGraphNode* startNode{ new NavGraphNode(graphCopy->GetNextFreeNodeIndex(), -1, startPos) };
			graphCopy->AddNode(startNode);

			//Create extra node for the endNode
			NavGraphNode* endNode{ new NavGraphNode(graphCopy->GetNextFreeNodeIndex(), -1, endPos) };
			graphCopy->AddNode(endNode);

			for (auto line : startTriangle->metaData.IndexLines)
			{
				auto idx = pNavGraph->GetNodeIdxFromLineIdx(line);
				if (idx != -1 && line != -1)
				{
					GraphConnection2D* connection = new GraphConnection2D{ startNode->GetIndex(), idx };
					connection->SetCost(Distance(startNode->GetPosition(), pNavGraph->GetNode(idx)->GetPosition()));
					graphCopy->AddConnection(connection);
				}
			}
			for (auto line : endTriangle->metaData.IndexLines)
			{
				auto idx = pNavGraph->GetNodeIdxFromLineIdx(line);
				if (idx != -1 && line != -1)
				{
					GraphConnection2D* connection = new GraphConnection2D{ endNode->GetIndex(), idx };
					connection->SetCost(Distance(endNode->GetPosition(), pNavGraph->GetNode(idx)->GetPosition()));
					graphCopy->AddConnection(connection);
				}
			}
			
			//Run A star on new graph
			//auto pathfinder = BFS<NavGraphNode, GraphConnection2D>(graphCopy.get());
			auto pathfinder = AStar<NavGraphNode, GraphConnection2D>(graphCopy.get(), HeuristicFunctions::Chebyshev);
			auto path = pathfinder.FindPath(startNode, endNode);

			for (auto coord : path)
			{
				finalPath.push_back(coord->GetPosition());
			}
			debugNodePositions = finalPath;
			//OPTIONAL BUT ADVICED: Debug Visualisation

			//Run optimiser on new graph, MAKE SURE the A star path is working properly before starting this section and uncommenting this!!!
			auto portals = SSFA::FindPortals(path, pNavGraph->GetNavMeshPolygon());
			debugPortals = portals;
			finalPath = SSFA::OptimizePortals(portals);
			//finalPath.push_back(endNode->GetPosition());

			return finalPath;
		}
	};
}
