#include "stdafx.h"
#include "ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

using namespace Elite;

Elite::NavGraph::NavGraph(const Polygon& contourMesh, float playerRadius = 1.0f) :
	Graph2D(false),
	m_pNavMeshPolygon(nullptr)
{
	//Create the navigation mesh (polygon of navigatable area= Contour - Static Shapes)
	m_pNavMeshPolygon = new Polygon(contourMesh); // Create copy on heap

	//Get all shapes from all static rigidbodies with NavigationCollider flag
	auto vShapes = PHYSICSWORLD->GetAllStaticShapesInWorld(PhysicsFlags::NavigationCollider);

	//Store all children
	for (auto shape : vShapes)
	{
		shape.ExpandShape(playerRadius);
		m_pNavMeshPolygon->AddChild(shape);
	}

	//Triangulate
	m_pNavMeshPolygon->Triangulate();

	//Create the actual graph (nodes & connections) from the navigation mesh
	CreateNavigationGraph();
}

Elite::NavGraph::~NavGraph()
{
	delete m_pNavMeshPolygon; 
	m_pNavMeshPolygon = nullptr;
}

int Elite::NavGraph::GetNodeIdxFromLineIdx(int lineIdx) const
{
	auto nodeIt = std::find_if(m_Nodes.begin(), m_Nodes.end(), [lineIdx](const NavGraphNode* n) { return n->GetLineIndex() == lineIdx; });
	if (nodeIt != m_Nodes.end())
	{
		return (*nodeIt)->GetIndex();
	}

	return invalid_node_index;
}

Elite::Polygon* Elite::NavGraph::GetNavMeshPolygon() const
{
	return m_pNavMeshPolygon;
}

void Elite::NavGraph::CreateNavigationGraph()
{
	for (auto line : m_pNavMeshPolygon->GetLines())
	{
		if (m_pNavMeshPolygon->GetTrianglesFromLineIndex(line->index).size() > 1)
		{
			auto center = line->p1 + (line->p2 - line->p1)/2.f;
			NavGraphNode* node = new NavGraphNode(m_Nodes.size(), line->index, center);
			AddNode(node);
		}
	}
	for (auto triangle : m_pNavMeshPolygon->GetTriangles())
	{
		std::vector<NavGraphNode*> tempNodes;
		for (auto lineIdx : triangle->metaData.IndexLines)
		{
			auto nodeIdx = GetNodeIdxFromLineIdx(lineIdx);
			if (nodeIdx != invalid_node_index)
			{
				tempNodes.push_back(GetNode(nodeIdx));
			}
		}
		if (tempNodes.size() == 2)
		{
			GraphConnection2D* connection = new GraphConnection2D{ tempNodes[0]->GetIndex(), tempNodes[1]->GetIndex() };
			AddConnection(connection);
		}
		else if (tempNodes.size() == 3)
		{
			GraphConnection2D* connection = new GraphConnection2D{ tempNodes[0]->GetIndex(), tempNodes[1]->GetIndex() };
			GraphConnection2D* connection2 = new GraphConnection2D{ tempNodes[1]->GetIndex(), tempNodes[2]->GetIndex() };
			GraphConnection2D* connection3 = new GraphConnection2D{ tempNodes[2]->GetIndex(), tempNodes[0]->GetIndex() };
			AddConnection(connection);
			AddConnection(connection2);
			AddConnection(connection3);
		}
	}
	SetConnectionCostsToDistance();
	//1. Go over all the edges of the navigationmesh and create nodes
	
	//2. Create connections now that every node is created
	
	//3. Set the connections cost to the actual distance
}

