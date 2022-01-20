//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "App_GraphTheory.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EEularianPath.h"

using namespace Elite;

//Destructor
App_GraphTheory::~App_GraphTheory()
{
	SAFE_DELETE(m_pGraph2D);
}

//Functions
void App_GraphTheory::Start()
{
	//Initialization of your application. If you want access to the physics world you will need to store it yourself.
	//----------- CAMERA ------------
	DEBUGRENDERER2D->GetActiveCamera()->SetZoom(80.f);
	DEBUGRENDERER2D->GetActiveCamera()->SetCenter(Vector2(0, 0));
	DEBUGRENDERER2D->GetActiveCamera()->SetMoveLocked(false);
	DEBUGRENDERER2D->GetActiveCamera()->SetZoomLocked(false);

	m_pGraph2D = new Graph2D<GraphNode2D, GraphConnection2D>(false);
	m_pGraph2D->AddNode(new GraphNode2D(0, Vector2{ 20.f, 30.f }));
	m_pGraph2D->AddNode(new GraphNode2D(1, Vector2{ -10.f, -10.f }));
	m_pGraph2D->AddConnection(new GraphConnection2D(0, 1));
	m_CalculateColors = false;
	m_CalculateSpanTree = false;
}

void App_GraphTheory::Update(float deltaTime)
{
	m_GraphEditor.UpdateGraph(m_pGraph2D);
	m_pGraph2D->SetConnectionCostsToDistance();

	auto eulerFinder = EulerianPath<GraphNode2D, GraphConnection2D>(m_pGraph2D);
	auto isEuler = eulerFinder.IsEulerian();

	switch (isEuler)
	{
	case Eulerianity::eulerian:
		cout << "eulerian\n";
		break;
	case Eulerianity::semiEulerian:
		cout << "semi eulerian\n";
		break;
	case Eulerianity::notEulerian:
		cout << "not eulerian\n";
		break;
	default:
		break;
	}

	

	if (m_CalculateColors)
		eulerFinder.GraphColouring();
	else
	{
		for (auto node : m_pGraph2D->GetAllActiveNodes())
		{
			node->SetColor(Color(1, 1, 1));
		}
	}
	if(m_CalculateSpanTree)
	{
		auto mstPath = eulerFinder.FindMST();
		for (auto node : m_pGraph2D->GetAllActiveNodes())
		{
			//node->SetColor(Color(1, 1, 1));
			for (auto link : m_pGraph2D->GetNodeConnections(node->GetIndex()))
			{
				link->SetColor(Color(0, 0, 0));
			}
		}
		for (auto link : mstPath)
		{
			//node->SetColor(Color(1, 1, 1));
			link->SetColor(Color(1, 1, 1));
		}
	}
	else
	{
		vector<GraphNode2D*> path{ eulerFinder.FindPath(isEuler) };

		float divider{ float(path.size() - 1) };

		if (isEuler == Eulerianity::eulerian || isEuler == Eulerianity::semiEulerian)
		{
			for (int i{ 0 }; i < int(path.size() - 1); ++i)
			{
				float scalar{ float(i) / divider };
				float greyValue{ scalar * 1 };

				auto connection = m_pGraph2D->GetConnection(path[i + 1]->GetIndex(), path[i]->GetIndex());
				if (connection != nullptr)
				{
					connection->SetColor(Color(greyValue, 0, 0));
					connection->SetCost(float(i));
					connection = m_pGraph2D->GetConnection(path[i]->GetIndex(), path[i + 1]->GetIndex());
					connection->SetColor(Color(greyValue, 0, 0));
					connection->SetCost(float(i));
				}
			}
		}
		else
		{
			for (auto node : m_pGraph2D->GetAllActiveNodes())
			{
				//node->SetColor(Color(1, 1, 1));
				for (auto link : m_pGraph2D->GetNodeConnections(node->GetIndex()))
				{
					link->SetColor(Color(0, 0, 0));
				}
			}
		}
	}
	

	//------- UI --------
#ifdef PLATFORM_WINDOWS
#pragma region UI
	{
		//Setup
		int menuWidth = 150;
		int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
		int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
		bool windowActive = true;
		ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
		ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 90));
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::PushAllowKeyboardFocus(false);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);
		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
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
		ImGui::Spacing();

		ImGui::Text("Graph Theory");
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Checkbox("Find MST: ", &m_CalculateSpanTree);
		ImGui::Checkbox("Color nodes: ", &m_CalculateColors);

		//End
		ImGui::PopAllowKeyboardFocus();
		ImGui::End();
	}
#pragma endregion
#endif
	

}

void App_GraphTheory::Render(float deltaTime) const
{
	m_GraphRenderer.RenderGraph(m_pGraph2D, true, true, true, true);
}
