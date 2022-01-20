#include "stdafx.h"
#include "TheFlock.h"

#include "../SteeringAgent.h"
#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"

//Constructor & Destructor
Flock::Flock(
	int flockSize /*= 50*/, 
	float worldSize /*= 100.f*/, 
	SteeringAgent* pAgentToEvade /*= nullptr*/, 
	bool trimWorld /*= false*/)

	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_TrimWorld { trimWorld }
	, m_pAgentToEvade{pAgentToEvade}
	, m_NeighborhoodRadius{ 5 }
	, m_NrOfNeighbors{0}
{
	m_Agents.resize(m_FlockSize);

	// TODO: initialize the flock and the memory pool
	m_pSeparationBehavior = new Separation(this);
	m_pCohesionBehavior = new Cohesion(this);
	m_pVelMatchBehavior = new VelocityMatch(this);
	m_pSeekBehavior = new Seek();
	m_pWanderBehavior = new Wander();
	m_pWanderBehavior->SetWanderOffset(20);
	m_pBlendedSteering = new BlendedSteering({ {m_pSeparationBehavior, 0.7f}, {m_pCohesionBehavior, 0.5f}, {m_pVelMatchBehavior, 0.5f}, {m_pSeekBehavior, 0.f}, {m_pWanderBehavior, 0.5f} });
	m_pEvadeBehavior = new Evade();
	m_pEvadeBehavior->SetEvadeRadius(15.f);
	m_pPrioritySteering = new PrioritySteering({ m_pEvadeBehavior, m_pBlendedSteering });

	m_pAgentToEvade = new SteeringAgent();
	m_pAgentToEvade->SetMaxLinearSpeed(50.f);
	m_pAgentToEvade->SetMass(0.1f);
	m_pAgentToEvade->SetSteeringBehavior(m_pWanderBehavior);
	m_pAgentToEvade->SetBodyColor(Elite::Color{ 1, 0, 0 });
	m_pAgentToEvade->SetAutoOrient(true);

	m_pCellSpace = new CellSpace(m_WorldSize * 2, m_WorldSize * 2, 25, 25, m_FlockSize);

	for (size_t i{ 0 }; i < (size_t)m_FlockSize; ++i)
	{
		m_Agents[i] = new SteeringAgent();

		//m_Agents[i]->SetSteeringBehavior(m_pBlendedSteering);
		m_Agents[i]->SetSteeringBehavior(m_pPrioritySteering);
		m_Agents[i]->SetMaxLinearSpeed(50.f);
		m_Agents[i]->SetMaxAngularSpeed(25.f);
		m_Agents[i]->SetMass(0.1f);
		//m_Agents[i]->SetMaxLinearSpeed(15.f);
		//m_Agents[i]->SetMass(1.f);
		m_Agents[i]->SetAutoOrient(true);
		m_Agents[i]->SetPosition({ float(rand() % int(m_WorldSize*2.f)) - m_WorldSize, float(rand() % int(m_WorldSize * 2.f)) - m_WorldSize });

		m_pCellSpace->AddAgent(m_Agents[i]);
		m_OldPositions.push_back(m_Agents[i]->GetPosition());
	}

	m_Neighbors.resize(m_FlockSize);
}

Flock::~Flock()
{
	// TODO: clean up any additional data


	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pVelMatchBehavior);
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pWanderBehavior);
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pAgentToEvade);
	SAFE_DELETE(m_pCellSpace);

	for(auto pAgent: m_Agents)
	{
		SAFE_DELETE(pAgent);
	}
	m_Agents.clear();
}

void Flock::Update(float deltaT)
{
	// TODO: update the flock
	// loop over all the agents
		// register its neighbors	(-> memory pool is filled with neighbors of the currently evaluated agent)
		// update it				(-> the behaviors can use the neighbors stored in the pool, next iteration they will be the next agent's neighbors)
		// trim it to the world
	m_pAgentToEvade->Update(deltaT);

	TargetData evadeTarget;
	evadeTarget.LinearVelocity = m_pAgentToEvade->GetLinearVelocity();
	evadeTarget.Position = m_pAgentToEvade->GetPosition();

	m_pEvadeBehavior->SetTarget(evadeTarget);

	for (size_t i{ 0 }; i<m_Agents.size(); ++i)
	{
		if (!m_UseSpacePar)
		{
			RegisterNeighbors(m_Agents[i]);
		}
		else
		{
			m_pCellSpace->RegisterNeighbors(m_Agents[i], m_NeighborhoodRadius);
			m_Neighbors = m_pCellSpace->GetNeighbors();
			m_NrOfNeighbors = m_pCellSpace->GetNrOfNeighbors();
		}
		m_Agents[i]->Update(deltaT);

		if (m_TrimWorld)
			m_Agents[i]->TrimToWorld(m_WorldSize);

		if (m_UseSpacePar)
		{
			m_pCellSpace->UpdateAgentCell(m_Agents[i], m_OldPositions[i]);
			m_OldPositions[i] = m_Agents[i]->GetPosition();
		}

		// Debug Update
		if (m_CanDebugRender)
		{
			if (i != 0)
			{
				m_Agents[i]->SetBodyColor(Elite::Color{ 1,1,0 });
				for (size_t j{ 0 }; j < (size_t)m_NrOfNeighbors; ++j)
				{
					if (m_Neighbors[j] == m_Agents[0])
					{
						m_Agents[i]->SetBodyColor(Elite::Color{ 0,1,0 });
						break;
					}
				}
			}
			else
			{
				AverageNeighborPosDebug = GetAverageNeighborPos();
				AverageNeighborVelDebug = GetAverageNeighborVelocity();
				m_pCellSpace->SetDebugValues(m_Agents[i], m_NeighborhoodRadius);
			}
		}
		else
			m_Agents[i]->SetBodyColor(Elite::Color{ 1,1,0 });
	}

	if (m_TrimWorld)
		m_pAgentToEvade->TrimToWorld(m_WorldSize);
}

void Flock::Render(float deltaT)
{
	if (m_TrimWorld)
	{
		std::vector<Elite::Vector2> points =
		{
			{ -m_WorldSize, m_WorldSize },
			{ m_WorldSize, m_WorldSize },
			{ m_WorldSize, -m_WorldSize },
			{ -m_WorldSize, -m_WorldSize }
		};
		DEBUGRENDERER2D->DrawPolygon(&points[0], 4, { 1,0,0,1 }, 0.4f);
	}
	// TODO: render the flock
	//m_Agents[0]->SetRenderBehavior(m_CanDebugRender);

	if (m_CanDebugRender)
	{
		DEBUGRENDERER2D->DrawCircle(m_Agents[0]->GetPosition(), m_NeighborhoodRadius, Elite::Color{ 1,0,0,1 }, 0.4f);
		DEBUGRENDERER2D->DrawDirection(m_Agents[0]->GetPosition(), m_Agents[0]->GetLinearVelocity(), 4.f, Elite::Color{ 0,1,0,1 }, 0.4f);
		m_pCellSpace->RenderCells();
	}

	/*for (auto pAgent : m_Agents)
	{
		pAgent->Render(deltaT);
	}*/
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
	m_NrOfNeighbors = 0;
	for (auto neighbor : m_Agents)
	{
		if (neighbor != pAgent && (pAgent->GetPosition() - neighbor->GetPosition()).Magnitude() <= m_NeighborhoodRadius)
		{
			m_Neighbors[m_NrOfNeighbors] = neighbor;
			++m_NrOfNeighbors;
		}
	}
}

void Flock::UpdateAndRenderUI()
{
	//Setup
	int menuWidth = 235;
	int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
	int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
	bool windowActive = true;
	ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
	ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
	ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	ImGui::PushAllowKeyboardFocus(false);

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

	// TODO: Implement checkboxes for debug rendering and weight sliders here
	ImGui::Checkbox("Debug Rendering", &m_CanDebugRender);
	//ImGui::Checkbox("Trim World", &m_TrimWorld);
	ImGui::Checkbox("Spacial partitioning", &m_UseSpacePar);
	//if (m_TrimWorld)
	//{
	//	ImGui::SliderFloat("Trim Size", &m_WorldSize, 0.f, 500.f, "%1.");
	//}

	ImGui::Spacing();
	ImGui::Text("Parameters");
	ImGui::SliderFloat("NeighborRadius", &m_NeighborhoodRadius, 0.f, 50.f, ".%2");

	ImGui::Spacing();
	ImGui::Text("Behaviors");
	ImGui::SliderFloat("Separation", &m_pBlendedSteering->GetWeightedBehaviorsRef()[0].weight, 0.f, 1.f, ".%2");
	ImGui::SliderFloat("Cohesion", &m_pBlendedSteering->GetWeightedBehaviorsRef()[1].weight, 0.f, 1.f, ".%2");
	ImGui::SliderFloat("VelocityMatch", &m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight, 0.f, 1.f, ".%2");
	ImGui::SliderFloat("Seek", &m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight, 0.f, 1.f, ".%2");
	ImGui::SliderFloat("Wander", &m_pBlendedSteering->GetWeightedBehaviorsRef()[4].weight, 0.f, 1.f, ".%2");

	//End
	ImGui::PopAllowKeyboardFocus();
	ImGui::End();
	
}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	Elite::Vector2 totalPos{};
	for (size_t i{ 0 }; i < (size_t)m_NrOfNeighbors; ++i)
		totalPos += m_Neighbors[i]->GetPosition();

	if (m_NrOfNeighbors > 0)
		totalPos /= (float)m_NrOfNeighbors;

	return totalPos;
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	Elite::Vector2 totalVel{};
	for (size_t i{ 0 }; i < (size_t)m_NrOfNeighbors; ++i)
		totalVel += m_Neighbors[i]->GetLinearVelocity();

	if(m_NrOfNeighbors > 0)
		totalVel /= (float)m_NrOfNeighbors;

	return totalVel;
}

void Flock::SetSeekTarget(TargetData target)
{
	// TODO: set target for Seek behavior
	m_pSeekBehavior->SetTarget(target);
}

float* Flock::GetWeight(ISteeringBehavior* pBehavior) 
{
	if (m_pBlendedSteering)
	{
		auto& weightedBehaviors = m_pBlendedSteering->GetWeightedBehaviorsRef();
		auto it = find_if(weightedBehaviors.begin(),
			weightedBehaviors.end(),
			[pBehavior](BlendedSteering::WeightedBehavior el)
			{
				return el.pBehavior == pBehavior;
			}
		);

		if(it!= weightedBehaviors.end())
			return &it->weight;
	}

	return nullptr;
}
