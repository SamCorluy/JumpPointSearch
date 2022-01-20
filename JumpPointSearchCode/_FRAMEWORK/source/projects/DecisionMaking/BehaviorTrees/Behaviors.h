/*=============================================================================*/
// Copyright 2020-2021 Elite Engine
/*=============================================================================*/
// Behaviors.h: Implementation of certain reusable behaviors for the BT version of the Agario Game
/*=============================================================================*/
#ifndef ELITE_APPLICATION_BEHAVIOR_TREE_BEHAVIORS
#define ELITE_APPLICATION_BEHAVIOR_TREE_BEHAVIORS
//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "framework/EliteMath/EMath.h"
#include "framework/EliteAI/EliteDecisionMaking/EliteBehaviorTree/EBehaviorTree.h"
#include "projects/Shared/Agario/AgarioAgent.h"
#include "projects/Shared/Agario/AgarioFood.h"
#include "projects/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

//-----------------------------------------------------------------
// Behaviors
//-----------------------------------------------------------------

//ACTIONS
Elite::BehaviorState ChangeToWander(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	pBlackboard->GetData("Agent", pAgent);

	if (!pAgent)
		return Elite::BehaviorState::Failure;

	pAgent->SetToWander();

	return Elite::BehaviorState::Success;
}

Elite::BehaviorState ChangeToSeek(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	pBlackboard->GetData("Agent", pAgent);

	Elite::Vector2 target;
	pBlackboard->GetData("Target", target);

	if (!pAgent)
		return Elite::BehaviorState::Failure;

	pAgent->SetToSeek(target);

	return Elite::BehaviorState::Success;
}

Elite::BehaviorState ChangeToFlee(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	pBlackboard->GetData("Agent", pAgent);

	AgarioAgent* target;
	pBlackboard->GetData("AgentFleeTarget", target);

	if (!pAgent)
		return Elite::BehaviorState::Failure;

	pAgent->SetToFlee(target->GetPosition());

	return Elite::BehaviorState::Success;
}

//CONDITIONALS
bool IsCloseToFood(Elite::Blackboard* pBlackboard)
{
	std::vector<AgarioFood*>* foodVec;
	pBlackboard->GetData("FoodVec", foodVec);

	AgarioAgent* pAgent{ nullptr };
	pBlackboard->GetData("Agent", pAgent);

	if (!pAgent || (*foodVec).size() < 1)
		return false;

	AgarioFood* closestFood = (*foodVec)[0];
	float closestDistanceSquared = DistanceSquared(closestFood->GetPosition(), pAgent->GetPosition());
	for (size_t i = 1; i < (*foodVec).size(); i++)
	{
		float agentToFood = DistanceSquared((*foodVec)[i]->GetPosition(), pAgent->GetPosition());
		if (closestDistanceSquared > agentToFood)
		{
			closestFood = (*foodVec)[i];
			closestDistanceSquared = agentToFood;
		}
	}

	const float closeToFoodRange{ 20.f };
	if (closestDistanceSquared < (closeToFoodRange + pAgent->GetRadius()) * (closeToFoodRange + pAgent->GetRadius()))
	{
		pBlackboard->ChangeData("Target", closestFood->GetPosition());
		return true;
	}

	return false;
}

bool IsCloseToBiggerEnemy(Elite::Blackboard* pBlackboard)
{
	std::vector<AgarioAgent*>* agentVec;
	pBlackboard->GetData("AgentsVec", agentVec);

	AgarioAgent* pAgent{ nullptr };
	pBlackboard->GetData("Agent", pAgent);

	if (!pAgent || (*agentVec).size() < 1)
		return false;

	AgarioAgent* closestAgent = (*agentVec)[0];
	float closestDistanceSquared = DistanceSquared(closestAgent->GetPosition(), pAgent->GetPosition());
	for (size_t i = 1; i < (*agentVec).size(); i++)
	{
		if (closestAgent->GetRadius() <= pAgent->GetRadius())
		{
			if ((*agentVec)[i]->GetRadius() > pAgent->GetRadius())
			{
				closestAgent = (*agentVec)[i];
				closestDistanceSquared = DistanceSquared(closestAgent->GetPosition(), pAgent->GetPosition());
			}
		}
		else
		{
			float agentToAgent = DistanceSquared((*agentVec)[i]->GetPosition(), pAgent->GetPosition());
			if ((*agentVec)[i]->GetRadius() > pAgent->GetRadius() && closestDistanceSquared > agentToAgent)
			{
				closestAgent = (*agentVec)[i];
				closestDistanceSquared = agentToAgent;
			}
		}
	}

	const float closeToEnemyRange{ 40.f };
	if (closestAgent->GetRadius() > pAgent->GetRadius() && closestDistanceSquared < (closeToEnemyRange + pAgent->GetRadius() + closestAgent->GetRadius()) * (closeToEnemyRange + pAgent->GetRadius() + closestAgent->GetRadius()))
	{
		pBlackboard->ChangeData("AgentFleeTarget", closestAgent);
		return true;
	}

	return false;
}
#endif