/*=============================================================================*/
// Copyright 2020-2021 Elite Engine
/*=============================================================================*/
// StatesAndTransitions.h: Implementation of the state/transition classes
/*=============================================================================*/
#ifndef ELITE_APPLICATION_FSM_STATES_TRANSITIONS
#define ELITE_APPLICATION_FSM_STATES_TRANSITIONS

#include "projects/Shared/Agario/AgarioAgent.h"
#include "projects/Shared/Agario/AgarioFood.h"
#include "projects/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "framework/EliteAI/EliteData/EBlackboard.h"

//------------
//---STATES---
//------------
class WanderState : public Elite::FSMState
{
public:
	virtual void OnEnter(Elite::Blackboard* pBlackboard) override
	{
		std::cout << "entering wander" << std::endl;
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return;

		pAgent->SetToWander();
	}
	virtual void OnExit(Elite::Blackboard* pBlackboard) override
	{
		std::cout << "exiting wander" << std::endl << std::endl;
	}
};

class SeekState : public Elite::FSMState
{
public:
	virtual void OnEnter(Elite::Blackboard* pBlackboard) override
	{
		std::cout << "entering seek" << std::endl;
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return;
		AgarioFood* foodTarget{ nullptr };
		success = pBlackboard->GetData("FoodTarget", foodTarget);
		if (!success || !foodTarget)
			return;

		pAgent->SetToSeek(foodTarget->GetPosition());
	}
	virtual void OnExit(Elite::Blackboard* pBlackboard) override
	{
		std::cout << "exiting seek" << std::endl << std::endl;
	}
};
class FleeState : public Elite::FSMState
{
public:
	virtual void OnEnter(Elite::Blackboard* pBlackboard) override
	{
		std::cout << "entering flee" << std::endl;
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return;
		AgarioAgent* enemyTarget{ nullptr };
		success = pBlackboard->GetData("AgentFleeTarget", enemyTarget);
		if (!success || !enemyTarget)
			return;

		pAgent->SetToFlee(enemyTarget->GetPosition());
	}
	virtual void Update(Elite::Blackboard* pBlackboard, float deltaTime) override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return;
		AgarioAgent* enemyTarget{ nullptr };
		success = pBlackboard->GetData("AgentFleeTarget", enemyTarget);
		if (!success || !enemyTarget)
			return;

		pAgent->SetToFlee(enemyTarget->GetPosition());
	}
	virtual void OnExit(Elite::Blackboard* pBlackboard) override
	{
		std::cout << "exiting flee" << std::endl << std::endl;
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return;
		pBlackboard->ChangeData("AgentFleeTarget", nullptr);
	}
};

class HuntState : public Elite::FSMState
{
public:
	virtual void OnEnter(Elite::Blackboard* pBlackboard) override
	{
		std::cout << "entering hunt" << std::endl;
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return;
		AgarioAgent* enemyTarget{ nullptr };
		success = pBlackboard->GetData("AgentFleeTarget", enemyTarget);
		if (!success || !enemyTarget)
			return;

		pAgent->SetToSeek(enemyTarget->GetPosition());
	}
	virtual void Update(Elite::Blackboard* pBlackboard, float deltaTime) override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return;
		AgarioAgent* enemyTarget{ nullptr };
		success = pBlackboard->GetData("AgentFleeTarget", enemyTarget);
		if (!success || enemyTarget->CanBeDestroyed())
			return;

		pAgent->SetToSeek(enemyTarget->GetPosition());
	}
	virtual void OnExit(Elite::Blackboard* pBlackboard) override
	{
		std::cout << "exiting hunt" << std::endl << std::endl;
		/*AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return;
		pBlackboard->ChangeData("AgentFleeTarget", nullptr);*/
	}
};
//-----------------
//---TRANSITIONS---
//-----------------
class CloseToFood : public Elite::FSMTransition
{
public:
	bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return false;
		std::vector<AgarioFood*>* foodTarget{ nullptr };
		success = pBlackboard->GetData("FoodVec", foodTarget);
		if (!success || !foodTarget)
			return false;

		AgarioFood* temp{};
		for (auto food : *foodTarget)
		{
			if (temp && Distance(pAgent->GetPosition(), temp->GetPosition()) > Distance(pAgent->GetPosition(), food->GetPosition()))
				temp = food;
			else if (!temp)
				temp = food;
		}

		if (Distance(temp->GetPosition(), pAgent->GetPosition()) > 40.f)
			return false;
		pBlackboard->ChangeData("FoodTarget", temp);
		return true;
	}
};

class FoodToWander : public Elite::FSMTransition
{
public:
	bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return false;

		AgarioFood* foodTarget{ nullptr };
		success = pBlackboard->GetData("FoodTarget", foodTarget);
		if (!success)
			return false;

		return foodTarget->CanBeDestroyed();
	}
};

class ToFlee : public Elite::FSMTransition
{
public:
	bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return false;
		std::vector<AgarioAgent*>* enemyTarget{ nullptr };
		success = pBlackboard->GetData("AgentsVec", enemyTarget);
		if (!success || !enemyTarget)
			return false;

		AgarioAgent* temp{};
		for (auto enemy : *enemyTarget)
		{
			if (temp && enemy->GetRadius() > pAgent->GetRadius() && Distance(pAgent->GetPosition(), temp->GetPosition()) > Distance(pAgent->GetPosition(), enemy->GetPosition()))
				temp = enemy;
			else if (!temp && enemy->GetRadius() > pAgent->GetRadius())
				temp = enemy;
		}
		if (!temp)
			return false;
		pBlackboard->ChangeData("AgentFleeTarget", temp);
		return Distance(temp->GetPosition(), pAgent->GetPosition()) < pAgent->GetRadius() + 15.f;
	}
};

class ExitFlee : public Elite::FSMTransition
{
public:
	bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return false;

		AgarioAgent* fleeTarget{ nullptr };
		success = pBlackboard->GetData("AgentFleeTarget", fleeTarget);
		if (!success)
			return false;

		return Distance(fleeTarget->GetPosition(), pAgent->GetPosition()) > pAgent->GetRadius() + 15.f;
	}
};

class ToHunt : public Elite::FSMTransition
{
public:
	bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return false;
		std::vector<AgarioAgent*>* enemyTarget{ nullptr };
		success = pBlackboard->GetData("AgentsVec", enemyTarget);
		if (!success || !enemyTarget)
			return false;

		AgarioAgent* temp{};
		for (auto enemy : *enemyTarget)
		{
			if (temp && enemy->GetRadius() + 1.f < pAgent->GetRadius() && Distance(pAgent->GetPosition(), temp->GetPosition()) > Distance(pAgent->GetPosition(), enemy->GetPosition()))
				temp = enemy;
			else if (!temp && enemy->GetRadius() + 1.f < pAgent->GetRadius())
				temp = enemy;
		}
		if (!temp)
			return false;
		pBlackboard->ChangeData("AgentFleeTarget", temp);
		return Distance(temp->GetPosition(), pAgent->GetPosition()) < pAgent->GetRadius() + 15.f;
	}
};

class ExitHunt : public Elite::FSMTransition
{
public:
	bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		if (!success)
			return false;

		AgarioAgent* enemyTarget{ nullptr };
		success = pBlackboard->GetData("AgentFleeTarget", enemyTarget);
		if (!success)
			return false;

		return enemyTarget->CanBeDestroyed();
	}
};

#endif