#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "TheFlock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	m_Target = m_pFlock->GetAverageNeighborPos();

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);

	return steering;
}


//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	Elite::Vector2 vectorSum{};
	SteeringOutput steering{};

	for (size_t i{0}; i < (size_t)m_pFlock->GetNrOfNeighbors(); ++i)
	{
		Elite::Vector2 target{ pAgent->GetPosition() - m_pFlock->GetNeighbors()[i]->GetPosition() };
		float inverse{ 1.f / target.Magnitude() };
		inverse *= inverse;

		target *= inverse;
		vectorSum += target;
	}
	if(m_pFlock->GetNrOfNeighbors() > 0)
		vectorSum /= (float)m_pFlock->GetNrOfNeighbors();

	steering.LinearVelocity = vectorSum;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);

	return steering;
}


//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = m_pFlock->GetAverageNeighborVelocity();

	//steering.LinearVelocity = vectorSum;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);

	return steering;
}
