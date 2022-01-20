//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "../SteeringAgent.h"
#include "../Obstacle.h"
#include "framework\EliteMath\EMatrix2x3.h"
#include <cmath>

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);

	return steering;
}

//FLEE
//****
SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = pAgent->GetPosition() - m_Target.Position;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);


	return steering;
}

//ARRIVE
//****
SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	Elite::Vector2 toTarget = m_Target.Position - pAgent->GetPosition();
	const float distance = toTarget.Magnitude();

	steering.LinearVelocity = toTarget;
	steering.LinearVelocity.Normalize();
	if (distance < m_SlowRadius)
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed() * distance / m_SlowRadius;
	else
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);

	return steering;
}

//FACE
//****
SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	Elite::Vector2 toTarget = (m_Target.Position - pAgent->GetPosition()).GetNormalized();
	const float finalRotation = atan2(toTarget.y, toTarget.x);

	float rotation = finalRotation - pAgent->GetRotation() + (float)E_PI_2;
	while (rotation > E_PI) rotation -= 2.f * (float)E_PI;
	while (rotation < -E_PI) rotation += 2.f * (float)E_PI;

	pAgent->SetAutoOrient(false);
	pAgent->SetAngularVelocity( Elite::Clamp(Elite::ToDegrees(rotation), -pAgent->GetMaxAngularSpeed(), pAgent->GetMaxAngularSpeed()) );
	//pAgent->SetAngularVelocity( rotation );
	steering.AngularVelocity = pAgent->GetAngularVelocity();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);

	return steering;
}

//WANDER
//****
SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	float agentRotation = pAgent->GetRotation() - (float)M_PI * 0.5f;
	Elite::Vector2 circleCenter{cosf(agentRotation), sinf(agentRotation)};
	circleCenter *= m_OffsetDistance;
	circleCenter += pAgent->GetPosition();

	float angleChange{ (rand() % (int)(m_MaxAngleChange * 200.f + 1.f)) / 100.f - m_MaxAngleChange };
	m_WanderAngle += angleChange;

	Elite::Vector2 target{ cosf(m_WanderAngle), sinf(m_WanderAngle) };
	target *= m_Radius;
	target += circleCenter;

	m_Target = TargetData(target);

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);
		DEBUGRENDERER2D->DrawCircle(circleCenter, m_Radius, { 0, 0, 1, 0.5f }, 0.4f);
		DEBUGRENDERER2D->DrawPoint(target, 4.f, { 1, 0, 0, 0.5f }, 0.4f);
	}
	return steering;
}

//PURSUIT
//****
SteeringOutput Pursuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	const float maxDistance{20.f};
	float amplifier{};
	auto distanceToTarget = Distance(pAgent->GetPosition(), m_Target.Position);
	amplifier = distanceToTarget;
	if (amplifier > maxDistance)
		amplifier = maxDistance;

	Elite::Vector2 distanceAheadOfTarget{ (m_Target.GetDirection() + m_Target.LinearVelocity) };
	distanceAheadOfTarget.Normalize();
	distanceAheadOfTarget *= amplifier;

	distanceAheadOfTarget += m_Target.Position;

	if(TargetData(distanceAheadOfTarget).Position.Distance(m_Target.Position) < TargetData(distanceAheadOfTarget).Position.Distance(pAgent->GetPosition()))
		m_Target = TargetData(distanceAheadOfTarget);

	steering.LinearVelocity = (m_Target.Position - pAgent->GetPosition());
	//steering.LinearVelocity = (m_Target.Position - pAgent->GetPosition()) + (m_Target.GetDirection() + m_Target.LinearVelocity);
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);
		DEBUGRENDERER2D->DrawPoint(m_Target.Position, 5.f, { 1, 0, 0, 0.5f }, 0.4f);
		//DEBUGRENDERER2D->DrawPoint(m_Target.Position + (m_Target.GetDirection() + m_Target.LinearVelocity), 5.f, { 1, 0, 0, 0.5f }, 0.4f);
	}

	return steering;
}

//EVADE
//****
SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	auto distanceToTarget = Distance(pAgent->GetPosition(), m_Target.Position);
	if (distanceToTarget > m_EvadeRadius)
	{
		return SteeringOutput(Elite::ZeroVector2, 0.f, false);
	}

	SteeringOutput steering{};

	//steering.LinearVelocity = (-(m_Target.Position - pAgent->GetPosition()) + m_Target.GetDirection() + (m_Target.LinearVelocity)*(distanceToTarget)/m_EvadeRadius);
	//steering.LinearVelocity = (-(m_Target.Position - pAgent->GetPosition())*(distanceToTarget)/m_EvadeRadius);
	steering.LinearVelocity = (-(m_Target.Position - pAgent->GetPosition()));
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	// Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);
		//DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), -steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);
		DEBUGRENDERER2D->DrawPoint(m_Target.Position - m_Target.GetDirection() + (m_Target.LinearVelocity) * (distanceToTarget) / m_EvadeRadius, 5.f, { 1, 0, 0, 0.5f }, 0.4f);
	}

	return steering;
}