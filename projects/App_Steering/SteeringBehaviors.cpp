//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "SteeringAgent.h"

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = (m_Target).Position - pAgent->GetPosition(); //Desired Velocity
	steering.LinearVelocity.Normalize(); //Normalize Desired Velocity
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed(); //Rescale to Max Speed

	//DEBUG RENDERING
	if (pAgent->CanRenderBehavior())
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 }, 0.4f);

	return steering;
}

//WANDER (base> SEEK)
//******
SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	Elite::Vector2 wanderCircle{};
	wanderCircle = pAgent->GetLinearVelocity();
	wanderCircle.Normalize();
	wanderCircle *= m_Offset;
	wanderCircle += pAgent->GetPosition();
	//wanderCircle.x = pAgent->GetPosition().x + m_Offset * (std::cos(pAgent->GetOrientation() - float(_Pi / 2)));
	//wanderCircle.y = pAgent->GetPosition().y + m_Offset * (std::sin(pAgent->GetOrientation() - float(_Pi / 2)));

	m_WanderAngle += randomFloat() * m_AngleChange - (m_AngleChange / 2);
	m_Target.Position.x = wanderCircle.x + std::cos(m_WanderAngle) * m_Radius;
	m_Target.Position.y = wanderCircle.y + std::sin(m_WanderAngle) * m_Radius;

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawCircle(wanderCircle, m_Radius, Elite::Color{ 0.f,1.f,1.f }, 0.4f);

	}


	return Seek::CalculateSteering(deltaT, pAgent);
}

//Flee
//****
SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = pAgent->GetPosition() - (m_Target).Position;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	//DEBUG RENDERER
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 }, 0.4f);
	}
	return steering;
}

//Arrive
//******
SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	if (steering.LinearVelocity.Magnitude() < m_SlowRadius)
	{
		steering.LinearVelocity.Normalize();
		steering.LinearVelocity *= steering.LinearVelocity.Magnitude();
	}
	else
	{
		steering.LinearVelocity.Normalize();
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	}

	//DEBUG RENDERER
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 }, 0.4f);
	}
	return steering;
}

void Arrive::SetSlowRadius(float slowRadius)
{
	m_SlowRadius = slowRadius;
}

//Face
//****
SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	pAgent->SetAutoOrient(false);
	Elite::Vector2 toTargetVector{ m_Target.Position - pAgent->GetPosition() };
	toTargetVector.Normalize();
	Elite::Vector2 agentVector{ std::cos(pAgent->GetOrientation() - float(_Pi / 2)), std::sin(pAgent->GetOrientation() - float(_Pi / 2)) };

	float angle = std::acos(toTargetVector.Dot(agentVector) / (toTargetVector.Magnitude() * agentVector.Magnitude())) * 360 / (2 * float(_Pi));
	float velocitySign = agentVector.Cross(toTargetVector);
	if (angle > 1.0f || angle < -1.0f)
	{
		steering.AngularVelocity = velocitySign * pAgent->GetMaxAngularSpeed();
	}
	return steering;
}

//PURSUIT
//*******
SteeringOutput Pursuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition() + m_Target.LinearVelocity;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	return steering;
}

//Evade
//*****
SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	auto distanceToTarget = Distance(pAgent->GetPosition(), m_Target.Position);

	SteeringOutput steering;
	if (distanceToTarget > m_FleeRadius)
	{
		steering.IsValid = false;
		return steering;
	}
	steering.LinearVelocity = pAgent->GetPosition() - m_Target.Position + m_Target.LinearVelocity;
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	return steering;
}


//Avoid Collision
//***************
AvoidObstacle::AvoidObstacle(std::vector<Obstacle*>* pObstacles)
	: m_pObstacles{ pObstacles }
{

}
SteeringOutput AvoidObstacle::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering;
	steering.LinearVelocity += Seek::CalculateSteering(deltaT, pAgent).LinearVelocity;
	steering.LinearVelocity += CalculateAvoidSteering(pAgent);
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	return steering;
}

Elite::Vector2 AvoidObstacle::CalculateAvoidSteering(SteeringAgent* pAgent)
{
	Elite::Vector2 collisionAvoidSteering, leftPos, rightPos;

	leftPos = m_AntiClock * pAgent->GetDirection() * pAgent->GetRadius();
	rightPos = m_Clock * pAgent->GetDirection() * pAgent->GetRadius();

	float dynamicLength = pAgent->GetLinearVelocity().Magnitude() / pAgent->GetMaxLinearSpeed();


	Obstacle* pObstacle = findObstacleToAvoid(pAgent, leftPos, rightPos);
	if (pObstacle != nullptr)
	{
		Elite::Vector2 aheadVector = pAgent->GetDirection() * dynamicLength * m_MaxAhead;
		collisionAvoidSteering = m_AntiClock* pAgent->GetDirection();

		Elite::Vector2 vectorToCenter = pAgent->GetPosition() - pObstacle->GetCenter();
		float sign = Cross(aheadVector, vectorToCenter);
		collisionAvoidSteering *= sign;
		Elite::Normalize(collisionAvoidSteering);
		collisionAvoidSteering *= pObstacle->GetRadius() * pAgent->GetLinearVelocity().Magnitude();
	}
	return collisionAvoidSteering;
}

Obstacle* AvoidObstacle::findObstacleToAvoid(SteeringAgent* pAgent, Elite::Vector2 leftPos, Elite::Vector2 rightPos)
{
	Obstacle* pCloseObstacle = nullptr;
	float closestDistance = FLT_MAX, distance;

	float dynamicLength = pAgent->GetLinearVelocity().Magnitude() / pAgent->GetMaxLinearSpeed();
	//left ahead line
	Elite::Vector2 startPointLeft = pAgent->GetPosition() + leftPos;
	Elite::Vector2 endPointLeft = pAgent->GetPosition() + leftPos + (pAgent->GetDirection() * dynamicLength * m_MaxAhead);

	//right ahead line
	Elite::Vector2 startPointRight = pAgent->GetPosition() + rightPos;
	Elite::Vector2 endPointRight = pAgent->GetPosition() + rightPos + (pAgent->GetDirection() * dynamicLength * m_MaxAhead);

	for (Obstacle* pObstacle : *m_pObstacles)
	{
		//check left side
		if (IsCloser(pAgent, closestDistance, startPointLeft, endPointLeft, pObstacle))
		{
			pCloseObstacle = pObstacle;
			continue;
		}

		//if left failed, check right
		if (IsCloser(pAgent, closestDistance, startPointRight, endPointRight, pObstacle))
		{
			pCloseObstacle = pObstacle;
			continue;
		}
	}

	//DEBUG RENDERER
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(startPointLeft, pAgent->GetDirection(), dynamicLength * m_MaxAhead, { 0.5,1,0.5 }, 0.4f);
		DEBUGRENDERER2D->DrawDirection(startPointRight, pAgent->GetDirection(), dynamicLength * m_MaxAhead, { 0.5,1,0.5 }, 0.4f);
	}

	return pCloseObstacle;
}

bool AvoidObstacle::IsCloser(SteeringAgent* pAgent, float& currentDistance, const Elite::Vector2& startPoint, const Elite::Vector2& endPoint, Obstacle* pObstacle)
{
	if (IsSegmentIntersectingWithCircle(startPoint, endPoint, pObstacle->GetCenter(), pObstacle->GetRadius()))
	{
		float distance = Distance(pAgent->GetPosition(), pObstacle->GetCenter());
		if (distance < currentDistance)
		{
			currentDistance = distance;
			return true;
		}
	}
	return false;
}