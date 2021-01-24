/*=============================================================================*/
// Copyright 2017-2018 Elite Engine
// Authors: Matthieu Delaere, Thomas Goussaert
/*=============================================================================*/
// SteeringBehaviors.h: SteeringBehaviors interface and different implementations
/*=============================================================================*/
#ifndef ELITE_STEERINGBEHAVIORS
#define ELITE_STEERINGBEHAVIORS

//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "SteeringHelpers.h"
#include "Obstacle.h"

class SteeringAgent;
using namespace Elite;

#pragma region **ISTEERINGBEHAVIOR** (BASE)
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) = 0;

	//Seek Functions
	void SetTarget(const TargetData& target) { m_Target = target; }

	template<class T, typename std::enable_if<std::is_base_of<ISteeringBehavior, T>::value>::type* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	TargetData m_Target;
};
#pragma endregion

///////////////////////////////////////
//SEEK
//****
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() = default;

	//Seek Behaviour
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
};

//////////////////////////
//WANDER
//******
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() = default;

	//Wander Behavior
	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;

	void SetWanderOffset(float offset) { m_Offset = offset; }
	void SetWanderRadius(float radius) { m_Radius = radius; }
	void SetMaxAngleChange(float rad) { m_AngleChange = rad; }

protected:
	float m_Offset = 6.f;
	float m_Radius = 4.f;
	float m_AngleChange = ToRadians(45); //max wanderAngle per frame
	float m_WanderAngle = 0.f;
};

//////////////////////////
//Flee
//****
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() = default;

	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
};

//////////////////////////
//Arrive
//******
class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	virtual ~Arrive() = default;

	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
	void SetSlowRadius(float slowRadius);
private:
	float m_SlowRadius = 10.0f;

};

/////////////////////////
//Face
//****
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() = default;

	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
};
///////////////////////////
//Pursuit
//*******
class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() = default;

	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
};

//////////////////////////
//Evade
//*****
class Evade : public ISteeringBehavior
{
public:
	Evade() = default;
	virtual ~Evade() = default;

	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
private:
	float m_FleeRadius = 15.f;
};

//Avoid Obstacle
class AvoidObstacle : public Seek
{
public:
	AvoidObstacle(std::vector<Obstacle*>* pObstacles);
	virtual ~AvoidObstacle() = default;

	SteeringOutput CalculateSteering(float deltaT, SteeringAgent* pAgent) override;
private:
	std::vector<Obstacle*>* m_pObstacles = nullptr;
	float m_MaxAhead = 10.f;
	Elite::Mat22 m_Clock = { cos(_Pi / 2.f), sin(_Pi / 2.f), -sin(_Pi / 2.f), cos(_Pi / 2.f) };
	Elite::Mat22 m_AntiClock = { cos(_Pi / 2.f), -sin(_Pi / 2.f), sin(_Pi / 2.f), cos(_Pi / 2.f) };

	Elite::Vector2 CalculateAvoidSteering(SteeringAgent* pAgent);
	Obstacle* findObstacleToAvoid(SteeringAgent* pAgent, Elite::Vector2 leftPos, Elite::Vector2 rightPos);
	bool IsCloser(SteeringAgent* pAgent, float& currentDistance, const Elite::Vector2& startPoint, const Elite::Vector2& endPoint, Obstacle* pObstacle);
};
#endif


