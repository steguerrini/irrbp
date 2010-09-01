#include "constraint/CIrrBPP2PConstraint.h"
#include "body/CIrrBPRigidBody.h"

CIrrBPP2PConstraint::CIrrBPP2PConstraint(CIrrBPRigidBody *  bodyA,const vector3df & pivotInA)
{
	m_Constraint = new btPoint2PointConstraint(*(bodyA->getBodyPtr()),irrVectorToBulletVector(pivotInA));
	this->m_bodyA = bodyA;
	this->m_pivotA = pivotInA;
}
CIrrBPP2PConstraint::CIrrBPP2PConstraint(CIrrBPRigidBody * bodyA, CIrrBPRigidBody * bodyB, const vector3df & pivotInA, const vector3df & pivotInB)
{
	m_Constraint = new btPoint2PointConstraint(*(bodyA->getBodyPtr()),*(bodyB->getBodyPtr()),irrVectorToBulletVector(pivotInA),irrVectorToBulletVector(pivotInB));
	this->m_bodyA = bodyA;
	this->m_pivotA = pivotInA;
	this->m_bodyB = bodyB;
	this->m_pivotB = pivotInB;
}
