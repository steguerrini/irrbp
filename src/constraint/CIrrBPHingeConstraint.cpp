#include "constraint/CIrrBPHingeConstraint.h"
#include "body/CIrrBPRigidBody.h"

CIrrBPHingeConstraint::CIrrBPHingeConstraint(CIrrBPRigidBody * bodyA,const vector3df & pivotInA,const vector3df & axisInA)
{
	m_Constraint = new btHingeConstraint(*(bodyA->getBodyPtr()),irrVectorToBulletVector(pivotInA),irrVectorToBulletVector(axisInA));
	this->m_bodyA = bodyA;
	this->m_axisA = axisInA;
	this->m_pivotA = pivotInA;
}

CIrrBPHingeConstraint::CIrrBPHingeConstraint(CIrrBPRigidBody * bodyA,CIrrBPRigidBody * bodyB,const vector3df & pivotInA,const vector3df & pivotInB,const vector3df & axisInA,const vector3df & axisInB)
{
	m_Constraint = new btHingeConstraint(*(bodyA->getBodyPtr()),*(bodyB->getBodyPtr()),
											irrVectorToBulletVector(pivotInA),irrVectorToBulletVector(pivotInB),
											irrVectorToBulletVector(axisInA),irrVectorToBulletVector(axisInB));
	this->m_bodyA = bodyA;
	this->m_axisA = axisInA;
	this->m_pivotA = pivotInA;
	this->m_bodyB = bodyB;
	this->m_axisB = axisInB;
	this->m_pivotB = pivotInB;
}
