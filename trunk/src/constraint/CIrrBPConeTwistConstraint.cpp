#include "constraint/CIrrBPConeTwistConstraint.h"
#include "body/CIrrBPRigidBody.h"

CIrrBPConeTwistConstraint::CIrrBPConeTwistConstraint(CIrrBPRigidBody * bodyA,CIrrBPRigidBody * bodyB,const vector3df & pivotInA,const vector3df & pivotInB)
{
	/*Setting object's members...*/
	this->m_bodyA = bodyA;
	this->m_pivotA = pivotInA;
	this->m_bodyB = bodyB;
	this->m_pivotB = pivotInB;
	
	//Body Transformation
	btTransform transf1,transf2;

	transf1 = transf1.getIdentity();
	transf2 = transf2.getIdentity();

	//Set origins...
	transf1.setOrigin(irrVectorToBulletVector(pivotInA));
	transf2.setOrigin(irrVectorToBulletVector(pivotInB));


	m_fixedConstraint = new btConeTwistConstraint(*bodyA->getBodyPtr(),*bodyB->getBodyPtr(),transf1,transf2);
	m_Constraint = m_fixedConstraint;
}