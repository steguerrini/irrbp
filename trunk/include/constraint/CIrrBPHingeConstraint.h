#ifndef CIrrBP_HINGE_CONST_H_
#define CIrrBP_HINGE_CONST_H_

#include "constraint/CIrrBPConstraint.h"

class CIrrBPHingeConstraint : public CIrrBPConstraint
{
public:
		CIrrBPHingeConstraint(CIrrBPRigidBody * bodyA,const vector3df & pivotInA,const vector3df & axisInA);
		CIrrBPHingeConstraint(CIrrBPRigidBody * bodyA,CIrrBPRigidBody * bodyB,const vector3df & pivotInA,const vector3df & pivotInB,const vector3df & axisInA,const vector3df & axisInB);
		void drop(){delete this;}
protected:
	vector3df m_axisA;
	vector3df m_axisB;
};
#endif