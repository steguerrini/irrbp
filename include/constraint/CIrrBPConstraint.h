#ifndef CIrrBP_CONSTRAINT_H_
#define CIrrBP_CONSTRAINT_H_

#include <btBulletDynamicsCommon.h>
#include <irrlicht.h>
#include "convert.h"


using namespace irr;
using namespace core;
using namespace scene;
using namespace io;
using namespace gui;
using namespace video;


class CIrrBPRigidBody;

class CIrrBPConstraint
{
public:
	CIrrBPConstraint();
	virtual ~CIrrBPConstraint();
	
	/*!
	Drop Function.
	This function should not be used. The destructor will be called automatically by the World Object.
	*/
	virtual void drop() = 0;
    btTypedConstraint * getConstraintPtr();
	CIrrBPRigidBody * getBodyA() { return m_bodyA;}
	CIrrBPRigidBody * getBodyB() { return m_bodyB;}
protected:
	btTypedConstraint * m_Constraint;
	CIrrBPRigidBody * m_bodyA;
	vector3df m_pivotA;
	CIrrBPRigidBody * m_bodyB;
	vector3df m_pivotB;
};


#endif