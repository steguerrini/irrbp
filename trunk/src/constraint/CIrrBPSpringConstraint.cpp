#include "constraint/CIrrBPSpringConstraint.h"
#include "body/CIrrBPRigidBody.h"

#include <iostream>
CIrrBPSpringConstraint::CIrrBPSpringConstraint(CIrrBPRigidBody *bodyA, CIrrBPRigidBody *bodyB,const irr::core::vector3df & pivotInA,const irr::core::vector3df & pivotInB,bool autoadapt) 
{
	m_bodyA = bodyA;
	m_bodyB = bodyB;
	btTransform frameInA, frameInB;
	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(bullet::irrVectorToBulletVector(pivotInA));
	frameInB = btTransform::getIdentity();
	frameInB.setOrigin(bullet::irrVectorToBulletVector(pivotInB));
	
	/*We need to do the same trick of slide constraint*/
	
	short sobj = bodyA->isStaticObject() ? 1 : bodyB->isStaticObject() ? 2 : 0; //Which obj is static? 0 = No static objects. 1 = Body A. 2 = Body B
	
	if(sobj != 0 && autoadapt) //If an object is static, and the user want to auto-adapt the object (to get a non-only-orthogonal obj) we need to do some calc...
	{
		irr::core::vector3df n1,n2,ris,rot;

		n1 = bodyA->getIrrlichtNode()->getAbsolutePosition();
		n2 = bodyB->getIrrlichtNode()->getAbsolutePosition();

		ris = n2-n1;
		rot = (ris.getHorizontalAngle()); //Irrlicht' object rotation
		
		rot.X = irr::core::degToRad(rot.X);
		rot.Y = irr::core::degToRad(rot.Y + 90); //Adapting to Bullet Reference System
		rot.Z = irr::core::degToRad(rot.Z);
		
		/*Setting rotations*/
		frameInA.setRotation(btQuaternion(rot.Y,0,rot.X));
		frameInB.setRotation(btQuaternion(rot.Y,0,rot.X));
	}
	m_fixedConstraint = new btGeneric6DofSpringConstraint(*bodyA->getBodyPtr(), *bodyB->getBodyPtr(), frameInA, frameInB, true);
	m_Constraint = m_fixedConstraint;
}