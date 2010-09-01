#include "constraint/CIrrBPSlideConstraint.h"
#include "body/CIrrBPRigidBody.h"

CIrrBPSlideConstraint::CIrrBPSlideConstraint(CIrrBPRigidBody * bodyA,CIrrBPRigidBody * bodyB,const vector3df & pivotInA,const vector3df & pivotInB,bool autoadapt, bool rotatepiston)
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

	short sobj = bodyA->isStaticObject() ? 1 : bodyB->isStaticObject() ? 2 : 0; //Which obj is static? 0 = No static objects. 1 = Body A. 2 = Body B
	
	if(sobj != 0 && autoadapt) //If an object is static, and the user want to auto-adapt the object (to get a non-only-orthogonal obj) we need to do some calc...
	{
		vector3df n1,n2,ris,rot;

		n1 = bodyA->getIrrlichtNode()->getAbsolutePosition();
		n2 = bodyB->getIrrlichtNode()->getAbsolutePosition();

		ris = n2-n1;
		rot = (ris.getHorizontalAngle()); //Irrlicht' object rotation
		
		rot.X = degToRad(rot.X);
		rot.Y = degToRad(rot.Y + 90); //Adapting to Bullet Reference System
		rot.Z = degToRad(rot.Z);
		
		/*Setting rotations*/
		transf1.setRotation(btQuaternion(rot.Y,0,rot.X));

		if(!rotatepiston)
			transf2.setRotation(btQuaternion(rot.Y,0,rot.X));
	}

	
	m_fixedConstraint = new btSliderConstraint(*bodyA->getBodyPtr(),*bodyB->getBodyPtr(),transf1,transf2,true);
	m_Constraint = m_fixedConstraint;
}