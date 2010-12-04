#ifndef _CRIGIDBODY_H
#define _CRIGIDBODY_H

#include "CIrrBPCollisionObject.h"
#include <btBulletCollisionCommon.h>
#include <BulletCollision\Gimpact\btGImpactShape.h>
#include <BulletCollision\Gimpact\btGImpactCollisionAlgorithm.h>


class CIrrBPAnimator;

class CIrrBPRigidBody : public CIrrBPCollisionObject
{
public:
	CIrrBPRigidBody();
	virtual ~CIrrBPRigidBody();
	virtual void drop() = 0;

	virtual void  applyTorque (const vector3df &torque) ;
	virtual void  applyForce (const vector3df &force, const vector3df &rel_pos) ;
	virtual void  applyCentralImpulse (const vector3df &impulse) ;
	virtual void  applyCentralForce(const vector3df &force);
	virtual void  applyTorqueImpulse (const vector3df &torque) ;
	virtual void  applyImpulse (const vector3df &impulse, const vector3df &rel_pos) ;

	virtual btRigidBody * getBodyPtr() {return m_RigidBody;};
	virtual CMotionState * getMotionState() {return m_MotionState;}

	virtual bool isStaticObject() { return m_RigidBody->isStaticObject();}
	virtual ISceneNode * getIrrlichtNode() { return m_IrrSceneNode;}

	vector3df getPosition();
	void setPosition(const vector3df & newPos);
	void setKinematic(bool isKinematic);
	bool isKinematic();
protected:
	irr::f32 getAutomaticCCDSSR();
	irr::f32 getAutomaticCCDMT();
	void setAutomaticCCD();
	ISceneNode * m_IrrSceneNode;
	CMotionState * m_MotionState;
	btCollisionShape * m_Shape;
	btRigidBody * m_RigidBody;
	
	bool kinematic;


};



#endif