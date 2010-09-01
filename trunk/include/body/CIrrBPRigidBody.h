#ifndef _CRIGIDBODY_H
#define _CRIGIDBODY_H

#include <btBulletCollisionCommon.h>
#include <BulletCollision\Gimpact\btGImpactShape.h>
#include <BulletCollision\Gimpact\btGImpactCollisionAlgorithm.h>
#include <irrlicht.h>
#include "convert.h"
#include "types.h"
#include "CMotionState.h"
#include <iostream>

using namespace std;

using namespace irr;
using namespace core;
using namespace scene;
using namespace io;
using namespace gui;
using namespace video;
using namespace bullet;

class CIrrBPAnimator;

static int UNIQUE_BODY_ID_GENERATOR = 0;
class CIrrBPRigidBody
{
public:
	CIrrBPRigidBody();
	
	/*!
	Drop Function.
	This function should not be used. The destructor will be called automatically by the World Object.
	*/
	virtual void drop() = 0;

	virtual void  applyTorque (const vector3df &torque) ;
	virtual void  applyForce (const vector3df &force, const vector3df &rel_pos) ;
	virtual void  applyCentralImpulse (const vector3df &impulse) ;
	virtual void  applyCentralForce(const vector3df &force);
	virtual void  applyTorqueImpulse (const vector3df &torque) ;
	virtual void  applyImpulse (const vector3df &impulse, const vector3df &rel_pos) ;

	virtual void setName(const stringc & newName)	{		m_BodyName = newName;	}
	virtual void setID(const irr::s32 & newID)	{		m_BodyId = newID;	}
	virtual const c8* getName() const { return m_BodyName.c_str();}
	virtual irr::s32 getID() const { return m_BodyId;}

	virtual irr::u32 getUniqueID() const { return m_uBodyId;}

	virtual btRigidBody * getBodyPtr() {return m_RigidBody;};
	virtual CMotionState * getMotionState() {return m_MotionState;}

	virtual void setCollisionParam(irr::s32 newP){ cParam = newP;}
	virtual irr::s32 getCollisionParam(){ return cParam;}

	virtual bool isStaticObject() { return m_RigidBody->isStaticObject();}
	virtual ISceneNode * getIrrlichtNode() { return m_IrrSceneNode;}
	/*! 
		Returns the status of the node. 
	*/
	virtual bool isValid(){ return m_BodyStatus;}

	/*! 
		Setting the value to false will cause the node's deletion.
	*/
	virtual void setValidStatus (bool newStat) { m_BodyStatus = newStat;}

	virtual irr::u32 getAnimatorsNr() {return m_Animators.size();}
	virtual const array<CIrrBPAnimator *> getAnimators() const { return m_Animators; }
	virtual void addAnimator(CIrrBPAnimator * anim);
protected:
	ISceneNode * m_IrrSceneNode;
	CMotionState * m_MotionState;
	btCollisionShape * m_Shape;
	btRigidBody * m_RigidBody;
	irr::core::stringc m_BodyName;
	irr::s32 m_BodyId; //User setted BodyID
	irr::u32 m_uBodyId; //Unique BodyID
	bool m_BodyStatus;

	irr::s32 cParam;//Collision Param

	array<CIrrBPAnimator *> m_Animators;

};



#endif