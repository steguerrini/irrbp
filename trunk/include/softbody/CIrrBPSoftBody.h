#ifndef _CSOFTBODY_H
#define _CSOFTBODY_H

#include <btBulletCollisionCommon.h>

#include <BulletSoftBody\btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody\btSoftBodyHelpers.h>
#include <body\CIrrBpRigidBody.h>
#include <irrlicht.h>
#include "convert.h"
#include "types.h"


using namespace irr;
using namespace core;
using namespace scene;
using namespace io;
using namespace gui;
using namespace video;
using namespace bullet;

class CIrrBPSoftBody
{
public:
	CIrrBPSoftBody(){};
	virtual void drop() = 0;
	virtual void addForce(const vector3df& force);
	virtual void addForce(const vector3df& force,int node);
	virtual void addVelocity(const vector3df& velocity);
	virtual void setVelocity(const vector3df& velocity);
	virtual void addVelocity(const vector3df& velocity,int node);
	void appendAnchor(CIrrBPRigidBody * body);
	void appendAnchor(CIrrBPRigidBody * body,int pos);
	void setMass(irr::s32 node, irr::f32 mass);
	virtual btSoftBody * getBodyPtr() { return m_softBody;}
	btSoftBody::Config getConfig() { return m_softBody->m_cfg;}
	void setConfig(btSoftBody::Config newConfig) { m_softBody->m_cfg = newConfig;}

protected:
	btSoftBody * m_softBody;
};

#endif