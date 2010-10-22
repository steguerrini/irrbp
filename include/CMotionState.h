#ifndef CMOTIONSTATE_H_
#define CMOTIONSTATE_H_

#include "convert.h"

#include <btBulletCollisionCommon.h>

#include <irrlicht.h>
#include "convert.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace io;
using namespace gui;
using namespace video;
using namespace bullet;



class CIrrBPRigidBody;

///Should not be used. Only for internal use.
class CMotionState : public btDefaultMotionState
{
    public:
		CMotionState(CIrrBPRigidBody * body,const btTransform &startTrans=btTransform::getIdentity(), const btTransform &centerOfMassOffset=btTransform::getIdentity());

        virtual ~CMotionState();

		virtual void getWorldTransform(btTransform &worldTrans);
        virtual void setWorldTransform(const btTransform &worldTrans);

    protected:
		CIrrBPRigidBody * m_body;
		ISceneNode * m_irrNode;
};

#endif 
