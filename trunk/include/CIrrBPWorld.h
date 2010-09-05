#ifndef _CBULLETWORLD_H
#define _CBULLETWORLD_H

#define IrrBP_MAJOR_VERSION 0
#define IrrBP_MINOR_VERSION 0 
#define IrrBP_REVISION_VERSION 1

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision\CollisionDispatch\btGhostObject.h>
#include "BulletSoftBody\btSoftRigidDynamicsWorld.h"
#include "BulletCollision\CollisionDispatch\btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision\CollisionDispatch\btCollisionDispatcher.h"
#include "body\CIrrBPRigidBody.h"
#include "animator\CIrrBPAnimator.h"
#include "constraint\CIrrBPConstraint.h"
#include "softbody\CIrrBPSoftBody.h"
#include "BulletSoftBody\btSoftBodyRigidBodyCollisionConfiguration.h"
#include "CIrrBPDebugDrawer.h"

#include <irrlicht.h>
#include "convert.h"
#include "types.h"
#include <iostream>
using namespace std;

using namespace irr;
using namespace core;
using namespace scene;
using namespace io;
using namespace gui;
using namespace video;
using namespace bullet;

class CIrrBPWorld
{
public:

	 /*!
        Constructor.
        @param device A pointer to a Irrlicht's device
        @param Gravity World's Gravity .
    */
	CIrrBPWorld(IrrlichtDevice *device,const vector3df & Gravity);

	~CIrrBPWorld();
	
	/*!
		Steps the simulation.
		It must be called each frame loop to step the bullet' simulation.
	*/
    void stepSimulation();

	/*!
		Adds a rigid Body to the world.
		@param body A pointer to the body that needs to be added
	*/
	irr::u32 addRigidBody(CIrrBPRigidBody * body);

	void addRigidBodyConstraint(CIrrBPConstraint * constraint);
	/*!
		Removes a rigid Body from the world.
		Please note that the body's Scene Node won't be dropped.
		@param body A pointer to the body that needs to be deleted.
	*/
	void removeRigidBody(CIrrBPRigidBody *body);


	void addSoftBody(CIrrBPSoftBody * sbody);

	/*!
		Gets the number of active bodies in the world
		@return number of active bodies
	*/

	irr::u32 getActiveBodies() { return rigidBodies.size();}
	
	/*!
		Gets a rigid Body from a id.
		@param id The id to search for
		@return Pointer to the first rigid body with this id. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPRigidBody * getRigidBodyFromId(irr::s32 id);

	/*!
		Gets a rigid Body from a unique id.
		@param id The unique id to search for
		@return Pointer to the first rigid body with this id. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPRigidBody * getRigidBodyFromUId(irr::u32 uid);


	/*!
		Gets a rigid Body from a name.
		@param name The name to search for
		@return Pointer to the first rigid body with this name. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPRigidBody * getRigidBodyFromName(irr::c8* name);

	/*!
		Verifies if a body is colliding or not.
		@param body body to verify
		@param collMask not yet full implemented.
		@return body colliding status.
	*/
	bool isBodyColliding(CIrrBPRigidBody *body, irr::s32 collMask=-1);
	
	/*!
		Drop the world pointer and all his child.
		Please note that all registered rigid bodies pointers, will be destroyed.
	*/
	void drop() { delete this;}

	void setGravity(const vector3df & newGravity) { World->setGravity(irrVectorToBulletVector(newGravity));}

	/*!
		Only for internal or expert use.
		@return a pointer to the bullet' world object
	*/
	btDiscreteDynamicsWorld* getBulletWorldPtr(){return World;}

	/*! 
		true if world is  going to close
	*/
	bool isClosing;

	/*!
		Creates a new bullet' debug drawer.
		This can be useful for debugging bounding boxes, constraints and contact points
	*/
	void createDebugDrawer();

	/*!
		Steps the debug drawer.
		Must be called between driver->beginScene() and driver->endScene() to update debug' datas.
	*/
	void stepDebugDrawer();

	/*!
		Sets the debug-drawer flags
		You can use the IBP_DEBUG_FLAGS defined in types.h
	*/
	void setDebugDrawerFlags(int flags);

	btSoftBodyWorldInfo & getSoftBodyWorldInfo();

private:
	inline void updateObjects();

	btSoftBodyRigidBodyCollisionConfiguration* CollisionConfiguration;
	//btDiscreteDynamicsWorld* World;	
	btSoftRigidDynamicsWorld * World;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* pairCache;
    btConstraintSolver*	constraintSolver;
	
	array<CIrrBPRigidBody*> rigidBodiesObj;
	array<btRigidBody *> rigidBodies;
	array<CIrrBPSoftBody*> softBodiesObj;
	array<CIrrBPConstraint*> rigidBodiesConst;
	//list<btRigidBody *> rigidBodies;
	ITimer* irrTimer;
	u32 TimeStamp;
    u32 DeltaTime;
	btVector3 Gravity;
	IrrlichtDevice *device;
	IVideoDriver* driver;
	IMeshSceneNode* worldNode;

	CIrrBPDebugDrawer * dDrawer;
	irr::video::SMaterial mat;

	btSoftBodyWorldInfo m_worldInfo;

};

#endif