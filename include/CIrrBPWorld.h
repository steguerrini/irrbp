#ifndef _CBULLETWORLD_H
#define _CBULLETWORLD_H

#define IrrBP_MAJOR_VERSION 0
#define IrrBP_MINOR_VERSION 1
#define IrrBP_REVISION_VERSION 1

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision\CollisionDispatch\btGhostObject.h>
#include "BulletSoftBody\btSoftRigidDynamicsWorld.h"
#include "BulletCollision\CollisionDispatch\btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision\CollisionDispatch\btCollisionDispatcher.h"
#include "CIrrBPCollisionObject.h"
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
	void addRigidBody(CIrrBPRigidBody * body);

	/*!
		Adds a constraint to the world.
		@param constraint A pointer to the constraint that needs to be added
	*/
	void addRigidBodyConstraint(CIrrBPConstraint * constraint);

	/*!
		Removes a rigid Body from the world.
		Please note that the body's Scene Node won't be dropped.
		@param body A pointer to the body that needs to be deleted.
		Only left for backwards compatibility. Use removeCollisionObject instead.
	*/
	void removeRigidBody(CIrrBPRigidBody *body);

	/*!
		Adds a soft Body to the world.
		@param sbody A pointer to the body that needs to be added
	*/
	void addSoftBody(CIrrBPSoftBody * sbody);

	/*!
		Removes a collision object to the world.
		@param cobj A pointer to the object that needs to be removed
	*/
	void removeCollisionObject(CIrrBPCollisionObject * cobj);
	/*! 
		Adds an 'unknown' object to the world.
		You can use that instead of addRigidBody, it will know the object for you.
	*/
	void addCollisionObject(CIrrBPCollisionObject * cobj);


	
	/*!
		Gets a Body from a id.
		@param id The id to search for
		@return Pointer to the first body with this id. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPCollisionObject * getBodyFromId(irr::s32 id);

	/*!
		Gets a Body from a unique id.
		@param id The unique id to search for
		@return Pointer to the first body with this id. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPCollisionObject * getBodyFromUId(irr::u32 uid);


	/*!
		Gets a Body from a name.
		@param name The name to search for
		@return Pointer to the first body with this name. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPCollisionObject * getBodyFromName(irr::c8* name);

	/*!
		Verifies if a body is colliding or not.
		@param body body to verify
		@return body colliding status.
	*/
	bool isBodyColliding(CIrrBPCollisionObject *body);
	
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

	IrrlichtDevice * getIrrDevice() {return device;}

	/*!
		Sets your own value of ERP
		@param erp new erp value
	*/
	void setERP(irr::f32 erp);

	/*!
		Sets your own value of ERP2
		@param erp2 new erp2 value
	*/
	void setERP2(irr::f32 erp2);

	/*!
		Sets your own value of CFM
		@param cfm new cfm value
	*/
	void setCFM(irr::f32 cfm);

	/*!
	   Sets your own time step. Use this function only if you know what you are doing.
	   Using a dynamic timestep can be useless, and your program can have an undefined behavior

	   @param step new time step
	*/
	void setTimeStep(irr::f32 step) {timestep = step;}

private:
	inline void updateObjects();

	btSoftBodyRigidBodyCollisionConfiguration* CollisionConfiguration;
	//btDiscreteDynamicsWorld* World;	
	btSoftRigidDynamicsWorld * World;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* pairCache;
    btConstraintSolver*	constraintSolver;
	
	array<CIrrBPCollisionObject *> collisionObj;
	//array<CIrrBPRigidBody*> rigidBodiesObj;
	//array<btRigidBody *> rigidBodies;
	//array<CIrrBPSoftBody*> softBodiesObj;
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

	irr::f32 timestep;

};

#endif