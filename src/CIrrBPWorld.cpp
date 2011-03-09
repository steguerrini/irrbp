#include "CIrrBPWorld.h"
#ifdef _WIN32
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#else
#include "BulletMultiThreaded/PosixThreadSupport.h"
#endif

#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#include "BulletMultiThreaded/btParallelConstraintSolver.h"
#include "BulletMultiThreaded/SequentialThreadSupport.h"

#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#include "body/CIrrBPTrimeshBody.h"
/*Multithreaded irrBP utilites*/
/* - START - */

#ifndef _IRR_WINDOWS_
	#ifndef USE_PTHREADS
		#ifdef _IRR_POSIX_API_
		#define USE_PTHREADS
		#else
		#warning Cannot use irrBP with multithread support (only works with Win32 and Posix compliant systems).
		#endif	
	#endif
#endif

btThreadSupportInterface* createSolverThreadSupport(int maxNumThreads)
{


#ifdef _WIN32
	Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("solverThreads",SolverThreadFunc,SolverlsMemoryFunc,maxNumThreads);
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(threadConstructionInfo);
	threadSupport->startSPU();
#elif defined (USE_PTHREADS)
	PosixThreadSupport::ThreadConstructionInfo solverConstructionInfo("solver", SolverThreadFunc,
																	  SolverlsMemoryFunc, maxNumThreads);
	
	PosixThreadSupport* threadSupport = new PosixThreadSupport(solverConstructionInfo);
	
#else
	SequentialThreadSupport::SequentialThreadConstructionInfo tci("solverThreads",SolverThreadFunc,SolverlsMemoryFunc);
	SequentialThreadSupport* threadSupport = new SequentialThreadSupport(tci);
	threadSupport->startSPU();
#endif
	
	return threadSupport;
}

btAlignedObjectArray<void*> sLocalStorePointers;

void* createCollisionLocalStoreMemoryWithDelete()
{
void* localStore = createCollisionLocalStoreMemory();
sLocalStorePointers.push_back(localStore);
return localStore;
}
/* - END - */

struct collisionCallback : public btCollisionWorld::ContactResultCallback
{
	bool collided;
	//Position World On First Body
	btVector3 pos;
	//Position World On Second Body
	btVector3 pos2;
	btCollisionObject* obj;
	btCollisionObject* obj2;
   collisionCallback(btCollisionObject* colObj,btCollisionObject* colObj2 = NULL)
   {
		collided=false;
		obj = colObj;
		obj2 = colObj2;
   }
   virtual   btScalar   addSingleResult(btManifoldPoint& cp,   const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
   {
	 if(obj2 == NULL)
	 {
		if(colObj0 == obj)
		{
			pos = cp.getPositionWorldOnA();
			pos2 = cp.getPositionWorldOnB();
		}
		else
		{
			pos = cp.getPositionWorldOnB();
			pos2 = cp.getPositionWorldOnA();
		}
	 }
	 else
	 {
		 pos = cp.getPositionWorldOnA();
		 pos2 = cp.getPositionWorldOnB();
	 }
	 collided=true;
     return 0;
   }
};


CIrrBPWorld::~CIrrBPWorld()
{
	isClosing=true;
	cout<<"# Cleaning IrrBP' pointers..."<<endl;
	m_worldInfo.m_sparsesdf.GarbageCollect();
	m_worldInfo.m_sparsesdf.Reset();

	clear();

	if(dDrawer)
		delete dDrawer;
	delete World;
	
	
	delete constraintSolver;
	
	if (m_threadSupportSolver) //!Only multithread
	{
		delete m_threadSupportSolver;
	}
		
	delete pairCache;
	delete dispatcher;
	
	if (m_threadSupportCollision) //!Only multithread
	{
		delete m_threadSupportCollision;
	}
	delete CollisionConfiguration;
	
	for (int i=0;i<sLocalStorePointers.size();i++) //!Only multithread
	{
		delete sLocalStorePointers[i];
	}
	sLocalStorePointers.clear();

	cout<<"# IrrBP closed successfully!"<<endl;

}
CIrrBPWorld::CIrrBPWorld(irr::IrrlichtDevice *device,const vector3df & Gravity, bool multithread, int maxtasks)
{
	cout<<"# # # IrrBP - Version "<<IrrBP_MAJOR_VERSION<<"."<<IrrBP_MINOR_VERSION<<"."<<IrrBP_REVISION_VERSION<<" # # #"<<endl;
	cout<<"# Initializing new Irr-Bullet World..."<<endl;
	if(multithread)
		cout<<"# Using IrrBP Multithreaded (beta)"<<endl;

	//Sets the thread supporters to NULL
	m_threadSupportSolver = NULL;
	m_threadSupportCollision = NULL;

	this->device = device;
	this->driver = device->getVideoDriver();

	this->Gravity = irrVectorToBulletVector(Gravity);
	
	if(multithread)
	{
		#ifdef _WIN32
		m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemoryWithDelete,
								maxtasks));

		#elif defined (USE_PTHREADS)
			PosixThreadSupport::ThreadConstructionInfo constructionInfo("collision",
										processCollisionTask,
										createCollisionLocalStoreMemory,
										maxtasks);
			m_threadSupportCollision = new PosixThreadSupport(constructionInfo);
		#else

			SequentialThreadSupport::SequentialThreadConstructionInfo colCI("collision",processCollisionTask,createCollisionLocalStoreMemory);
			SequentialThreadSupport* m_threadSupportCollision = new SequentialThreadSupport(colCI);
				
		#endif 

		m_threadSupportSolver = createSolverThreadSupport(maxtasks);
	}
	CollisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	//CollisionConfiguration->setConvexConvexMultipointIterations();
	if(!multithread)
		dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	else
		dispatcher = new SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxtasks,CollisionConfiguration);

	
	pairCache = new btDbvtBroadphase();

	if(!multithread)
		constraintSolver = new btSequentialImpulseConstraintSolver();
	else
		constraintSolver = new btParallelConstraintSolver(m_threadSupportSolver);
	
    World = new btSoftRigidDynamicsWorld(dispatcher, pairCache,
        constraintSolver, CollisionConfiguration);

	btGImpactCollisionAlgorithm::registerAlgorithm(/*(btCollisionDispatcher*)*/dispatcher);
	
	irrTimer = device->getTimer();
	World->setGravity(this->Gravity);
	isClosing = false;

	dDrawer = NULL;

	/*Multithread data*/
	if(multithread)
	{
		World->getSimulationIslandManager()->setSplitIslands(false);
		World->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;
		World->getDispatchInfo().m_enableSPU = true;
	}	

	/*Set soft body informer*/
	
	m_worldInfo.m_broadphase = pairCache;
    m_worldInfo.m_dispatcher = dispatcher;
	m_worldInfo.m_sparsesdf.Initialize();

    m_worldInfo.m_gravity.setValue(0,-10.0,0);
    m_worldInfo.air_density = 1.0f;
    m_worldInfo.water_density = 0;
    m_worldInfo.water_offset = 0;
    m_worldInfo.water_normal = btVector3(0,0,0);

	timestep = 1.0f/100.0f;
	maxSubSteps = 5;
}
void CIrrBPWorld::clear()
{

	/*Delete all constraints*/
	for(u32 i=0;i<this->rigidBodiesConst.size();i++)
	{
		World->removeConstraint(rigidBodiesConst[i]->getConstraintPtr());
		rigidBodiesConst[i]->drop();
	}
	rigidBodiesConst.set_used(0);

	/*Delete all objects*/
	for(u32 i=0;i<this->collisionObj.size();i++)
	{
		World->removeCollisionObject(collisionObj[i]->getPtr());
		collisionObj[i]->drop();
	}
	collisionObj.set_used(0);

	/*Delete all action intefaces*/
	for(u32 i=0;i<this->actionObj.size();i++)
	{
		World->removeAction(actionObj[i]->getPtr());
		actionObj[i]->drop();
	}

	actionObj.set_used(0);
}
CIrrBPCollisionObject * CIrrBPWorld::getObjectByPointer(btCollisionObject* cObj)
{
	for(irr::u32 i=0;i<collisionObj.size();i++)
		if(collisionObj[i]->getPtr() == cObj)
			return collisionObj[i];
	return NULL;
}

//Thanks to mihail isakov for the code below
//Taken from game-ws repository
bool getTriangleFromCallBack(btCollisionShape * shape,
									 int hitTriangleIndex,
									 irr::core::triangle3df & triangle)
{
btStridingMeshInterface * meshInterface = NULL;

	if (shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
	{
		meshInterface = (static_cast<btGImpactMeshShape*>(shape))->getMeshInterface();
	}
	else if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
	{
		meshInterface = (static_cast<btBvhTriangleMeshShape*>(shape))->getMeshInterface();
	}
	else
	{
	return false;
	}

	if (!meshInterface) return false;

unsigned char *vertexbase;
int numverts;
PHY_ScalarType type;
int stride;
unsigned char *indexbase;
int indexstride;
int numfaces;
PHY_ScalarType indicestype;

meshInterface->getLockedVertexIndexBase(
	&vertexbase,
	numverts,
	type,
	stride,
	&indexbase,
	indexstride,
	numfaces,
	indicestype,
	0);

unsigned int * gfxbase = (unsigned int*)(indexbase+hitTriangleIndex*indexstride);
const btVector3 & meshScaling = shape->getLocalScaling();
btVector3 triangle_v[3];
	for (int j=2;j>=0;j--)
	{
		int graphicsindex = indicestype==PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];

		btScalar * graphicsbase = (btScalar*)(vertexbase+graphicsindex*stride);

//#define TEST_DEFORM
#ifdef TEST_DEFORM
		if (shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		{
			// model-specific approximations
			if ( graphicsindex >= 0 && graphicsindex <= 31)
			{
			graphicsbase[0] *= btScalar(0.98);
			graphicsbase[1] *= btScalar(0.98);
			graphicsbase[2] *= btScalar(0.98);
			}
			else if (graphicsindex >= 32 && graphicsindex <= 33)
			{
			graphicsbase[0] *= btScalar(0.91);
			graphicsbase[1] *= btScalar(0.91);
			graphicsbase[2] *= btScalar(0.91);
			}
			else
			{
			graphicsbase[0] *= btScalar(0.88);
			graphicsbase[1] *= btScalar(0.88);
			graphicsbase[2] *= btScalar(0.88);
			}
		}

#endif

	triangle_v[j] = btVector3(graphicsbase[0]*meshScaling.getX(),
							  graphicsbase[1]*meshScaling.getY(),
							  graphicsbase[2]*meshScaling.getZ());	

	}

meshInterface->unLockVertexBase(0);

#ifdef TEST_DEFORM
	if (shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
	{
		btGImpactMeshShape * gimp_shape = static_cast<btGImpactMeshShape*>(shape);
		gimp_shape->postUpdate();
	}
#endif

triangle.pointA = bulletVectorToIrrVector(triangle_v[0]);
triangle.pointB = bulletVectorToIrrVector(triangle_v[1]);
triangle.pointC = bulletVectorToIrrVector(triangle_v[2]);

return true;
}

struct	ClosestRayResultCallback : public btCollisionWorld::RayResultCallback
{
	ClosestRayResultCallback(const btVector3&	rayFromWorld,const btVector3&	rayToWorld)
	:m_rayFromWorld(rayFromWorld),
	m_rayToWorld(rayToWorld)
	{
	}

	btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
	btVector3	m_rayToWorld;

	btVector3	m_hitNormalWorld;
	btVector3	m_hitPointWorld;
	int m_hitTriangle;
	virtual	btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		m_hitTriangle = rayResult.m_localShapeInfo->m_triangleIndex;
		m_closestHitFraction = rayResult.m_hitFraction;
		m_collisionObject = rayResult.m_collisionObject;
		if (normalInWorldSpace)
		{
			m_hitNormalWorld = rayResult.m_hitNormalLocal;
		} else
		{
			///need to transform normal into worldspace
			m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
		}
		m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
		return rayResult.m_hitFraction;
	}
};

struct	AllHitsRayResultModCallback : public btCollisionWorld::RayResultCallback
{
	AllHitsRayResultModCallback(const btVector3&	rayFromWorld,const btVector3&	rayToWorld)
	:m_rayFromWorld(rayFromWorld),
	m_rayToWorld(rayToWorld)
	{
	}

	btAlignedObjectArray<btCollisionObject*>		m_collisionObjects;

	btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
	btVector3	m_rayToWorld;

	btAlignedObjectArray<btVector3>	m_hitNormalWorld;
	btAlignedObjectArray<btVector3>	m_hitPointWorld;
	btAlignedObjectArray<btScalar> m_hitFractions;
	btAlignedObjectArray<int> m_hitTriangles;
	virtual	btScalar	addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		m_collisionObject = rayResult.m_collisionObject;
		m_collisionObjects.push_back(rayResult.m_collisionObject);
		m_hitTriangles.push_back(rayResult.m_localShapeInfo->m_triangleIndex);
		btVector3 hitNormalWorld;
		if (normalInWorldSpace)
		{
			hitNormalWorld = rayResult.m_hitNormalLocal;
		} else
		{
			///need to transform normal into worldspace
			hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
		}
		m_hitNormalWorld.push_back(hitNormalWorld);
		btVector3 hitPointWorld;
		hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
		m_hitPointWorld.push_back(hitPointWorld);
		m_hitFractions.push_back(rayResult.m_hitFraction);
		return m_closestHitFraction;
	}
};


bool CIrrBPWorld::rayCastTest(vector3df from,vector3df to, irr::core::array<contactPoint> * points)
{
	AllHitsRayResultModCallback cb(irrVectorToBulletVector(from),irrVectorToBulletVector(to));
	
	World->rayTest(irrVectorToBulletVector(from),irrVectorToBulletVector(to),cb);
	bool hit = cb.hasHit();
	if(points && hit)
	{
		for(int i=0;i<cb.m_collisionObjects.size();i++)
		{
			contactPoint cp;
			cp.contact = true;
			cp.point = bulletVectorToIrrVector(cb.m_hitPointWorld[i]);
			cp.body = getObjectByPointer(cb.m_collisionObjects[i]);
			if(cp.body->getObjectType() == RIGID_BODY)
			{
				if(((CIrrBPRigidBody*)cp.body)->getRigidBodyType() == TRIMESH)
				{
					irr::core::triangle3df tr;
					getTriangleFromCallBack(cb.m_collisionObjects[i]->getCollisionShape(),cb.m_hitTriangles[i],tr);
					
					if(cp.triangle)
						delete cp.triangle;
					cp.triangle = new irr::core::triangle3df(tr);
				}
			}
			(*points).push_back(cp);
		}
	}
	return hit;

}


bool CIrrBPWorld::rayCastClosestHitTest(vector3df from,vector3df to, contactPoint * point)
{
	ClosestRayResultCallback cb(irrVectorToBulletVector(from),irrVectorToBulletVector(to));
	
	World->rayTest(irrVectorToBulletVector(from),irrVectorToBulletVector(to),cb);
	bool hit = cb.hasHit();
		
	if(point && hit)
	{
		point->contact = true;
		point->point = bulletVectorToIrrVector(cb.m_hitPointWorld);
		point->body = getObjectByPointer(cb.m_collisionObject);
		
		if(point->body->getObjectType() == RIGID_BODY)
		{
			if(((CIrrBPRigidBody*)point->body)->getRigidBodyType() == TRIMESH)
			{
				irr::core::triangle3df tr;
				getTriangleFromCallBack(cb.m_collisionObject->getCollisionShape(),cb.m_hitTriangle,tr);

				if(point->triangle)
					delete point->triangle;
				point->triangle = new irr::core::triangle3df(tr);

			}
		}
	}
	return hit;
}
bool CIrrBPWorld::isPairColliding(CIrrBPCollisionObject *body1,CIrrBPCollisionObject *body2, contactPoint * dCP, bool returnSecondPoint)
{
	if(!body1)
		assert(!body1);
	if(!body2)
		assert(!body2);
	collisionCallback cBack(body1->getPtr(),body2->getPtr());
	World->contactPairTest(body1->getPtr(),body2->getPtr(),cBack);	

	if(dCP)
	{
		if(!returnSecondPoint)
			dCP->point = bulletVectorToIrrVector(cBack.pos);
		else
			dCP->point = bulletVectorToIrrVector(cBack.pos2);
		dCP->contact = cBack.collided;
	}
	
	return cBack.collided;

}
bool CIrrBPWorld::getBodyCollidingPoint(CIrrBPCollisionObject *body, contactPoint * dCP)
{
	if(!body)
		assert(!body);
	if(!dCP)
		assert(!dCP);
	collisionCallback cBack(body->getPtr());
	World->contactTest(body->getPtr(),cBack);
	dCP->point = bulletVectorToIrrVector(cBack.pos);
	dCP->contact = cBack.collided;
	dCP->body = body;
	return cBack.collided;
}
bool CIrrBPWorld::isBodyColliding(CIrrBPCollisionObject *body)
{
	const int numManifolds = World->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  World->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		if(obA == body->getPtr() || obB == body->getPtr())
		{
			int numContacts = contactManifold->getNumContacts();

			if (numContacts == 0)
				return false;

			for (int j=0;j<numContacts;j++)
			{
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if (pt.getDistance()>0.f)
					return false;
			}
			return true;
		}
	}
	return false;
}
void CIrrBPWorld::addSoftBody(CIrrBPSoftBody * sbody)
{
	collisionObj.push_back(sbody);
	World->addSoftBody(sbody->getBodyPtr());
	sbody->setValidStatus(true);
	#ifdef IRRBP_DEBUG_TEXT
	cout<<"# Added new Soft Body"<<endl;
	#endif
	
}
void CIrrBPWorld::addRigidBody(CIrrBPRigidBody *body)
{
	
	collisionObj.push_back(body);
	World->addRigidBody(body->getBodyPtr());
	body->setValidStatus(true);
	
	#ifdef IRRBP_DEBUG_TEXT
	cout<<"# Added new Body "<<endl<<"## Body ID: "<<body->getID()<<endl<<"## Absolute Body ID: "<<body->getUniqueID()<<endl;
	#endif

}

void CIrrBPWorld::addCollisionObject(CIrrBPCollisionObject * cobj)
{
	switch(cobj->getObjectType())
	{
		case RIGID_BODY:
			this->addRigidBody(dynamic_cast<CIrrBPRigidBody*>(cobj));
		break;
		case SOFT_BODY:
			this->addSoftBody(dynamic_cast<CIrrBPSoftBody*>(cobj));
		break;
	}
}
void CIrrBPWorld::addRigidBodyConstraint(CIrrBPConstraint * constraint)
{
	rigidBodiesConst.push_back(constraint);
	World->addConstraint(constraint->getConstraintPtr());
	#ifdef IRRBP_DEBUG_TEXT
	cout<<"# Added new constraint"<<endl<<"## Body ID (A): "<<constraint->getBodyA()->getID()<<endl<<"## Body UID (A): "<<constraint->getBodyA()->getUniqueID()<<endl;
	
	if(constraint->getBodyB())
		cout<<"## Body ID (B): "<<constraint->getBodyB()->getID()<<endl<<"## Body UID (B): "<<constraint->getBodyB()->getUniqueID()<<endl;
	#endif
}

void CIrrBPWorld::addAction(CIrrBPActionInterface * action)
{
	actionObj.push_back(action);
	World->addAction(action->getPtr());
	#ifdef IRRBP_DEBUG_TEXT
	cout<<"# Added new action "<<endl<<"## Action ID: "<<action->getID()<<endl<<"## Absolute Action ID: "<<action->getUniqueID()<<endl;
	#endif

}

void CIrrBPWorld::removeAction(CIrrBPActionInterface * action)
{
	for(irr::u32 i=0;i<actionObj.size();i++)
	{
		if(action == actionObj[i])
		{
			action->setValidStatus(false);
			World->removeAction(action->getPtr());
			actionObj[i]->drop();
			actionObj.erase(i);
			#ifdef IRRBP_DEBUG_TEXT
			cout<<"# Deleted action "<<endl<<"## Action ID: "<<action->getID()<<endl<<"## Absolute Action ID: "<<action->getUniqueID()<<endl;
			#endif
			return;
		}
	}
}

void CIrrBPWorld::removeCollisionObject(CIrrBPCollisionObject * cobj)
{
	for(irr::u32 i = 0;i<collisionObj.size(); i++)
	{
		if(cobj == collisionObj[i])
		{
			cobj->setValidStatus(false);
			World->removeCollisionObject(cobj->getPtr());
			collisionObj[i]->drop();
			collisionObj.erase(i);
			#ifdef IRRBP_DEBUG_TEXT
			cout<<"# Deleted Body"<<endl<<"## Body ID: "<<cobj->getID()<<endl;
			#endif

			return;
		}
	}
	#ifdef IRRBP_DEBUG_TEXT
	cout<<"# Error while deleting body...Body not found!";
	#endif
}
void CIrrBPWorld::removeRigidBody(CIrrBPRigidBody *body)
{
	this->removeCollisionObject(body);
	
}
void CIrrBPWorld::autoMaxSubSteps(int minFPS)
{

	//Equation: timestamp < m.s.s. * f.t.s
	// 1/FPS <  m.s.s. * f.t.s
	// m.s.s. > (1/FPS)/f.t.s
	float eq = (1.0f/((float)minFPS))/timestep;
	maxSubSteps = irr::core::ceil32(eq);
}
void CIrrBPWorld::stepSimulation()
{
	
	DeltaTime = irrTimer->getRealTime();
	#ifdef IRRBP_DEBUG_TEXT
		if((DeltaTime-TimeStamp) / 1000.0f >= (10*timestep))
		cout<<"You must fix your physics parameters"<<endl;
	#endif

	World->stepSimulation((DeltaTime-TimeStamp) / 1000.0f,maxSubSteps,timestep);
	TimeStamp = irrTimer->getRealTime();

	m_worldInfo.m_sparsesdf.GarbageCollect();
	updateObjects();
	
	
};

void CIrrBPWorld::updateObjects()
{
	array<CIrrBPAnimator *> anims;
	for(irr::u32 i=0;i<collisionObj.size();)
	{
		if(collisionObj[i]->isValid() == false)
		{
			removeCollisionObject(collisionObj[i]);
			continue;
		}
		anims = collisionObj[i]->getAnimators();
		for(irr::u32 k=0;k<anims.size();k++)
			anims[k]->animate();

		
		if(collisionObj[i]->getObjectType() == SOFT_BODY)
			static_cast<CIrrBPSoftBody*>(collisionObj[i])->update();
		i++;
	}
}

CIrrBPCollisionObject * CIrrBPWorld::getBodyFromUId(irr::u32 uid)
{
	for(irr::u32 i=0;i<this->collisionObj.size();i++)
		if(collisionObj[i]->getUniqueID() == uid)
			return collisionObj[i];
	
	return NULL;
}
CIrrBPCollisionObject * CIrrBPWorld::getBodyFromId(irr::s32 id)
{
	
	for(irr::u32 i=0;i<this->collisionObj.size();i++)
		if(collisionObj[i]->getID() == id)
			return collisionObj[i];
	
	return NULL;
}
CIrrBPCollisionObject * CIrrBPWorld::getBodyFromName(irr::c8* name)
{
	for(irr::u32 i=0;i<this->collisionObj.size();i++)
		if(strcmp(collisionObj[i]->getName(),name)==0)
			return collisionObj[1];

	return NULL;
}

CIrrBPActionInterface * CIrrBPWorld::getActionFromId(irr::s32 id)
{
	for(irr::u32 i=0;i<this->actionObj.size();i++)
		if(actionObj[i]->getID() == id)
			return actionObj[i];
	
	return NULL;
}
CIrrBPActionInterface * CIrrBPWorld::getActionFromUId(irr::u32 uid)
{
	
	for(irr::u32 i=0;i<this->actionObj.size();i++)
		if(actionObj[i]->getUniqueID() == uid)
			return actionObj[i];
	
	return NULL;
}
CIrrBPActionInterface * CIrrBPWorld::getActionFromName(irr::c8* name)
{
	for(irr::u32 i=0;i<this->actionObj.size();i++)
		if(strcmp(actionObj[i]->getName(),name)==0)
			return actionObj[i];
	
	return NULL;
}

void CIrrBPWorld::createDebugDrawer()
{
	dDrawer = new CIrrBPDebugDrawer(this->device->getVideoDriver());
	if(World)
		World->setDebugDrawer(dDrawer);

	mat.Lighting = false;
	//mat.Thickness = 3;
}

void CIrrBPWorld::stepDebugDrawer()
{
	driver->setTransform(ETS_WORLD,matrix4());
	driver->setMaterial(mat);
	World->debugDrawWorld();
}

void CIrrBPWorld::setDebugDrawerFlags(int flags)
{
	if(dDrawer)
		dDrawer->setDebugMode(flags);
}

btSoftBodyWorldInfo& CIrrBPWorld::getSoftBodyWorldInfo() 
{
	return m_worldInfo;
}

void CIrrBPWorld::setERP(irr::f32 erp)
{
	World->getSolverInfo().m_erp = erp;
}
void CIrrBPWorld::setERP2(irr::f32 erp2)
{
	World->getSolverInfo().m_erp2 = erp2;
}

void CIrrBPWorld::setCFM(irr::f32 cfm)
{
	World->getSolverInfo().m_globalCfm = cfm;
}