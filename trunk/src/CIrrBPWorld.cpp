#include "CIrrBPWorld.h"


CIrrBPWorld::~CIrrBPWorld()
{
	isClosing=true;
	cout<<"# Cleaning IrrBP' pointers..."<<endl;
	m_worldInfo.m_sparsesdf.GarbageCollect();
	m_worldInfo.m_sparsesdf.Reset();

	/*Delete all objects*/
	for(u32 i=0;i<this->collisionObj.size();i++)
	{
		World->removeCollisionObject(collisionObj[i]->getPtr());
		collisionObj[i]->drop();
	}
	/*Delete all constraints*/
	for(u32 i=0;i<this->rigidBodiesConst.size();i++)
	{
		World->removeConstraint(rigidBodiesConst[i]->getConstraintPtr());
		rigidBodiesConst[i]->drop();
	}
	if(dDrawer)
		delete dDrawer;
	delete World;
	delete constraintSolver;
	delete pairCache;
	delete dispatcher;
	delete CollisionConfiguration;
	

	cout<<"# IrrBP closed successfully!"<<endl;

}
CIrrBPWorld::CIrrBPWorld(irr::IrrlichtDevice *device,const vector3df & Gravity)
{
	cout<<"# # # IrrBP - Version "<<IrrBP_MAJOR_VERSION<<"."<<IrrBP_MINOR_VERSION<<"."<<IrrBP_REVISION_VERSION<<" # # #"<<endl;
	cout<<"# Initializing new Irr-Bullet World..."<<endl;
	this->device = device;
	this->driver = device->getVideoDriver();

	this->Gravity = irrVectorToBulletVector(Gravity);
	
	CollisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	//CollisionConfiguration->setConvexConvexMultipointIterations();
	dispatcher = new btCollisionDispatcher(CollisionConfiguration);

	
	pairCache = new btDbvtBroadphase();

	constraintSolver = new btSequentialImpulseConstraintSolver();
	
    World = new btSoftRigidDynamicsWorld(dispatcher, pairCache,
        constraintSolver, CollisionConfiguration);

	btGImpactCollisionAlgorithm::registerAlgorithm(/*(btCollisionDispatcher*)*/dispatcher);
	
	irrTimer = device->getTimer();
	World->setGravity(this->Gravity);
	isClosing = false;

	dDrawer = NULL;

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

}
void CIrrBPWorld::clear()
{
	/*Delete all objects*/
	for(u32 i=0;i<this->collisionObj.size();i++)
	{
		World->removeCollisionObject(collisionObj[i]->getPtr());
		collisionObj[i]->drop();
	}
	/*Delete all constraints*/
	for(u32 i=0;i<this->rigidBodiesConst.size();i++)
	{
		World->removeConstraint(rigidBodiesConst[i]->getConstraintPtr());
		rigidBodiesConst[i]->drop();
	}
}
bool CIrrBPWorld::isBodyColliding(CIrrBPCollisionObject *body)
{
	const int numManifolds = World->getDispatcher()->getNumManifolds();
   
   int i;
   for (i=0;i<numManifolds;i++)
   {
      int id[2];
      id[0]=id[1]=-1;

      btPersistentManifold* contactManifold = World->getDispatcher()->getManifoldByIndexInternal(i);
      btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
      btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	  if(obA == body->getPtr() || obB == body->getPtr())
	  {
		  //cout<<"#Collided"<<endl;
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

void CIrrBPWorld::stepSimulation()
{
	DeltaTime = irrTimer->getTime() - TimeStamp;
    TimeStamp = irrTimer->getTime();
	World->stepSimulation(DeltaTime * 0.001f,1,timestep);
	
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