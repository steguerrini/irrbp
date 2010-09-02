#include "CIrrBPWorld.h"


CIrrBPWorld::~CIrrBPWorld()
{
	isClosing=true;
	cout<<"# Cleaning IrrBP' pointers..."<<endl;
	delete World;
	delete CollisionConfiguration;
	delete dispatcher;
	delete pairCache;
	delete constraintSolver;

	/*Delete all rigid bodies*/
	for(u32 i=0;i<this->rigidBodiesObj.size();i++)
		rigidBodiesObj[i]->drop();

	/*Delete all constraints*/
	for(u32 i=0;i<this->rigidBodiesConst.size();i++)
		rigidBodiesConst[i]->drop();

	if(dDrawer)
		delete dDrawer;
	
	cout<<"# IrrBP closed successfully!"<<endl;

}
CIrrBPWorld::CIrrBPWorld(irr::IrrlichtDevice *device,const vector3df & Gravity)
{
	cout<<"# # # IrrBP - Version "<<IrrBP_MAJOR_VERSION<<"."<<IrrBP_MINOR_VERSION<<"."<<IrrBP_REVISION_VERSION<<" # # #"<<endl;
	cout<<"# Initializing new Irr-Bullet World..."<<endl;
	this->device = device;
	this->driver = device->getVideoDriver();

	this->Gravity = irrVectorToBulletVector(Gravity);
	
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	
	//CollisionConfiguration->setConvexConvexMultipointIterations();
	dispatcher = new btCollisionDispatcher(CollisionConfiguration);

	
	pairCache = new btDbvtBroadphase();

	constraintSolver = new btSequentialImpulseConstraintSolver();
	
    World = new btSoftRigidDynamicsWorld(dispatcher, pairCache,
        constraintSolver, CollisionConfiguration);
	
	btGImpactCollisionAlgorithm::registerAlgorithm(/*(btCollisionDispatcher*)*/dispatcher);

	World->getSolverInfo().m_erp = 0.9;
	World->getSolverInfo().m_erp2 = 0.9f;
	World->getSolverInfo().m_globalCfm = 0.9f;
	
	irrTimer = device->getTimer();
	World->setGravity(this->Gravity);
	isClosing = false;

	dDrawer = NULL;
}
bool CIrrBPWorld::isBodyColliding(CIrrBPRigidBody *body, irr::s32 collMask)
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
	  if(obA == body->getBodyPtr() || obB == body->getBodyPtr())
	  {
		  //cout<<"#Collided"<<endl;

		 if(collMask == -1)
					  return true;

		  btCollisionObject * obC;
		  if(obA==body->getBodyPtr())
			obC=obB;
		  else
			 obC=obA;
		  
		  for(u32 i=0;i<rigidBodiesObj.size();i++)
		  {
			  if(rigidBodiesObj[i]->getBodyPtr() == obC)
			  {
				//  cout<<"##Collided With: "<<rigidBodiesObj[i]->getName()<<endl;
				  if(rigidBodiesObj[i]->getCollisionParam() == collMask)
					  return true;
				  else
					  return false;
			  }
		  }
	  }
	   
   } 
   return false;
}
irr::u32 CIrrBPWorld::addRigidBody(CIrrBPRigidBody *body)
{
	
	rigidBodies.push_back(body->getBodyPtr());
	World->addRigidBody(body->getBodyPtr());
	body->setValidStatus(true);
	rigidBodiesObj.push_back(body);
	
	#ifdef IRRBP_DEBUG_TEXT
	cout<<"# Added new Body "<<endl<<"## Body ID: "<<body->getID()<<endl<<"## Absolute Body ID: "<<body->getUniqueID()<<endl;
	#endif
	return (rigidBodiesObj.size()-1); //UniqueID
	
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

void CIrrBPWorld::removeRigidBody(CIrrBPRigidBody *body)
{
	btRigidBody * tofind = body->getBodyPtr();
	bool finded = false;
	for(irr::u32 i = 0; i<rigidBodies.size();i++)
	{
		if(tofind == rigidBodies[i])
		{
			#ifdef IRRBP_DEBUG_TEXT
			cout<<"# Deleted Body"<<endl<<"## Body ID: "<<body->getID()<<endl;
			#endif

			body->setValidStatus(false);
			World->removeRigidBody(body->getBodyPtr());
			
			rigidBodies.erase(i);
			rigidBodiesObj[i]->drop();
			rigidBodiesObj.erase(i);
			finded=true;
			return;
		}
	}
	
	#ifdef IRRBP_DEBUG_TEXT
	if(!finded)
	cout<<"# Error while deleting body...Body not found!";
	#endif

	
	
}

void CIrrBPWorld::stepSimulation()
{
	DeltaTime = irrTimer->getTime() - TimeStamp;
    TimeStamp = irrTimer->getTime();
	World->stepSimulation(DeltaTime * 0.001f,1);
	
	updateObjects();
};

void CIrrBPWorld::updateObjects()
{
	array<CIrrBPAnimator *> anims;
	for(irr::u32 i=0;i<rigidBodies.size();)
	{
		if(rigidBodiesObj[i]->isValid() == false)
		{
			removeRigidBody(rigidBodiesObj[i]);
			continue;
		}

		anims = rigidBodiesObj[i]->getAnimators();
			
		for(irr::u32 k=0;k<anims.size();k++)
			anims[k]->animate();	
			
			i++;

	}
}

CIrrBPRigidBody * CIrrBPWorld::getRigidBodyFromUId(irr::u32 uid)
{
	for(irr::u32 i=0;i<this->rigidBodiesObj.size();i++)
		if(rigidBodiesObj[i]->getUniqueID() == uid)
			return rigidBodiesObj[i];
	
	return NULL;
}
CIrrBPRigidBody * CIrrBPWorld::getRigidBodyFromId(irr::s32 id)
{
	
	for(irr::u32 i=0;i<this->rigidBodiesObj.size();i++)
		if(rigidBodiesObj[i]->getID() == id)
			return rigidBodiesObj[i];
	
	return NULL;
}
CIrrBPRigidBody * CIrrBPWorld::getRigidBodyFromName(irr::c8* name)
{
	for(irr::u32 i=0;i<this->rigidBodiesObj.size();i++)
		if(strcmp(rigidBodiesObj[i]->getName(),name)==0)
			return rigidBodiesObj[1];

	return NULL;
}

void CIrrBPWorld::createDebugDrawer()
{
	dDrawer = new CIrrBPDebugDrawer(this->device->getVideoDriver());
	if(World)
		World->setDebugDrawer(dDrawer);

	mat.Lighting = false;
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