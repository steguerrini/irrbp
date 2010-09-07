#include "body/CIrrBPSphereBody.h"

CIrrBPSphereBody::CIrrBPSphereBody(irr::scene::ISceneNode *node, irr::f32 mass, irr::s32 bodyId)
{
	m_IrrSceneNode = node;
   m_BodyId = bodyId;

      
   vector3df Extent;
   Extent = node->getBoundingBox().getExtent();

	//Calculate the estimated radius.
   irr::f32 estimatedradius =  (Extent.Y * node->getScale().Y)/2.0f;

   m_MotionState = new CMotionState(this,getTransformFromIrrlichtNode(node));

   m_Shape = new btSphereShape(estimatedradius);
   
   btVector3 LocalInertia;
   m_Shape->calculateLocalInertia(mass, LocalInertia);

   m_RigidBody = new btRigidBody(mass, m_MotionState, m_Shape, LocalInertia);
   
   m_RigidBody->setUserPointer((void *)(node));

//   m_RigidBody->setActivationState(DISABLE_DEACTIVATION);
   collisionObj = m_RigidBody;
   
}

CIrrBPSphereBody::~CIrrBPSphereBody()
{
	if(m_IrrSceneNode != NULL)
		this->m_IrrSceneNode->remove();
	delete this->m_Shape;
	this->m_Shape = NULL;
	delete this->m_MotionState;
	this->m_MotionState = NULL;
	delete this->m_RigidBody;
	this->m_RigidBody = NULL;
}

