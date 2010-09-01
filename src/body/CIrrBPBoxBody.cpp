#include "body/CIrrBPBoxBody.h"

CIrrBPBoxBody::CIrrBPBoxBody(irr::scene::ISceneNode *node, irr::f32 mass, irr::s32 bodyId)
{
   m_IrrSceneNode = node;
   m_BodyId = bodyId;

   vector3df Extent;
   Extent = node->getBoundingBox().getExtent();

   vector3df TScale = node->getScale();
   	//Calculate the half edges
   vector3df HalfEdges(Extent.X/2.0f,Extent.Y/2.0f,Extent.Z/2.0f);

   m_MotionState = new CMotionState(this,getTransformFromIrrlichtNode(node));

   btVector3 HalfExtents(TScale.X * HalfEdges.X, TScale.Y * HalfEdges.Y, TScale.Z * HalfEdges.Z);
   
   m_Shape = new btBoxShape(HalfExtents);
   
   btVector3 LocalInertia;
   m_Shape->calculateLocalInertia(mass, LocalInertia);

   m_RigidBody = new btRigidBody(mass, m_MotionState, m_Shape, LocalInertia);

   m_RigidBody->setUserPointer((void *)(node));
  // m_RigidBody->setActivationState(DISABLE_DEACTIVATION);
     
}
CIrrBPBoxBody::~CIrrBPBoxBody()
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
