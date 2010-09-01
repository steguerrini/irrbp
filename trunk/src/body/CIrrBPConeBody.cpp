#include "body/CIrrBPConeBody.h"

CIrrBPConeBody::CIrrBPConeBody(ISceneNode * node,irr::f32 mass, irr::s32 bodyId,BODY_OR bodyOrientationAxis)
{
   m_IrrSceneNode = node;
   m_BodyId = bodyId;
   irr::core::vector3df edges[8];
   node->getBoundingBox().getEdges(edges);
   
   vector3df TScale = node->getScale();

   
   irr::f32 radius,height;
   radius = fabs(edges[4].X-edges[0].X)/2.0f;
   height = fabs(edges[1].Y-edges[0].Y);

   radius *= TScale.X;
   height *= TScale.Y;

   m_MotionState = new CMotionState(this,getTransformFromIrrlichtNode(node));

   m_Shape = new btConeShape(radius,height);
   
   btVector3 LocalInertia;
   m_Shape->calculateLocalInertia(mass, LocalInertia);

   m_RigidBody = new btRigidBody(mass, m_MotionState, m_Shape, LocalInertia);

   m_RigidBody->setUserPointer((void *)(node));

}
CIrrBPConeBody::~CIrrBPConeBody()
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