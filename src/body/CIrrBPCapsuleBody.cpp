#include "body/CIrrBPCapsuleBody.h"

CIrrBPCapsuleBody::CIrrBPCapsuleBody(ISceneNode * node,irr::f32 mass, irr::s32 bodyId,BODY_OR bodyOrientationAxis)
{
   m_IrrSceneNode = node;
   m_BodyId = bodyId;
   vector3df Extent;
   Extent = node->getBoundingBox().getExtent();

   vector3df TScale = node->getScale();

   m_MotionState = new CMotionState(this,getTransformFromIrrlichtNode(node));

   irr::f32 radius,height;

   /*Searching Capsule orientation*/
   BODY_OR capOr = bodyOrientationAxis;
   if(capOr == AUTO)
   {
	   if(Extent.X == Extent.Y && Extent.X == Extent.Z) //not recognizable. so take default axis (irrlicht' X , bullet' Z)
		   capOr = X;
	   else if(Extent.Y ==  Extent.Z)
		   capOr = X;
	   else if(Extent.X == Extent.Y)
		   capOr = Z;
	   else if(Extent.X == Extent.Z)
		   capOr = Y;
	   else //Also not recognizable. take default axis
		   capOr = X;

   }
   switch(capOr)
   {
	   case X:
		   radius = Extent.Z/2.0f;
		   height = Extent.X-(2*radius);
		   m_Shape = new btCapsuleShapeX(radius*TScale.Z,height*TScale.X);
		   break;
	   case Y:
		   radius = Extent.Z/2.0f;
		   height = Extent.Y-(2*radius);
		   m_Shape = new btCapsuleShape(radius*TScale.Z,height*TScale.Y);
		   break;
	   case Z:
		   radius = Extent.Y/2.0f;
		   height = Extent.Z - (2*radius);
		   m_Shape = new btCapsuleShapeZ(radius*TScale.Y,height*TScale.Z); 
		   break;
   }
   
   btVector3 LocalInertia;
   m_Shape->calculateLocalInertia(mass, LocalInertia);
   m_RigidBody = new btRigidBody(mass, m_MotionState, m_Shape, LocalInertia);

   m_RigidBody->setUserPointer((void *)(node));
   collisionObj = m_RigidBody;

}
CIrrBPCapsuleBody::~CIrrBPCapsuleBody()
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