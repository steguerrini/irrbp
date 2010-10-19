#include "animator/CIrrBPCollisionCallBackAnimator.h"
#include "CIrrBPCollisionObject.h"
#include "CIrrBPWorld.h"

CIrrBPCollisionCallbackAnimator::CIrrBPCollisionCallbackAnimator(CIB_DFLAG collFlag,CIrrBPWorld * world, void (*Func)(const irr::core::vector3df &))
{
	rWorld = world;
	rBody= NULL;
	internalStatus = 0;
	cbkFunc = Func;
	cFlag = collFlag;
	coll=false;
}
void CIrrBPCollisionCallbackAnimator::setBody(CIrrBPCollisionObject *body)
{
	rBody = body;
}
void CIrrBPCollisionCallbackAnimator::animate()
{
	if(rBody == NULL || rWorld == NULL || cFlag <= 0)
		return;

	switch(cFlag)
	{
		case ON_COLLIDE:
			if(rWorld->getBodyCollidingPoint(rBody,&contact))
			{
				cbkFunc(contact.point);
				isEnded=true;
			
			}
		break;
		case ON_COLLISION_RELEASE:
			if(internalStatus == 0)
			{
				if(rWorld->getBodyCollidingPoint(rBody,&contact))
					internalStatus =1;
			}
			else if(internalStatus == 1)
			{
				if(contact.contact ? !rWorld->isBodyColliding(rBody) : !rWorld->getBodyCollidingPoint(rBody,&contact))
					internalStatus =2;
			}
			else if(internalStatus == 2)
			{
				cbkFunc(contact.point);
				isEnded = true;
			}

		break;

	}

}