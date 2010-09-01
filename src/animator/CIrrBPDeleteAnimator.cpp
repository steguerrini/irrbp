#include "animator/CIrrBPDeleteAnimator.h"

#include "body/CIrrBPRigidBody.h"


CIrrBPDeleteAnimator::CIrrBPDeleteAnimator(ITimer* timer, irr::u32 end)
{
	isEnded = false;
	rBody = NULL;
	irrTimer = timer;
	timeMs = end;
}

void CIrrBPDeleteAnimator::setBody(CIrrBPRigidBody *body)
{
	activationTime = irrTimer->getRealTime();
	
	endTime = timeMs+activationTime;
	rBody = body;
}
void CIrrBPDeleteAnimator::animate()
{
	if(rBody == NULL)
		return;

	if(endTime <= irrTimer->getRealTime())
	{
		isEnded = true;
		rBody->setValidStatus(false);
	}
}