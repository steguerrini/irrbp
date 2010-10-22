#include "animator/CIrrBPTimeCallBackAnimator.h"

CIrrBPTimeCallbackAnimator::CIrrBPTimeCallbackAnimator(ITimer* timer, irr::s32 ms, void (*Func)())
{
		isEnded = false;
	rBody = NULL;
	irrTimer = timer;
	timeMs = ms;
	cbkFunc=Func;
}
void CIrrBPTimeCallbackAnimator::setBody(CIrrBPCollisionObject *body)
{
	activationTime = irrTimer->getRealTime();
	
	endTime = timeMs+activationTime;
	rBody = body;
}
void CIrrBPTimeCallbackAnimator::animate()
{
	
	if(rBody == NULL)
		return;

	if(endTime <= irrTimer->getRealTime())
	{
		isEnded = true;
		cbkFunc();
	}
}