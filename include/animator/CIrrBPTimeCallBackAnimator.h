#ifndef CIrrBP_TCBL_ANIM_H_
#define CIrrBP_TCBL_ANIM_H_
#include "animator/CIrrBPAnimator.h"

class CIrrBPCollisionObject;
class CIrrBPWorld;

//!Please note that collision callback against soft body is not yet implemented in Bullet Physics.<br> So The Collision Delete Animator DOESN'T works with soft bodies.
class CIrrBPTimeCallbackAnimator  : public CIrrBPAnimator
{
public:
	CIrrBPTimeCallbackAnimator(ITimer* timer, irr::s32 ms, void (*Func)());
	void setBody(CIrrBPCollisionObject* body) ;
	void animate();
	
	void drop() {delete this;}
private:
	ITimer* irrTimer;
	irr::u32 activationTime;
	irr::u32 endTime;
	irr::u32 timeMs;

	CIrrBPWorld * rWorld;

	void (*cbkFunc)();
};
#endif