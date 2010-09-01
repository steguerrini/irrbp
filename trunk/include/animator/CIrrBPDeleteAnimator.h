#ifndef CIrrBP_DEL_ANIM_H_
#define CIrrBP_DEL_ANIM_H_
#include "animator/CIrrBPAnimator.h"

class CIrrBPRigidBody;
class CIrrBPDeleteAnimator : public CIrrBPAnimator
{
public:
	CIrrBPDeleteAnimator(ITimer* timer, irr::u32 end);
	void setBody(CIrrBPRigidBody* body) ;
	void animate();
	void drop() {delete this;}
private:
	ITimer* irrTimer;
	irr::u32 activationTime;
	irr::u32 endTime;
	irr::u32 timeMs;
};

#endif