#ifndef CIrrBP_CDL_ANIM_H_
#define CIrrBP_CDL_ANIM_H_
#include "animator/CIrrBPAnimator.h"

class CIrrBPRigidBody;
class CIrrBPWorld;
class CIrrBPCollisionDeleteAnimator  : public CIrrBPAnimator
{
public:
	CIrrBPCollisionDeleteAnimator(CIB_DFLAG deleteFlag, CIrrBPWorld * world);
	void setBody(CIrrBPRigidBody* body) ;
	void animate();
	void drop() {delete this;}
private:
	irr::s16 internalStatus;
	CIrrBPWorld * rWorld;
	CIB_DFLAG dFlag; //delete flag

};

#endif