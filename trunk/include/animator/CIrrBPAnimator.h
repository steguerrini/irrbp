#ifndef CIrrBP_ANIMATOR_H_
#define CIrrBP_ANIMATOR_H_
#include <irrlicht.h>
#include "convert.h"
#include "types.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace io;
using namespace gui;
using namespace video;

class CIrrBPCollisionObject;

class CIrrBPAnimator
{
public:
	CIrrBPAnimator();
	virtual ~CIrrBPAnimator();
	virtual void animate() = 0;
	virtual void drop() = 0;
	virtual bool isEnd();
	
	virtual void setBody(CIrrBPCollisionObject* body) { rBody = body;}
protected:
	CIrrBPCollisionObject* rBody;
	bool isEnded;
};


#endif