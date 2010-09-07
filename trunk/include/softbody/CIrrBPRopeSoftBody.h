#ifndef CIrrBP_ROPE_SBODY_H_
#define CIrrBP_ROPE_SBODY_H_

#include "SoftBody/CIrrBPSoftBody.h"
#include "Body/CIrrBPRigidBody.h"

class CIrrBPWorld;

class CIrrBPRopeSoftBody : public CIrrBPSoftBody
{
public:
	CIrrBPRopeSoftBody(const vector3df & from ,const vector3df & to,irr::f32 mass,CIrrBPWorld * world,int res=-1);
	~CIrrBPRopeSoftBody();
	void drop() { delete this;}
	/*Set this to true to draw the rope*/
	void setDraw(bool Draw) { draw=Draw;}
	void update();
protected:
	bool draw;
	IVideoDriver* driver;
	SMaterial drawMat;
	int getAutoFixedRes();
	vector3df from;
	vector3df to;
};

#endif