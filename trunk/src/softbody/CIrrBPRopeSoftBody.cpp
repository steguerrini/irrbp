#include <SoftBody/CIrrBPRopeSoftBody.h>
#include "CIrrBpWorld.h"

CIrrBPRopeSoftBody::~CIrrBPRopeSoftBody()
{
	delete m_softBody;
}
CIrrBPRopeSoftBody::CIrrBPRopeSoftBody(const vector3df & From ,const vector3df & To,irr::f32 mass,CIrrBPWorld * world,int res)
{
	from=From;
	to=To;
	m_softBody = btSoftBodyHelpers::CreateRope(world->getSoftBodyWorldInfo(),irrVectorToBulletVector(from),irrVectorToBulletVector(to),res == -1 ? getAutoFixedRes() : res,1);
	m_softBody->setTotalMass(mass);
	m_softBody->randomizeConstraints();
	m_softBody->setMass(res-1,0);
	
}

int CIrrBPRopeSoftBody::getAutoFixedRes()
{
	int distance = to.getDistanceFrom(from);
	return (distance/2);
}