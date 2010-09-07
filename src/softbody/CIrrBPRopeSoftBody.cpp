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
	driver = world->getIrrDevice()->getVideoDriver();
	draw = false;
	drawMat.Thickness = 3;
	drawMat.Lighting = false;
	m_softBody = btSoftBodyHelpers::CreateRope(world->getSoftBodyWorldInfo(),irrVectorToBulletVector(from),irrVectorToBulletVector(to),res == -1 ? getAutoFixedRes() : res,1);
	m_softBody->setTotalMass(mass);
	m_softBody->randomizeConstraints();
	collisionObj = m_softBody;
}

int CIrrBPRopeSoftBody::getAutoFixedRes()
{
	int distance = to.getDistanceFrom(from);
	return (distance/2);
}

void CIrrBPRopeSoftBody::update()
{
	if(!draw)
		return;
	driver->setTransform(ETS_WORLD,matrix4());
	driver->setMaterial(drawMat);
	for(int i=0;i<m_softBody->m_links.size();i++)
		driver->draw3DLine(bulletVectorToIrrVector(m_softBody->m_links[i].m_n[0]->m_x),bulletVectorToIrrVector(m_softBody->m_links[i].m_n[1]->m_x),SColor(0,0,0,0));
}