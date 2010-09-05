#include <SoftBody/CIrrBPPatchSoftBody.h>
#include "CIrrBpWorld.h"

CIrrBPPatchSoftBody::~CIrrBPPatchSoftBody()
{
	delete m_softBody;
}
CIrrBPPatchSoftBody::CIrrBPPatchSoftBody(const vector3df & corner00 ,const vector3df & corner01,const vector3df & corner10 ,const vector3df & corner11,irr::f32 mass,CIrrBPWorld * world,int resx,int resy)
{
	
	m_corner00 = corner00;
	m_corner01 = corner01;
	m_corner10 = corner10;
	m_corner11 = corner11;
	this->resx = resx;
	this->resy = resy;
	m_softBody = btSoftBodyHelpers::CreatePatch(world->getSoftBodyWorldInfo(),irrVectorToBulletVector(corner00),irrVectorToBulletVector(corner01),irrVectorToBulletVector(corner10),irrVectorToBulletVector(corner11), resx,resy,0,true);
	m_softBody->setTotalMass(mass);
	
	btSoftBody::Material*	pm=m_softBody->appendMaterial();
	m_softBody->randomizeConstraints();

}

void CIrrBPPatchSoftBody::setMass(irr::u32 x,irr::u32 y,irr::f32 mass)
{
	#define IDX(_x_,_y_)	((_y_)*resx+(_x_))
	CIrrBPSoftBody::setMass(IDX(x,y),mass);
}