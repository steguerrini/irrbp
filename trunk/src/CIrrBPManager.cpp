#include "CIrrBPManager.h"

CIrrBPManager::CIrrBPManager(IrrlichtDevice * device)
{
	
	if(device)
		m_irrDevice = device;
	else
		assert(!m_irrDevice);

	m_bulletWorld = new CIrrBPWorld(device,vector3df(0,0,0));
}
CIrrBPManager::~CIrrBPManager()
{
	delete m_bulletWorld;
	for(irr::u32 i=0;i<m_bodyAnimators.size();i++)
		m_bodyAnimators[i]->drop();
}

CIrrBPBoxBody * CIrrBPManager::addRigidBox(irr::scene::ISceneNode *node, irr::f32 mass, irr::s32 bodyId)
{
	CIrrBPBoxBody * box = new CIrrBPBoxBody(node,mass,bodyId);
	m_bulletWorld->addRigidBody(box);
	return box;
}

CIrrBPSphereBody * CIrrBPManager::addRigidSphere(irr::scene::ISceneNode *node, irr::f32 mass, irr::s32 bodyId)
{
	CIrrBPSphereBody * sphere = new CIrrBPSphereBody(node,mass,bodyId);
	m_bulletWorld->addRigidBody(sphere);
	return sphere;
}

CIrrBPCylinderBody * CIrrBPManager::addRigidCylinder(irr::scene::ISceneNode *node, irr::f32 mass, irr::s32 bodyId,BODY_OR bodyOrientation)
{
	CIrrBPCylinderBody * cyl = new CIrrBPCylinderBody(node,mass,bodyId,bodyOrientation);
	m_bulletWorld->addRigidBody(cyl);
	return cyl;
}

CIrrBPConeBody * CIrrBPManager::addRigidCone(irr::scene::ISceneNode *node, irr::f32 mass, irr::s32 bodyId,BODY_OR bodyOrientation)
{
	CIrrBPConeBody * cone = new CIrrBPConeBody(node,mass,bodyId,bodyOrientation);
	m_bulletWorld->addRigidBody(cone);
	return cone;
}

CIrrBPCapsuleBody * CIrrBPManager::addRigidCapsule(irr::scene::ISceneNode *node, irr::f32 mass, irr::s32 bodyId,BODY_OR bodyOrientation)
{
	CIrrBPCapsuleBody * caps = new CIrrBPCapsuleBody(node,mass,bodyId,bodyOrientation);
	m_bulletWorld->addRigidBody(caps);
	return caps;
}

CIrrBPTrimesh * CIrrBPManager::addTrimesh(irr::scene::IMeshSceneNode *node, irr::f32 mass, irr::s32 bodyId)
{
	CIrrBPTrimesh * trimesh = new CIrrBPTrimesh(node,mass,bodyId);
	m_bulletWorld->addRigidBody(trimesh);
	return trimesh;
}
CIrrBPTrimesh * CIrrBPManager::addTrimesh(irr::scene::IAnimatedMeshSceneNode *node, irr::f32 mass, irr::s32 bodyId)
{
	CIrrBPTrimesh * trimesh = new CIrrBPTrimesh(node,mass,bodyId);
	m_bulletWorld->addRigidBody(trimesh);
	return trimesh;
}
void CIrrBPManager::addBodyToBulletQueue(CIrrBPRigidBody * body)
{
	m_bulletWorld->addRigidBody(body);
}
CIrrBPDeleteAnimator * CIrrBPManager::createDeleteAnimator(irr::u32 timeMs)
{
	CIrrBPDeleteAnimator * delAnim = new CIrrBPDeleteAnimator(this->m_irrDevice->getTimer(),timeMs);
	m_bodyAnimators.push_back(delAnim);
	return delAnim;
	//return NULL;
}

void CIrrBPManager::removeBody(CIrrBPRigidBody * body)
{
	m_bulletWorld->removeRigidBody(body);
}
CIrrBPCollisionDeleteAnimator * CIrrBPManager::createCollisionDeleteAnimator(CIB_DFLAG delFlag)
{
	CIrrBPCollisionDeleteAnimator * collAnim = new CIrrBPCollisionDeleteAnimator(delFlag,m_bulletWorld);
	m_bodyAnimators.push_back(collAnim);
	return collAnim;
}

CIrrBPSlideConstraint  * CIrrBPManager::buildSlideConstraint(CIrrBPRigidBody * bodyA,CIrrBPRigidBody * bodyB,const vector3df & pivotInA,const vector3df & pivotInB,bool autoadapt, bool rotatepiston)
{
	CIrrBPSlideConstraint * slide = new CIrrBPSlideConstraint(bodyA,bodyB,pivotInA,pivotInB,autoadapt,rotatepiston);
	m_bulletWorld->addRigidBodyConstraint(slide);
	return slide;
}
CIrrBPP2PConstraint * CIrrBPManager::buildP2PConstraint(CIrrBPRigidBody * bodyA,const vector3df & pivotInA)
{
	CIrrBPP2PConstraint * p2p = new CIrrBPP2PConstraint(bodyA,pivotInA);
	m_bulletWorld->addRigidBodyConstraint(p2p);
	return p2p;
}
CIrrBPP2PConstraint * CIrrBPManager::buildP2PConstraint (CIrrBPRigidBody * bodyA, CIrrBPRigidBody * bodyB, const vector3df & pivotInA, const vector3df & pivotInB)
{
	CIrrBPP2PConstraint * p2p = new CIrrBPP2PConstraint(bodyA,bodyB,pivotInA,pivotInB);
	m_bulletWorld->addRigidBodyConstraint(p2p);
	return p2p;
}
CIrrBPConeTwistConstraint* CIrrBPManager::buildConeTwistConstraint(CIrrBPRigidBody * bodyA, CIrrBPRigidBody * bodyB, const vector3df & pivotInA, const vector3df & pivotInB)
{
	CIrrBPConeTwistConstraint * ctw = new CIrrBPConeTwistConstraint(bodyA,bodyB,pivotInA,pivotInB);
	m_bulletWorld->addRigidBodyConstraint(ctw);
	return ctw;
}
CIrrBPHingeConstraint * CIrrBPManager::buildHingeConstraint(CIrrBPRigidBody * bodyA,const vector3df & pivotInA,const vector3df & axisInA)
{
	CIrrBPHingeConstraint * hinge = new CIrrBPHingeConstraint(bodyA,pivotInA,axisInA);
	m_bulletWorld->addRigidBodyConstraint(hinge);
	return hinge;
}
CIrrBPHingeConstraint * CIrrBPManager::buildHingeConstraint(CIrrBPRigidBody * bodyA,CIrrBPRigidBody * bodyB,const vector3df & pivotInA,const vector3df & pivotInB,const vector3df & axisInA,const vector3df & axisInB)
{
	CIrrBPHingeConstraint * hinge = new CIrrBPHingeConstraint(bodyA,bodyB,pivotInA,pivotInB,axisInA,axisInB);
	m_bulletWorld->addRigidBodyConstraint(hinge);
	return hinge;
}

CIrrBPRopeSoftBody * CIrrBPManager::addRopeSoftBody(const vector3df & from ,const vector3df & to,irr::f32 mass,int res)
{
	CIrrBPRopeSoftBody * rope = new CIrrBPRopeSoftBody(from,to,mass,m_bulletWorld,res);
	m_bulletWorld->addSoftBody(rope);
	return rope;
}

CIrrBPPatchSoftBody * CIrrBPManager::addPatchSoftBody(const vector3df & corner00 ,const vector3df & corner01,const vector3df & corner10 ,const vector3df & corner11,irr::f32 mass,s32 resx,s32 resy)
{
	CIrrBPPatchSoftBody * patch = new CIrrBPPatchSoftBody(corner00,corner01,corner10,corner11,mass,m_bulletWorld,resx,resy);
	m_bulletWorld->addSoftBody(patch);
	return patch;
}