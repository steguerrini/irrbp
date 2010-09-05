#include "softbody/CIrrBPSoftBody.h"

void CIrrBPSoftBody::addForce(const vector3df& force)
{
	m_softBody->addForce(irrVectorToBulletVector(force));
}
void CIrrBPSoftBody::addForce(const vector3df& force,int node)
{
	m_softBody->addForce(irrVectorToBulletVector(force),node);
}
void CIrrBPSoftBody::addVelocity(const vector3df& velocity)
{
	m_softBody->addVelocity(irrVectorToBulletVector(velocity));
}
void CIrrBPSoftBody::setVelocity(const vector3df& velocity)
{
	m_softBody->setVelocity(irrVectorToBulletVector(velocity));
}
void CIrrBPSoftBody::addVelocity(const vector3df& velocity,int node)
{
	m_softBody->addVelocity(irrVectorToBulletVector(velocity),node);
}

void CIrrBPSoftBody::appendAnchor(CIrrBPRigidBody * body)
{
	int end = m_softBody->m_nodes.size()-1;
	appendAnchor(body,end);
}
void CIrrBPSoftBody::appendAnchor(CIrrBPRigidBody * body,int pos)
{
	m_softBody->appendAnchor(pos,body->getBodyPtr());
}
void CIrrBPSoftBody::setMass(irr::s32 node, irr::f32 mass)
{
	m_softBody->setMass(node,mass);
}