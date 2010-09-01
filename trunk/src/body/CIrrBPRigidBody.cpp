#include "body/CIrrBPRigidBody.h"
#include "animator/CIrrBPAnimator.h"

CIrrBPRigidBody::CIrrBPRigidBody()
{
	cParam = -1;
	m_BodyStatus = false;
	m_uBodyId = UNIQUE_BODY_ID_GENERATOR++;
}

void CIrrBPRigidBody::applyCentralImpulse(const irr::core::vector3df &impulse)
{
	m_RigidBody->applyCentralImpulse(irrVectorToBulletVector(impulse));
}
void CIrrBPRigidBody::applyCentralForce(const vector3df &force)
{
	m_RigidBody->applyCentralForce(irrVectorToBulletVector(force));
}
void  CIrrBPRigidBody::applyForce (const vector3df &force, const vector3df &rel_pos)
{
	m_RigidBody->applyForce(irrVectorToBulletVector(force),irrVectorToBulletVector(rel_pos));
}
void  CIrrBPRigidBody::applyTorqueImpulse (const vector3df &torque)
{
	m_RigidBody->applyTorqueImpulse(irrVectorToBulletVector(torque));
}
void  CIrrBPRigidBody::applyImpulse (const vector3df &impulse, const vector3df &rel_pos) 
{
	m_RigidBody->applyImpulse(irrVectorToBulletVector(impulse),irrVectorToBulletVector(rel_pos));
}
void CIrrBPRigidBody::applyTorque (const vector3df &torque)
{
	m_RigidBody->applyTorque(irrVectorToBulletVector(torque));
}

void CIrrBPRigidBody::addAnimator(CIrrBPAnimator * anim)
{
	anim->setBody(this);
	m_Animators.push_back(anim);
}

