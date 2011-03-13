#include "CIrrBPCamera.h"

#include "body/CIrrBPBoxBody.h"

CIrrBPCamera::CIrrBPCamera(irr::scene::ICameraSceneNode * cam, int size)
{
	camera = cam;
	irr::scene::ISceneNode * camvnode = cam->getSceneManager()->addCubeSceneNode(size,0,-1,cam->getPosition());
	body = new CIrrBPBoxBody(camvnode,0);
	anim = new CIrrBPFollowAnimator(cam);
	anim->setBody(body);
	body->addAnimator(anim);
}
CIrrBPCamera::CIrrBPCamera(irr::scene::ICameraSceneNode * cam, CIrrBPRigidBody * relativeBody)
{
	camera = cam;
	body = relativeBody;
	anim = new CIrrBPFollowAnimator(cam);
	anim->setBody(body);
	body->addAnimator(anim);
}