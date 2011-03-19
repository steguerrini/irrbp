#ifndef CIrrBP_FPSCAM_H_
#define CIrrBP_FPSCAM_H_
#include <irrlicht.h>
#include "body/CIrrBPRigidBody.h"
#include "animator/CIrrBPFollowAnimator.h"

class CIrrBPCamera
{
public:
	/*!
	  Main constructor for a Physics Camera.
	  To use your own body to approximate the camera, you can use the 2nd constructor
	  @param cam Camera scene node to attach to.
	  @param size "Block" size of the camera. The camera "body" is approximate to a cube with this size
	*/
	CIrrBPCamera(irr::scene::ICameraSceneNode * cam, int size = 15);

	/*!
	  Constructor for a Physics Camera.
	  This constructor can help you to create a camera with your own approximation, using your own body type (for example a capsule to simulate a body)
	  @param cam Camera scene node to attach to.
	  @param relativeBody body to attach.
	*/
	CIrrBPCamera(irr::scene::ICameraSceneNode * cam, CIrrBPRigidBody * relativeBody);

	void drop();
	~CIrrBPCamera(){ delete anim;}
	CIrrBPRigidBody * getCameraBody() { return body;}

private:
	CIrrBPFollowAnimator * anim;
	CIrrBPRigidBody * body;
	irr::scene::ICameraSceneNode * camera;

};
#endif