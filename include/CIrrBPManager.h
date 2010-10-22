#ifndef CIrrBP_MGR_H_
#define CIrrBP_MGR_H_
#include "CIrrBPWorld.h"

#include "CIrrBPCollisionObject.h"
#include "body/CIrrBPRigidBody.h"
#include "body/CIrrBPTrimeshBody.h"
#include "body/CIrrBPBoxBody.h"
#include "body/CIrrBPCylinderBody.h"
#include "body/CIrrBPSphereBody.h"
#include "body/CIrrBPConeBody.h"
#include "body/CIrrBPCapsuleBody.h"

#include "constraint/CIrrBPConstraint.h"
#include "constraint/CIrrBPHingeConstraint.h"
#include "constraint/CIrrBPP2PConstraint.h"
#include "constraint/CIrrBPSlideConstraint.h"
#include "constraint/CIrrBPConeTwistConstraint.h"

#include "animator/CIrrBPAnimator.h"
#include "animator/CIrrBPCollisionDeleteAnimator.h"
#include "animator/CIrrBPDeleteAnimator.h"
#include "animator/CIrrBPCollisionCallbackAnimator.h"
#include "animator/CIrrBPTimeCallbackAnimator.h"

#include "softbody/CIrrBPSoftBody.h"
#include "softbody/CIrrBPRopeSoftBody.h"
#include "softbody/CIrrBPPatchSoftBody.h"

#include "irrlicht.h"
#include "types.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace io;
using namespace gui;
using namespace video;


class CIrrBPManager
{
public:
	 /*!
        Constructor.
        @param device A pointer to a Irrlicht's device
    */

	CIrrBPManager(IrrlichtDevice * device);
	~CIrrBPManager();

	/*!
	   Gets the world pointer.
	   @return Pointer to the world object
	*/
	CIrrBPWorld * getWorld() {return m_bulletWorld;}

	/*!
	  Adds a rigid box into the world.
	  @param node scene node to which engage the body
	  @param mass body's mass
	  @param bodyId a irrlicht-style body id
	  @return Pointer to the object.
	*/
	CIrrBPBoxBody * addRigidBox(ISceneNode * node, irr::f32 mass, irr::s32 bodyId = -1);

	/*!
	  Adds a rigid sphere into the world
	  @param node scene node to which engage the body
	  @param mass body's mass
	  @param bodyId a irrlicht-style body id
	  @return Pointer to the object.
	*/
	CIrrBPSphereBody * addRigidSphere(ISceneNode * node, irr::f32 mass, irr::s32 bodyId = -1);

	/*!
	  Adds a rigid cylinder into the world
	  @param node scene node to which engage the body
	  @param mass body's mass
	  @param bodyId a irrlicht-style body id
	  @param bodyOrientation the object' axis orientation. For example [====] has an X orientation.
	  @return Pointer to the object.
	*/
	CIrrBPCylinderBody * addRigidCylinder(ISceneNode * node, irr::f32 mass, irr::s32 bodyId = -1,BODY_OR bodyOrientationAxis = AUTO);

	/*!
	  Adds a rigid cone into the world
	  @param node scene node to which engage the body
	  @param mass body's mass
	  @param bodyId a irrlicht-style body id
	  @param bodyOrientation the object' axis orientation. /\ has an Y orientation.
	  @return Pointer to the object.
	*/
	CIrrBPConeBody * addRigidCone(ISceneNode * node, irr::f32 mass, irr::s32 bodyId = -1,BODY_OR bodyOrientationAxis=AUTO);

	/*!
	  Adds a rigid capsule into the world
	  @param node scene node to which engage the body
	  @param mass body's mass
	  @param bodyId a irrlicht-style body id
	  @param bodyOrientation the object' axis orientation. (====) has an X orientation.
	  @return Pointer to the object.
	*/
	CIrrBPCapsuleBody * addRigidCapsule (ISceneNode * node,irr::f32 mass, irr::s32 bodyId = -1,BODY_OR bodyOrientationAxis =AUTO);

	/*!
	  Adds a rigid trimesh into the world
	  @param node scene node to which engage the body
	  @param mass body's mass
	  @param bodyId a irrlicht-style body id
	  @return Pointer to the object.
	*/
	CIrrBPTrimesh * addTrimesh (IAnimatedMeshSceneNode * node,irr::f32 mass, irr::s32 bodyId = -1);
	
	/*!
	  Adds a rigid trimesh into the world
	  @param node scene node to which engage the body
	  @param mass body's mass
	  @param bodyId a irrlicht-style body id
	  @return Pointer to the object.
	*/
	CIrrBPTrimesh * addTrimesh (IMeshSceneNode * node,irr::f32 mass, irr::s32 bodyId = -1);


	/*!
	  Adds a rope (soft body) into the world.
	  @param from start of the rope
	  @param to end of the rope
	  @param mass rope's mass
	  @param res resolution of the rope. By increasing the resolution, you'll have got a more detailed rope but less performance. Leave -1 to auto-detect
	  @return Pointer to the object.
	*/
	CIrrBPRopeSoftBody * addRopeSoftBody(const vector3df & from ,const vector3df & to,irr::f32 mass,int res=-1);

	/*!
	  Adds a cloth\patch (soft body) into the world. You must specify the 4 cloth' corners.
	  @param corner00 first corner
	  @param corner01 second corner
	  @param corner10 third corner
	  @param corner11 fourth corner
	  @param mass patch's mass
	  @param resx lenght resolution. More resolution, minor performance.
	  @param resy depth resolution. More resolution, minor performance.
	  @return Pointer to the object.
	*/
	CIrrBPPatchSoftBody * addPatchSoftBody(const vector3df & corner00 ,const vector3df & corner01,const vector3df & corner10 ,const vector3df & corner11,irr::f32 mass,s32 resx,s32 resy);
	/*!
		Builds and attach a Slide Constraint to the bodies.
		The last 2 parameters will only works if there is a static object.<br>
		Please also note that due to internal conversions, if you place the moving body to a 0.0f position, the object will be moved toward a NEAR 0 value.<br>

		@param bodyA The first body
		@param bodyB The second body
		@param pivotInA The constraint position in A (0,0,0 by default)
		@param pivotInB The constraint position in B (0,0,0 by default)
		@param autoadapt If one body is static, and this flag is setted to false. The Slide will be only orthogonal
		@param rotatepiston If setted to true, the dynamic object (piston) will be rotated in the slide direction.
		@return pointer to the constraint
	*/
	CIrrBPSlideConstraint *	buildSlideConstraint(CIrrBPRigidBody * bodyA,CIrrBPRigidBody * bodyB,const vector3df & pivotInA=vector3df(0,0,0),const vector3df & pivotInB=vector3df(0,0,0),bool autoadapt=true, bool rotatepiston=true);

	/*!
		Builds and attach a point-to-point constraint to the body.

		@param bodyA The first body
		@param pivotInA The constraint position in A
		@return pointer to the constraint
	*/

	CIrrBPP2PConstraint * buildP2PConstraint(CIrrBPRigidBody * bodyA,const vector3df & pivotInA=vector3df(0,0,0));

	/*!
		Builds and attach a point-to-point constraint to the bodies.

		@param bodyA The first body
		@param bodyB The second body
		@param pivotInA The constraint position in A
		@param pivotInB The constraint position in B
		@return pointer to the constraint
	*/
	CIrrBPP2PConstraint * buildP2PConstraint (CIrrBPRigidBody * bodyA, CIrrBPRigidBody * bodyB, const vector3df & pivotInA=vector3df(0,0,0), const vector3df & pivotInB=vector3df(0,0,0));

	/*!
		Builds a cone twist constraint to the bodies. A cone-twist can be used to simulate ragdoll joints (shoulders, legs..)

		@param bodyA The first body
		@param bodyB The second body
		@param pivotInA The constraint position in A
		@param pivotInB The constraint position in B
		@return pointer to the constraint
	*/
	CIrrBPConeTwistConstraint * buildConeTwistConstraint(CIrrBPRigidBody * bodyA, CIrrBPRigidBody * bodyB, const vector3df & pivotInA=vector3df(0,0,0), const vector3df & pivotInB=vector3df(0,0,0));

	/*!
		Builds and attach a hinge constraint to the body.

		@param bodyA The first body
		@param pivotInA The constraint position in A
		@param axisInA The axis position in A
		@return pointer to the constraint
	*/
	CIrrBPHingeConstraint * buildHingeConstraint(CIrrBPRigidBody * bodyA,const vector3df & pivotInA,const vector3df & axisInA);
	
	/*!
		Builds and attach a hinge constraint to the body.

		@param bodyA The first body
		@param bodyB The second body
		@param pivotInA The constraint position in A
		@param pivotInB The constraint position in B
		@param axisInA The axis position in A
		@param axisInB The axis position in B
		@return pointer to the constraint
	*/
	CIrrBPHingeConstraint * buildHingeConstraint(CIrrBPRigidBody * bodyA,CIrrBPRigidBody * bodyB,const vector3df & pivotInA,const vector3df & pivotInB,const vector3df & axisInA,const vector3df & axisInB);


	/*! 
		Creates a delete animator to attach to a body.

		@param timeMs Time after which the body will be deleted
		@return pointer to the animator
	*/
	CIrrBPDeleteAnimator * createDeleteAnimator(irr::u32 timeMs);


	/*!
		Creates an on-event collision delete animator

		@param delFlag The flag that will set the deletion condition
		@return pointer to the animator
	*/
	CIrrBPCollisionDeleteAnimator * createCollisionDeleteAnimator(CIB_DFLAG delFlag);

	/*!
		Creates an on-collsion event callback animator. It will call Func when the body collides

		@param Func Pointer to the user call back function. The vector will contain the collision point.
		@return pointer to the animator
	*/
	CIrrBPCollisionCallbackAnimator * createCollisionCallbackAnimator(CIB_DFLAG cFlag, void (*Func)(const irr::core::vector3df &));

	/*!
		Creates an on-time event callback animator. It will call Func after timeMs is passed.

		@param timeMs Time after which the function will be call
		@param Func Pointer to the user call back function. 
		@return pointer to the animator
	*/
	
	CIrrBPTimeCallbackAnimator * createTimeCallbackAnimator(irr::s32 timeMs,void(*Func)());

	/*!
		Adds your own collision object to the bullet queue.
		You need to call this if you're not using the IrrBP manager to create a collision object
	*/
	void addCollisionObjectToBulletQueue(CIrrBPCollisionObject * cobj);

	/*!
		Removes a body from the bullet queue.
		Call this function instead of using body's internal drop function
	*/
	void removeBody(CIrrBPRigidBody * body);


	/*!
		Sets the gravity in the world.
		@param Gravity vector containing direction
	*/
	void setWorldGravity(const vector3df & Gravity) { m_bulletWorld->setGravity(Gravity);}

	/*!
		Drops the bullet manager
	*/
	void drop() { delete this;}

	/*!
		Gets a Body from a id.
		@param id The id to search for
		@return Pointer to the first body with this id. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPCollisionObject * getBodyFromId(irr::s32 id){ m_bulletWorld->getBodyFromId(id);}

	/*!
		Gets a Body from a unique id.
		@param id The unique id to search for
		@return Pointer to the first body with this id. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPCollisionObject * getBodyFromUId(irr::u32 uid){ m_bulletWorld->getBodyFromUId(uid);}


	/*!
		Gets a Body from a name.
		@param name The name to search for
		@return Pointer to the first body with this name. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPCollisionObject * getBodyFromName(irr::c8* name) { m_bulletWorld->getBodyFromName(name);}


	/*!
		Cleans the bullet world: removes all bodies and joints
	*/
	void clear();

	/*!
		Steps the simulation.
		It must be called each frame loop to step the bullet' simulation.
	*/
	REALINLINE void stepSimulation()
	{
		m_bulletWorld->stepSimulation();
	}
private:
	IrrlichtDevice * m_irrDevice;
	CIrrBPWorld * m_bulletWorld;
	array<CIrrBPAnimator *> m_bodyAnimators;
};
#endif