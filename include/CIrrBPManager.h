#ifndef CIrrBP_MGR_H_
#define CIrrBP_MGR_H_
#include "CIrrBPWorld.h"

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


#include "animator/CIrrBPAnimator.h"
#include "animator/CIrrBPCollisionDeleteAnimator.h"
#include "animator/CIrrBPDeleteAnimator.h"

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
	CIrrBPCylinderBody * addRigidCylinder(ISceneNode * node, irr::f32 mass, irr::s32 bodyId = -1,BODY_OR bodyOrientation = AUTO);

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
	CIrrBPCapsuleBody * addRigidCapsule (ISceneNode * node,irr::f32 mass, irr::s32 bodyId = -1,BODY_OR bodyOrientation=AUTO);

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

	CIrrBPP2PConstraint * buildP2PConstraint(CIrrBPRigidBody * bodyA,const vector3df & pivotInA);

	/*!
		Builds and attach a point-to-point constraint to the bodies.

		@param bodyA The first body
		@param bodyB The second body
		@param pivotInA The constraint position in A
		@param pivotInB The constraint position in B
		@return pointer to the constraint
	*/
	CIrrBPP2PConstraint * buildP2PConstraint (CIrrBPRigidBody * bodyA, CIrrBPRigidBody * bodyB, const vector3df & pivotInA, const vector3df & pivotInB);

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
		Adds your own rigid body to the bullet queue.
		You need to call this if you're not using the IrrBP manager to create a body
	*/
	void addBodyToBulletQueue(CIrrBPRigidBody * body);

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
		Gets a rigid Body from a id.
		@param id The id to search for
		@return Pointer to the first rigid body with this id. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPRigidBody * getRigidBodyFromId(irr::s32 id){ m_bulletWorld->getRigidBodyFromId(id);}

	/*!
		Gets a rigid Body from a unique id.
		@param id The unique id to search for
		@return Pointer to the first rigid body with this id. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPRigidBody * getRigidBodyFromUId(irr::u32 uid){ m_bulletWorld->getRigidBodyFromUId(uid);}


	/*!
		Gets a rigid Body from a name.
		@param name The name to search for
		@return Pointer to the first rigid body with this name. Returns NULL if no bodies couldn't be found.
	*/
	CIrrBPRigidBody * getRigidBodyFromName(irr::c8* name) { m_bulletWorld->getRigidBodyFromName(name);}

	inline void stepSimulation()
	{
		m_bulletWorld->stepSimulation();
	}
private:
	IrrlichtDevice * m_irrDevice;
	CIrrBPWorld * m_bulletWorld;
	array<CIrrBPAnimator *> m_bodyAnimators;
};
#endif