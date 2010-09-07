#ifndef CIrrBP_PATCH_SBODY_H_
#define CIrrBP_PATCH_SBODY_H_

#include "SoftBody/CIrrBPSoftBody.h"
#include "Body/CIrrBPRigidBody.h"
#include <map>

class CIrrBPWorld;

class CIrrBPPatchSoftBody : public CIrrBPSoftBody
{
public:
	CIrrBPPatchSoftBody(const vector3df & corner00 ,const vector3df & corner01,const vector3df & corner10 ,const vector3df & corner11,irr::f32 mass,CIrrBPWorld * world,s32 resx,s32 resy);
	~CIrrBPPatchSoftBody();
	void setMass(irr::u32 x,irr::u32 y,irr::f32 mass);

	/*! Returns the mesh to attach to a node.*/
	IMesh * getMesh();
	void drop() { delete this;}
	REALINLINE void update();
protected:
	void createMesh();
	irr::s32 resx,resy;
	vector3df m_corner00;
	vector3df m_corner01;
	vector3df m_corner10;
	vector3df m_corner11;
	SMesh * mesh;
	IMeshBuffer *meshbuff;
	std::map<btSoftBody::Node*,int> index;
	std::map<int,btSoftBody::Node*> indexsw; //Swapped index
};

#endif