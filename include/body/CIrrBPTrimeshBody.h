#ifndef CIrrBP_TRM_BODY_H_
#define CIrrBP_TRM_BODY_H_
#include "body/CIrrBPRigidBody.h"

class CIrrBPTrimesh : public CIrrBPRigidBody 
{
public:
	virtual void drop() { delete this;}
	~CIrrBPTrimesh();
	CIrrBPTrimesh(IAnimatedMeshSceneNode * node,irr::f32 mass, irr::s32 bodyId = -1);
	CIrrBPTrimesh(IMeshSceneNode * node,irr::f32 mass, irr::s32 bodyId = -1);
private:
	btGImpactMeshShape * m_trimeshShape;
	btTriangleMesh* m_indexVertexArrays;

	void initializeMesh(IMesh * mesh,const vector3df & pos, const vector3df & scale, void * nodePtr,irr::f32 mass);
};
#endif