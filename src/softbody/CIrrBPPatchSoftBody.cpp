#include <SoftBody/CIrrBPPatchSoftBody.h>
#include "CIrrBpWorld.h"

CIrrBPPatchSoftBody::~CIrrBPPatchSoftBody()
{
	mesh->clear();
	if(mesh)
		mesh->drop();
	delete m_softBody;
}
CIrrBPPatchSoftBody::CIrrBPPatchSoftBody(const vector3df & corner00 ,const vector3df & corner01,const vector3df & corner10 ,const vector3df & corner11,irr::f32 mass,CIrrBPWorld * world,int resx,int resy)
{
	mesh =NULL;
	m_corner00 = corner00;
	m_corner01 = corner01;
	m_corner10 = corner10;
	m_corner11 = corner11;
	this->resx = resx;
	this->resy = resy;
	m_softBody = btSoftBodyHelpers::CreatePatch(world->getSoftBodyWorldInfo(),irrVectorToBulletVector(corner00),irrVectorToBulletVector(corner01),irrVectorToBulletVector(corner10),irrVectorToBulletVector(corner11), resx,resy,0,true);
	m_softBody->setTotalMass(mass);
	m_softBody->randomizeConstraints();
	collisionObj = m_softBody;

}

void CIrrBPPatchSoftBody::setMass(irr::u32 x,irr::u32 y,irr::f32 mass)
{
	#define IDX(_x_,_y_)	((_y_)*resx+(_x_))
	CIrrBPSoftBody::setMass(IDX(x,y),mass);
}

IMesh * CIrrBPPatchSoftBody::getMesh()
{
	if(!mesh)
		createMesh();
	return mesh;
}

void CIrrBPPatchSoftBody::createMesh()
{
	SMeshBuffer * buffer = new SMeshBuffer();
	mesh = new SMesh();
	btSoftBody::tNodeArray nodes = m_softBody->m_nodes;
	btSoftBody::tFaceArray faces = m_softBody->m_faces;
	
	//Indices
	irr::core::array<u16> indices;
	
	
	std::map<btSoftBody::Node*,int>::iterator it;

	
	for(u32 i=0;i<faces.size();i++)
	{
		for(u32 k=0;k<3;k++)
		{
			it = index.find(faces[i].m_n[k]);		
			if(it == index.end())
			{
				indices.push_back(index.size());
				indexsw.insert(std::make_pair((int)index.size(),faces[i].m_n[k]));
				index.insert(std::make_pair(faces[i].m_n[k],(int)index.size()));
				
			}
			else
			{
				indices.push_back((*it).second);
			}
			
		}			
	}


	buffer->Indices = indices;

	for(int i=0; i<nodes.size();i++)
	{

		video::S3DVertex mtri;
		vector3df trianglePos = bulletVectorToIrrVector(indexsw.find(i)->second->m_x);
		mtri.Pos = trianglePos;
		mtri.Color = SColor(255,255,255,255);
		
		buffer->Vertices.push_back(mtri);
	}

	SMaterial material;
	material.BackfaceCulling = false;

	buffer->Material = material;
	
	aabbox3df bbox;
	btVector3 MinEdge,MaxEdge;
	m_softBody->getAabb(MinEdge,MaxEdge);
	bbox.MinEdge = bulletVectorToIrrVector(MinEdge);
	bbox.MaxEdge = bulletVectorToIrrVector(MaxEdge);
	
	mesh->addMeshBuffer(buffer);
	mesh->setBoundingBox(bbox);
	buffer->drop();

	meshbuff = mesh->getMeshBuffer(0);
}

void CIrrBPPatchSoftBody::update()
{
	if(!mesh)
		return;
	video::S3DVertex* mb_vertices;
	mb_vertices = (video::S3DVertex*)meshbuff->getVertices();

	//Update Vertices..
	for(u32 i=0;i<meshbuff->getVertexCount();i++)
		mb_vertices[i].Pos = bulletVectorToIrrVector(indexsw[i]->m_x);
	
	//Recalculate AABB
	btVector3 MinEdge,MaxEdge;
	m_softBody->getAabb(MinEdge,MaxEdge);
	mesh->setBoundingBox(aabbox3df(bulletVectorToIrrVector(MinEdge),bulletVectorToIrrVector(MaxEdge)));
	
}