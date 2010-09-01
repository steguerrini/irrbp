#ifndef CIrrBP_CONVERT_H_
#define CIrrBP_CONVERT_H_

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <irrlicht.h>
using namespace irr;
using namespace core;
using namespace video;
using namespace scene;
namespace bullet
{
	inline static btVector3 irrVectorToBulletVector(const vector3df & toConvert)
	{
		return btVector3(toConvert.X,toConvert.Y,toConvert.Z);
	}
	inline static vector3df bulletVectorToIrrVector(const btVector3 & toConvert)
	{
		return vector3df(toConvert.x(),toConvert.y(),toConvert.z());
	}
	inline static void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler)
	{
	   btScalar W = TQuat.getW();
	   btScalar X = TQuat.getX();
	   btScalar Y = TQuat.getY();
	   btScalar Z = TQuat.getZ();
	   float WSquared = W * W;
	   float XSquared = X * X;
	   float YSquared = Y * Y;
	   float ZSquared = Z * Z;
	   TEuler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
	   TEuler.setY(asinf(-2.0f * (X * Z - Y * W)));
	   TEuler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
	   TEuler *= RADTODEG;
	}

	static vector3df QuaternionToIrrEuler(const btQuaternion &TQuat)
	{
		btVector3 bulletEuler;
		QuaternionToEuler(TQuat,bulletEuler);
		return bulletVectorToIrrVector(bulletEuler);
	}

	inline static btQuaternion irrRotationToBulletQuaterion(const vector3df & rotation)
	{
		return btQuaternion(degToRad(rotation.Y),degToRad(rotation.X),degToRad(rotation.Z));
	}

	static btTransform getTransformFromIrrlichtNode(const ISceneNode * irrNode)
	{
		btTransform transf;
		transf.setIdentity();
		/*If this node is the 1st, we need the relative position due to possible offset.*/
		transf.setOrigin(irrVectorToBulletVector(irrNode->getPosition()));
		//transf.setOrigin(irrVectorToBulletVector((*irrNode->getSceneManager()->getRootSceneNode()->getChildren().begin()) != irrNode ?
		//	irrNode->getAbsolutePosition() : irrNode->getPosition()));
		transf.getBasis().setEulerZYX(irrNode->getRotation().X * DEGTORAD,irrNode->getRotation().Y* DEGTORAD,irrNode->getRotation().Z* DEGTORAD);
		return transf;
	}
	static btTriangleMesh* irrMeshToBulletTriangleMesh(IMesh* pMesh,const vector3df& scaling)
	{
	  btVector3 vertices[3];
	  u32 i,j,k,index,numVertices,numIndices;
	  u16* mb_indices;
	  btTriangleMesh *pTriMesh = new btTriangleMesh();
	  for (i=0; i<pMesh->getMeshBufferCount(); i++)
	  {
		IMeshBuffer* mb=pMesh->getMeshBuffer(i);
		if(mb->getVertexType()==EVT_STANDARD)
		{
		  S3DVertex* mb_vertices=(S3DVertex*)mb->getVertices();
		  mb_indices = mb->getIndices();
		  numVertices = mb->getVertexCount();
		  numIndices = mb->getIndexCount();
		  for(j=0;j<numIndices;j+=3)
		  {
			for (k=0;k<3;k++)
			{
			  index = mb_indices[j+k];
			  vertices[k] = btVector3(mb_vertices[index].Pos.X*scaling.X, mb_vertices[index].Pos.Y*scaling.Y, mb_vertices[index].Pos.Z*scaling.Z);
			}

			pTriMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
		  }
		}
		else if(mb->getVertexType()==EVT_2TCOORDS)
		{
		  S3DVertex2TCoords* mb_vertices=(S3DVertex2TCoords*)mb->getVertices();
		  mb_indices = mb->getIndices();
		  numVertices = mb->getVertexCount();
		  numIndices = mb->getIndexCount();
		  for(j=0;j<numIndices;j+=3)
		  {
			for (k=0;k<3;k++)
			{
			  index = mb_indices[j+k];
			  vertices[k] = btVector3(mb_vertices[index].Pos.X*scaling.X, mb_vertices[index].Pos.Y*scaling.Y, mb_vertices[index].Pos.Z*scaling.Z);
			}
			pTriMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
		  }
		}
	  }
	  return pTriMesh;
	}



};

#endif