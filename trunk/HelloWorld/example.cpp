#include <irrlicht.h>
#include <iostream>
#include <IrrBP.h>

using namespace irr;
using namespace core;
using namespace video;
using namespace scene;
#include <vld.h>


static CIrrBPManager * bulletmgr;
static ISceneManager* smgr;
class Receiver : public IEventReceiver
{
public:
		Receiver();
		virtual bool OnEvent(const SEvent& event);
private:
};
Receiver::Receiver()
{

}

bool Receiver::OnEvent(const irr::SEvent &event)
{

	if(event.EventType == EET_MOUSE_INPUT_EVENT)
	{
		if(event.MouseInput.Event == EMIE_LMOUSE_PRESSED_DOWN)
		{
			ISceneNode * node = smgr->addCubeSceneNode(10,0,-1,smgr->getActiveCamera()->getPosition());
			CIrrBPBoxBody * body = bulletmgr->addRigidBox(node,40);
			irr::core::vector3df rot = smgr->getActiveCamera()->getRotation();
			irr::core::matrix4 mat;
	
			mat.setRotationDegrees(rot);
			irr::core::vector3df forwardDir(irr::core::vector3df(mat[8],mat[9],mat[10]) *120);

			body->getBodyPtr()->setLinearVelocity(irrVectorToBulletVector(forwardDir) * 2);
	
		}
		
	}

				
	return false;
}
int main()
{

	

	IrrlichtDevice *device =
		createDevice(video::EDT_OPENGL, core::dimension2d<u32>(640, 480));
	Receiver * recv = new Receiver();
	if (device == 0)
		return 1; // could not create selected driver.
	
	video::IVideoDriver* driver = device->getVideoDriver();
	 smgr = device->getSceneManager();

	device->getFileSystem()->addZipFileArchive("map-20kdm2.pk3");

	device->setEventReceiver(recv);

	scene::IAnimatedMesh* mesh = smgr->getMesh("20kdm2.bsp");
	scene::IMeshSceneNode* node = 0;

	if (mesh)
		node = smgr->addOctreeSceneNode(mesh->getMesh(0), 0, -1, 1024);

	if (node)
		node->setPosition(core::vector3df(-1350,-130,-1400));

	
	ICameraSceneNode * cam =  smgr->addCameraSceneNodeFPS(0,100,0.1f);
	cam->setPosition(vector3df(-20,60,-30));

	
	device->getCursorControl()->setVisible(false);

	bulletmgr = createBulletManager(device);
	bulletmgr->getWorld()->setGravity(vector3df(0,-10,0));
	bulletmgr->addTrimesh(node,0);

	int xshift,yshift,zshift;
	IMeshSceneNode * Node;
	IMeshSceneNode * Node2;
	Node = smgr->addCubeSceneNode(5,0,-1,vector3df(-20,30,0));

	Node->setMaterialType(EMT_TRANSPARENT_ADD_COLOR);
	Node->setMaterialFlag(EMF_LIGHTING,false);
	Node->setMaterialTexture(0,driver->getTexture("sphere1.jpg"));
	CIrrBPBoxBody * box= bulletmgr->addRigidBox(Node,0);
	
	Node2 = smgr->addCubeSceneNode(5,0,-1,vector3df(20,0,-20));
	Node2->setMaterialType(EMT_TRANSPARENT_ADD_COLOR);
	Node2->setMaterialFlag(EMF_LIGHTING,false);
	Node2->setMaterialTexture(0,driver->getTexture("sphere1.jpg"));
	CIrrBPBoxBody * box2 = bulletmgr->addRigidBox(Node2,40);
	
	CIrrBPConeTwistConstraint * constr = bulletmgr->buildConeTwistConstraint(box,box2,vector3df(0,-5,0),vector3df(0,5,0));
	constr->setLimit(PI/4*0.6,PI/4);

	int lastFPS = -1;

	while(device->run())
	{
		if (device->isWindowActive())
		{
			driver->beginScene(true, true, video::SColor(255,200,200,200));
			bulletmgr->stepSimulation();
			smgr->drawAll();
		
			int fps = driver->getFPS();

			if (lastFPS != fps)
			{
				
				core::stringw str = L"Irrlicht Engine - Quake 3 Map example [";
				str += driver->getName();
				str += "] FPS:";
				str += fps;

				device->setWindowCaption(str.c_str());
				lastFPS = fps;
			}
			driver->endScene();
		}
		else
			device->yield();
	}
	/*
	In the end, delete the Irrlicht device.
*/
	delete recv;
	bulletmgr->drop();
	device->drop();
	return 0;
}

/*
That's it. Compile and play around with the program.
**/
