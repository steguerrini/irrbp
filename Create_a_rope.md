# Introduction #

This tutorial will guide you to create a rope and attach it to a body;

# Coding... #

First you need to include the irrBP general header..and setting some irrlicht variables...

```
#include <IrrBP.h>
int main()
{

	

	IrrlichtDevice *device =
		createDevice(video::EDT_OPENGL, core::dimension2d<u32>(640, 480));
	if (device == 0)
		return 1; // could not create selected driver.
	
	video::IVideoDriver* driver = device->getVideoDriver();
	smgr = device->getSceneManager();
```

Ok...now create the bullet' manager and add the rope to the world!

```
	const float	s=5;
	const float	h=5;
        bulletmgr = createBulletManager(device);
        CIrrBPPatchSoftBody * rope = bulletmgr->addPatchSoftBody(vector3df(-s,h,-s),vector3df(s,h,-s),vector3df(-s,h,+s),vector3df(s,h,s),10,10,10);

```


Now, we create a cube body to attach to the cloth

```
	Node2 = smgr->addCubeSceneNode(10,0,-1,vector3df(0,0,0));
	Node2->setMaterialFlag(EMF_LIGHTING,false);
	CIrrBPBoxBody * box = bulletmgr->addRigidBox(Node2,40);
```

And finally...we attach te box to the low corners of the cloth!

```
	rope->appendAnchor(box,0);
	rope->appendAnchor(box,9);
```

**When appending an** _anchor_**to a soft body, we need to specify to which node is applied the anchor. In this case we've got a 10 x 10 precision, so we attach the body to the first row corners [0~9]**

Now let's setup up the rendering loop

```
	while(device->run())
	{
		if (device->isWindowActive())
		{
			driver->beginScene(true, true, video::SColor(255,200,200,200));
			bulletmgr->stepSimulation();
			smgr->drawAll();
			driver->endScene();
		}
		else
			device->yield();
	}
```

and close the devices...

```
	delete recv;
	bulletmgr->drop();
	device->drop();
	return 0;
```

Have Fun!