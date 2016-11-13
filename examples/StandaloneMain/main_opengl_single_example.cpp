/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../Utils/b3Clock.h"




#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include <stdio.h>
#include "../ExampleBrowser/OpenGLGuiHelper.h"
#include <iostream>

CommonExampleInterface*    example;
int gSharedMemoryKey=-1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove( float x, float y)
{
	bool handled = false; 
	handled = example->mouseMoveCallback(x,y); 	 
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback (x,y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback  = 0;
static void OnMouseDown(int button, int state, float x, float y) {
	bool handled = false;

	handled = example->mouseButtonCallback(button, state, x,y); 
	if (!handled)
	{
		if (prevMouseButtonCallback )
			prevMouseButtonCallback (button,state,x,y);
	}
}

class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;
public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		:m_app(app)
	{
	}
};
int main(int argc, char* argv[])
{
	
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Standalone Example",1024,768,true);
	
	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);
	
	OpenGLGuiHelper gui(app,false);
	//LessDummyGuiHelper gui(app);
	//DummyGUIHelper gui;

	CommonExampleOptions options(&gui);
	srand(time(NULL));
	example = StandaloneExampleCreateFunc(options);
	//std::cout << "before vertexPoints" << std::endl;
	//initialize default vertices



	float l = 10;
	float b = 11;
	float h = 12;
	float length = l;
	float height = b;
	float width = h;
	btVector3 v0;
	v0.setX(0.0f);
	v0.setY(0.0f);
	v0.setZ(0.0f);
	btVector3 v1;
	v1.setX(0.0f);
	v1.setY(0.0f);
	v1.setZ(width);
	btVector3 v2;
	v2.setX(length);
	v2.setY(0.0f);
	v2.setZ(width);
	btVector3 v3;
	v3.setX(length);
	v3.setY(0.0f);
	v3.setZ(0.0f);
	btVector3 v4;
	v4.setX(0.0f);
	v4.setY(height);
	v4.setZ(0.0f);
	btVector3 v5;
	v5.setX(0.0f);
	v5.setY(height);
	v5.setZ(width);
	btVector3 v6;
	v6.setX(length);
	v6.setY(height);
	v6.setZ(width);
	btVector3 v7;
	v7.setX(length);
	v7.setY(height);
	v7.setZ(0.0f);


	vector<btVector3> v;
	v.push_back(v0);
	v.push_back(v1);
	v.push_back(v2);
	v.push_back(v3);
	v.push_back(v4);
	v.push_back(v5);
	v.push_back(v6);
	v.push_back(v7);
	example->vertexPoints(v);
	//std::cout << "passed vertexPoints" << std::endl;
	example->initPhysics();
	example->resetCamera();



	b3Clock clock;
	btScalar seconds_pass = 0;
	//std::cout << "passed init Physics" << std::endl;
	do
	{
		app->m_instancingRenderer->init();
        app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		example->stepSimulation(dtSec);
	  	clock.reset();
	  	seconds_pass += dtSec;

		example->renderScene();
 	
		DrawGridData dg;
        dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);
		
		app->swapBuffer();

		example->checkDice();
	} while (seconds_pass <= 100);

	example->checkFace();
	example->exitPhysics();
	
	delete example;
	delete app;
	return 0;
}

