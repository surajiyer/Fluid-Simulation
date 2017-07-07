#pragma once
#include "Inc.h"

#include <OpenGLBase/Include.h>
#include <OpenGLBase/src/GLCore/GraphicsIncludes.h>
#include <OpenGLBase/src/Example/GLExampleObject.h>
#include <OpenGLBase/src/CGMain/MeshGen.h>
#include <OpenGLBase/src/GLBase/Environment/Scene.h>

class SystemManager
{
public:
	SystemManager();
	~SystemManager();

	void Run();
	static void DefaultScene(FluidSystem*);

private:
	void Setup();
	void Update(float dt);
	void ReadInput();
	void ResetSystem();

	typedef void (*FSFunc)(FluidSystem*);

	System::InputManager*	gInputListener = nullptr;
	GLCore::GLEngine*		gpEngine = nullptr;
	bool					gEnableRender;
	bool					gSimulationEnabled = false;
	bool					gRefreshVisual = true;

	FluidSystem*			gpFluidSystem = nullptr;
	FluidRenderer*			gpFluidRenderer = nullptr;
	FluidInteraction*		gpFluidInteraction = nullptr;

	struct TimeAnalyse {
	public:
		double dt_max = 0;
		int stepRefresh = 200;
	private:
		double temp_total[2] = {0,0};
		int steps[2] = {0,0};
		int current_bufferid;
	public:
		void Reset();
		void Step(double dt, double sdt);
		double AvgDt();
	} gTimeAnalyse;
};

