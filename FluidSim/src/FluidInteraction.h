#pragma once
#include "Inc.h"
#include "FluidSystem.h"

#include <WindowWin32/Include.h>

class FluidInteraction :
	public System::InputListener,
	public Tools::ResizedListener,
	public Tools::UpdatableR<bool, FluidUpdate> 
{
public:
	FluidInteraction(System::InputManager*, Tools::ResizedShouter* surface, FluidRenderer* pRenderer);
	virtual ~FluidInteraction();

	virtual bool ObjectResized(void* pCaller, Surface2D<int> newSize) override;
	virtual bool Update(FluidUpdate) override;

protected:

	virtual void MouseClick(int x, int y) override;
	virtual void MouseMove(int x, int y) override;
	virtual void MouseRelease(int x, int y) override;
	virtual void MouseScroll(int x, int y, int delta) override;
	virtual void KeyClick(char key) override;

	void*					gpSurface;
	Tools::Surface2D<int>	gSurface;
	FluidRenderer*			gpRenderer;

	real source = 10;
	real force = 2;
	int mx = 0, my = 0, omx = 0, omy = 0;
	bool m_down = false;

	FluidCollider* gpLockObj = nullptr;
	bool locked = false;
	bool first = false;
};

