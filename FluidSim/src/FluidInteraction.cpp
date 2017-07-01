#include "FluidInteraction.h"


FluidInteraction::FluidInteraction(System::InputManager* pInputManager, Tools::ResizedShouter* pSurface, FluidRenderer* pRenderer)
{
	pInputManager->AddInputListener(this);
	pSurface->AddResizedListener(this);
	gSurface = pSurface->GetSize();
	gpSurface = pSurface;
	gpRenderer = pRenderer;
}

FluidInteraction::~FluidInteraction()
{

}

bool FluidInteraction::ObjectResized(void* pCaller, Surface2D<int> newSize)
{
	if (pCaller == gpSurface) {
		gSurface = newSize;
		return true;
	}
	return false;
}

bool FluidInteraction::Update(FluidUpdate fu)
{
	if (fu.type == FluidUpdate::VEL) {
		int i, j;
		int N = fu.fs->GetN_inner();

		if (!m_down) return true;

		i = (int) ((mx / (float) gSurface.width)*N + 1);
		j = (int) (((gSurface.height - my) / (float) gSurface.width)*N + 1);

		if (i<1 || i>N || j<1 || j>N) return true;

		if (m_down) {
			fu.fs->VelX(i, j) = force * (mx - omx);
			fu.fs->VelY(i, j) = force * (omy - my);
		}

		if (m_down) {
			fu.fs->Density(i, j, 0) = source;
		}

		omx = mx;
		omy = my;
	}

	return true;
}

void FluidInteraction::MouseClick(int x, int y)
{
	omx = mx = x;
	omy = my = y;
	m_down = true;
}

void FluidInteraction::MouseMove(int x, int y)
{
	mx = x; 
	my = y;
}

void FluidInteraction::MouseRelease(int x, int y)
{
	omx = mx = x;
	omy = my = y;
	m_down = false;
}

void FluidInteraction::MouseScroll(int x, int y, int delta)
{
	
}

void FluidInteraction::KeyClick(char key)
{
	
}
