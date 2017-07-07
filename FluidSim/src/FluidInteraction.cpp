#include "FluidInteraction.h"
#include "FluidCollider.h"


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
		int NOuter = fu.fs->GetN_inner() + 2;

		if (!m_down) return true;

		i = (int) ((mx / (float) gSurface.width)*NOuter);
		j = (int) (((gSurface.height - my) / (float) gSurface.height)*NOuter);

		if (i<1 || i>NOuter || j<1 || j>NOuter) return true;

		if (m_down) {
			if ((fu.fs->CellInfo(i, j) & OBJECT) && first) {
				for (auto* ptr : fu.fs->GetObjects()) {
					if (ptr->Contains(i, j)) {
						gpLockObj = ptr;
						locked = true;
						std::cout << "\nLOCK\n";
						break;
					}
				}
			}

			if (locked) {
				real dragForce = 1;
				auto objPos = gpLockObj->GetLoc();
				real mPosX = ((mx / (float) gSurface.width)*NOuter);
				real mPosY = (((gSurface.height - my) / (float) gSurface.height)*NOuter);
				real dx = dragForce * (mPosX - objPos(0));
				real dy = dragForce * (mPosY - objPos(1));
				//std::cout << "[" << objPos(0) << ", " << objPos(1) << "]" << mPosX << "," << mPosY << " | " << dx << "," << dy << "\n";
				gpLockObj->AddVel(dx, dy);
			}
			else {
				fu.fs->VelX(i, j) = force * (mx - omx);
				fu.fs->VelY(i, j) = force * (omy - my);
				//fu.fs->Density(i, j, 0) = source;
				first = false;
			}
			first = false;
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
	first = true;
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
	locked = false;
}

void FluidInteraction::MouseScroll(int x, int y, int delta)
{

}

void FluidInteraction::KeyClick(char key)
{

}
