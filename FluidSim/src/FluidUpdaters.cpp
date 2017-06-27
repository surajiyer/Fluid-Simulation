#include "FluidUpdaters.h"
#include "FluidSystem.h"

namespace FluidUpdaters {

void HalfFull(FluidSystem* pSys)
{

}

bool Square::Update(FluidUpdate fu)
{
	if (fu.type == FluidUpdate::VEL) {
		if (id++ == 0) {
			fu.fs->Clear();

			auto size = fu.fs->N;
			for (int y = 0; y < size; y++) {
				for (int x = 0; x < size; x++) {
					real d = (y > size / 4 && y < 3 * size / 4
						&& x > size / 4 && x < 3 * size / 4) ? 0.5 : 0;
					fu.fs->SetDensity(x, y, d);
				}
			}

			delete this;
		}
	}

	return false;
}

bool Blower::Update(FluidUpdate fu)
{
	if (fu.type == FluidUpdate::VEL) {
		real d = fu.fs->GetSize().Area() / (64.0*4.0*64.0*4.0);
		real v = 0.5 * fu.fs->GetSize().width / (64.0);
		real vx = v;
		real vy = v;
		for (int i = 3; i < 5; i++) {
			for (int j = 3; j < 5; j++) {
				fu.fs->AddFluid(i, j, d, vx, vy, false);
			}
		}
	}

	return true;
}

bool Gravity::Update(FluidUpdate fu)
{
	if (fu.type == FluidUpdate::VEL) {
		int N = fu.fs->GetN_inner();
		for (int i = 2; i <= N-1; i++) {
			for (int j = 2; j <= N-1; j++) {
				fu.fs->VelY(i, j) -= fu.dt * 5 * (fu.fs->Density(i, j) > 0.0001 ? 1 : 0);
			}
		}
	}

	return true;
}

}