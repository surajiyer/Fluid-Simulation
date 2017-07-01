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
			for (int y = 0; y <= size+1; y++) {
				for (int x = 0; x <= size+1; x++) {
					real d = (y > size / 4 && y < 3 * size / 4
						&& x > size / 4 && x < 3 * size / 4) ? 0.5f : 0;
					fu.fs->SetDensity(x, y, d);
				}
			}

			delete this;
		}
	}

	return false;
}

Blower::Blower(int dn)
{
	this->dn = dn;
}

bool Blower::Update(FluidUpdate fu)
{
	if (fu.type == FluidUpdate::VEL) {
		real d = 5;
		real v = 1.8 * fu.fs->GetSize().width / (64.0f);
		real vx = dn == 0 ? v : -v;
		real vy = dn == 0 ? v : v;
		int N = fu.fs->N;
		int size = 2;
		int offset = 3;
		int low = 1 + (dn == 0 ? offset : (N - offset - size));
		int high = 1 + (dn == 0 ? (offset + size) : (N - offset));
		for (int i = low; i < high; i++) {
			for (int j = offset; j < offset + size; j++) {
				fu.fs->AddFluid(i, j, d, vx, vy, dn);
			}
		}
	}

	return true;
}

bool Gravity::Update(FluidUpdate fu)
{
	if (fu.type == FluidUpdate::VEL) {
		int N = fu.fs->GetN_inner();
		for (int i = 1; i <= N; i++) {
			for (int j = 1; j <= N; j++) {
				fu.fs->VelY(i, j) -= fu.dt * 0.7 * fu.fs->CombinedDensity(i, j);
			}
		}
	}

	return true;
}

}