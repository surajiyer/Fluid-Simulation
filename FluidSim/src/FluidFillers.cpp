#include "FluidFillers.h"
#include "FluidSystem.h"

namespace FluidFillers {

void HalfFull(FluidSystem* pSys)
{

}

bool Square::Update(FluidSystem* pSys)
{
	if (id++ == 0) {
		pSys->Clear();

		auto size = pSys->N;
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				real d = (y > size / 4 && y < 3 * size / 4
					&& x > size / 4 && x < 3 * size / 4) ? 0.5 : 0;
				pSys->SetDensity(x, y, d);
			}
		}

		delete this;
	}

	return false;
}

bool Blower::Update(FluidSystem* fs)
{
	real d = fs->GetSize().Area() / (64.0*4.0*64.0*4.0);
	real v = fs->GetSize().width / (64.0);
	real vx = v;
	real vy = v;
	for (int i = 3; i < 5; i++) {
		for (int j = 3; j < 5; j++) {
			fs->AddFluid(i, j, d, vx, vy, false);
		}
	}

	return true;
}

}