#include "FluidFillers.h"
#include "FluidSystem.h"

namespace FluidFillers {

void HalfFull(FluidSystem* pSys)
{
	auto size = pSys->GetSize();
	for (int y = 0; y < size.height; y++) {
		for (int x = 0; x < size.width; x++) {
			real d = (y > size.height / 4 && y < 3 * size.height / 4 
					&& x > size.width / 4 && x < 3 * size.width / 4) ? 0.5 : 0;
			pSys->SetDensity(x, y, d);
		}
	}
	pSys->SetZeroVelocity();
}

}