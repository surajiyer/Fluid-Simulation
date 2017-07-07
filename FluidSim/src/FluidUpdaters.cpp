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

Blower::Blower(real x, real y, real dx, real dy, real sc, real v, real d, int fid)
{
	this->px = x;
	this->py = y;
	this->dx = dx;
	this->dy = dy;
	this->scale = sc;
	this->vel = v;
	this->dens = d;
	this->fid = fid;
}

bool Blower::Update(FluidUpdate fu)
{
	if (fu.type == FluidUpdate::VEL) {
		real d = dens;
		real v = this->vel * fu.fs->GetSize().width / (64.0f);

		real vx = dx * v;
		real vy = dy * v;

		int N = fu.fs->N;
		int lowX = 1 + -scale * N + px * N;
		int highX = 1 + scale * N + px * N;
		int lowY = 1 + -scale * N + py * N;
		int highY = 1 + scale * N + py * N;
		for (int i = lowY; i < highY; i++) {
			for (int j = lowX; j < highX; j++) {
				fu.fs->AddFluid(i, j, d, vx, vy, fid);
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