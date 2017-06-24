#include "FluidSystem.h"

FluidSystem::FluidSystem()
{
}


FluidSystem::~FluidSystem()
{

}

void FluidSystem::Setup(Tools::Surface2D<uint32_t> surface)
{
	// metadata
	gTrueSurface = surface;
	gTrueCellCount = surface.Area();
	gNi = surface.width - 2;
	gInnerCellCount = gNi * gNi;

	// parameters
	gDiffuse		= 0.00005;
	gViscosity		= 0.5;

	// data
	for (int i = 0; i < 2; i++) {
		gDensities.gStates[i].resize(gTrueCellCount);
		gVelU.gStates[i].resize(gTrueCellCount);
		gVelV.gStates[i].resize(gTrueCellCount);
	}

	gConnections.resize(gTrueCellCount);
	
	// set border connections to false
}

void FluidSystem::SetDensity(int x, int y, real v)
{
	gDensities.Current()[ID(x, y)] = v;
	gDensities.Previous()[ID(x, y)] = v;
}

void FluidSystem::Update(real dt)
{
	ForcesStep(dt);
	VelocityStep(dt);
	DensityStep(dt);
}

Tools::Surface2D<uint32_t> FluidSystem::GetSize()
{
	return gTrueSurface;
}

std::vector<real>& FluidSystem::GetDensities()
{
	return gDensities.gStates[gDensities.gCurrID]; //.Current();
}

void FluidSystem::SetZeroVelocity()
{
	for (int i = 0; i < gTrueCellCount; i++) {
		gVelU.Current()[i] = 0;
		gVelV.Current()[i] = 0;
		gVelU.Previous()[i] = 0;
		gVelV.Previous()[i] = 0;
	}
}

void FluidSystem::DensityStep(real dt)
{
	AddArrayDt(gDensities.Current(), gDensities.Previous(), dt);

	gDensities.Swap();
	Diffuse(0, gDensities.Current(), gDensities.Previous(), gDiffuse, dt);
	
	gDensities.Swap();
	Advect(0, gDensities.Current(), gDensities.Previous(),
		gVelU.Current(), gVelV.Current(), dt);
}

void FluidSystem::VelocityStep(real dt)
{
	AddArrayDt(gVelU.Current(), gVelU.Previous(), dt);
	AddArrayDt(gVelV.Current(), gVelV.Previous(), dt);

	gVelU.Swap();
	Diffuse(1, gVelU.Current(), gVelU.Previous(), gViscosity, dt);

	gVelV.Swap();
	Diffuse(2, gVelV.Current(), gVelV.Previous(), gViscosity, dt);
	
	Project(gVelU.Current(), gVelV.Current(), 
		gVelU.Previous(), gVelV.Previous());

	gVelU.Swap();
	gVelV.Swap();

	Advect(1, gVelU.Current(), gVelU.Previous(), 
		gVelU.Previous(), gVelV.Previous(), dt);
	Advect(2, gVelV.Current(), gVelU.Previous(),
		gVelU.Previous(), gVelV.Previous(), dt);

	Project(gVelU.Current(), gVelV.Current(),
		gVelU.Previous(), gVelV.Previous());
}

void FluidSystem::ForcesStep(real dt)
{
	list v = gVelV.Previous();
	
	for (int i = 0; i < gTrueCellCount; i++) {
		v[i] -= dt * 0.1;
	}
}

void FluidSystem::Diffuse(int b, list x, list x0, real strength, real dt)
{
	real a = dt * strength * gInnerCellCount;
	LinearSolve(b, x, x0, a, 1 + 4 * a);
}

void FluidSystem::Advect(int b, list d, list d0, list u, list v, real dt)
{
	// TODO: make compatible with width/height

	int i0, j0, i1, j1;
	real x, y, s0, s1, t0, t1, dt0;

	dt0 = dt * gNi;

	for (int iy = 1; iy <= gNi; iy++) {
		for (int ix = 1; ix <= gNi; ix++) {
			x = ix - dt0*u[ID(ix, iy)]; 
			y = iy - dt0*v[ID(ix, iy)];
			
			if (x < 0.5f) x = 0.5f; 
			if (x > gNi + 0.5f) x = gNi + 0.5f; 
			
			i0 = (int) x; 
			i1 = i0 + 1;
			
			if (y < 0.5f) y = 0.5f; 
			if (y > gNi + 0.5f) y = gNi + 0.5f; 

			j0 = (int) y; 
			j1 = j0 + 1;
			s1 = x - i0; 
			s0 = 1 - s1; 
			t1 = y - j0; 
			t0 = 1 - t1;

			d[ID(ix, iy)] = s0*(t0*d0[ID(i0, j0)] + t1*d0[ID(i0, j1)]) +
							s1*(t0*d0[ID(i1, j0)] + t1*d0[ID(i1, j1)]);
			// TODO: take out d0 -> common factor
		}
	}

	SetBand(b, d);
}

void FluidSystem::LinearSolve(int b, list x, list x0, real a, real c)
{
	int k, ix, iy;

	// TODO: maybe instead of 20 steps, do a horizontal and vertical kernel step as approximation?
	for (k = 0; k < 20; k++) {
		for (iy = 1; iy <= gNi; iy++) {
			for (ix = 1; ix <= gNi; ix++) {
				x[ID(ix, iy)] = (x0[ID(ix, iy)] + 
					a * (x[ID(ix - 1, iy)] + x[ID(ix + 1, iy)] + x[ID(ix, iy - 1)] + x[ID(ix, iy + 1)])) / c;
			}
		}

		// TODO: width and hight seperation
		SetBand(b, x);
	}
}

void FluidSystem::SetBand(int b, list x)
{
	// TODO: width and hight seperation for N

	// sides
	for (int i = 1; i <= gNi; i++) {
		x[ID(0, i)] = b == 1 ? -x[ID(1, i)] : x[ID(1, i)];
		x[ID(gNi + 1, i)] = b == 1 ? -x[ID(gNi, i)] : x[ID(gNi, i)];
		x[ID(i, 0)] = b == 2 ? -x[ID(i, 1)] : x[ID(i, 1)];
		x[ID(i, gNi + 1)] = b == 2 ? -x[ID(i, gNi)] : x[ID(i, gNi)];
	}

	// corners
	x[ID(0, 0)] = 0.5f*(x[ID(1, 0)] + x[ID(0, 1)]);
	x[ID(0, gNi + 1)] = 0.5f*(x[ID(1, gNi + 1)] + x[ID(0, gNi)]);
	x[ID(gNi + 1, 0)] = 0.5f*(x[ID(gNi, 0)] + x[ID(gNi + 1, 1)]);
	x[ID(gNi + 1, gNi + 1)] = 0.5f*(x[ID(gNi, gNi + 1)] + x[ID(gNi + 1, gNi)]);
}

void FluidSystem::Project(list u, list v, list p, list div)
{
	// for each cell
	for (int iy = 1; iy <= gNi; iy++) {
		for (int ix = 1; ix <= gNi; ix++) {
			div[ID(ix, iy)] = -0.5f*(u[ID(ix + 1, iy)] - u[ID(ix - 1, iy)] + v[ID(ix, iy + 1)] - v[ID(ix, iy - 1)]) / gNi;
			p[ID(ix, iy)] = 0;
		}
	}

	SetBand(0, div); 
	SetBand(0, p);

	LinearSolve(0, p, div, 1, 4);

	// for each inner cell
	for (int iy = 1; iy <= gNi; iy++) {
		for (int ix = 1; ix <= gNi; ix++) {
			u[ID(ix, iy)] -= 0.5f * gNi * (p[ID(ix + 1, iy)] - p[ID(ix - 1, iy)]);
			v[ID(ix, iy)] -= 0.5f * gNi * (p[ID(ix, iy + 1)] - p[ID(ix, iy - 1)]);
		}
	}

	SetBand(1, u); 
	SetBand(2, v);
}

void FluidSystem::AddArrayDt(list x, list
	s, real dt)
{
	for (int i = 0; i < gTrueCellCount; i++) {
		x[i] += dt * s[i];
	} // TODO : Check vectorize operation
}