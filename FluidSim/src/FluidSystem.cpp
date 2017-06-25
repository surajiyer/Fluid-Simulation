#include "FluidSystem.h"

#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}


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
	gDiffuse = 0.0001;
	gViscosity = 0.9;

	// data
	for (int i = 0; i < 2; i++) {
		gDens.gStates[i].resize(gTrueCellCount);
		gVelY.gStates[i].resize(gTrueCellCount);
		gVelX.gStates[i].resize(gTrueCellCount);
	}

	gConnections.resize(gTrueCellCount);

	// set border connections to false
}

void FluidSystem::SetDensity(int x, int y, real val)
{
	gDens.Curr()[ID(x, y)] = val;
	gDens.Prev()[ID(x, y)] = val;
}

void FluidSystem::Update(real dt)
{
	ForcesStep(dt);
	VelocityStep(dt);
	DensityStep(dt);
	//DampStep(dt);
}

Tools::Surface2D<uint32_t> FluidSystem::GetSize()
{
	return gTrueSurface;
}

std::vector<real>& FluidSystem::GetDensities()
{
	return gDens.gStates[gDens.gCurrID]; //.Current();
}

std::vector<real>& FluidSystem::GetVelX()
{
	return gVelY.gStates[gVelY.gCurrID];
}

std::vector<real>& FluidSystem::GetVelY()
{
	return gVelX.gStates[gVelX.gCurrID];
}

void FluidSystem::SetZeroVelocity()
{
	for (int i = 0; i < gTrueCellCount; i++) {
		gVelX.Curr()[i] = 0;
		gVelX.Prev()[i] = 0;
		gVelY.Curr()[i] = 0;
		gVelY.Prev()[i] = 0;
	}
}

void FluidSystem::DensityStep(real dt)
{
	AddArrayDt(gDens.Curr(), gDens.Prev(), dt);

	gDens.Swap();
	Diffuse(0, gDens.Curr(), gDens.Prev(), gDiffuse, dt);

	gDens.Swap();
	Advect(0, gDens.Curr(), gDens.Prev(), gVelY.Curr(), gVelX.Curr(), dt);
}

void FluidSystem::VelocityStep(real dt)
{
	int N = gNi;

	AddArrayDt(gVelY.Curr(), gVelY.Prev(), dt);
	AddArrayDt(gVelX.Curr(), gVelX.Prev(), dt);
	gVelY.Swap();
	Diffuse(1, gVelY.Curr(), gVelY.Prev(), gViscosity, dt);
	gVelX.Swap();
	Diffuse(1, gVelX.Curr(), gVelX.Prev(), gViscosity, dt);
	Project(gVelY.Curr(), gVelX.Curr(), gVelY.Prev(), gVelX.Prev());
	gVelY.Swap();
	gVelX.Swap();
	Advect(1, gVelY.Curr(), gVelY.Prev(), gVelY.Prev(), gVelX.Prev(), dt);
	Advect(1, gVelX.Curr(), gVelX.Prev(), gVelY.Prev(), gVelX.Prev(), dt);
	Project(gVelY.Curr(), gVelX.Prev(), gVelY.Prev(), gVelX.Prev());

	/*
	add_source(N, u(), u0(), dt); add_source(N, v(), v0(), dt);
	gVel_x.Swap(); diffuse(N, 1, u(), u0(), visc(), dt);
	gVel_y.Swap(); diffuse(N, 2, v(), v0(), visc(), dt);
	project(N, u(), v(), u0(), v0());
	gVel_x.Swap(); gVel_y.Swap();
	advect(N, 1, u(), u0(), u0(), v0(), dt); advect(N, 2, v(), v0(), u0(), v0(), dt);
	project(N, u(), v(), u0(), v0());
	*/
}

void FluidSystem::ForcesStep(real dt)
{
	list u = gVelY.Curr();
	list v = gVelX.Curr();

	for (int i = 0; i < gTrueCellCount; i++) {
		//u[i] -= dt * 1.0;
		v[i] -= dt * 1.0;
	}
}

void FluidSystem::DampStep(real dt)
{
	list vx = gVelY.Curr();
	list vy = gVelX.Curr();
	for (int i = 0; i < gTrueCellCount; i++) {
		//real my = (v[i] * v[i]) * 10;
		//real mx = (u[i] * u[i]) * 10;
		vy[i] *= powf(0.95, dt);// / (1 + dt * mx);
		vx[i] *= powf(0.95, dt);// / (1 + dt * my);
	}
}

void FluidSystem::Diffuse(int b, list x, list x0, real strength, real dt)
{
	float a = dt * gDiffuse * gInnerCellCount;
	LinearSolve(b, x, x0, a, 1 + 4 * a);
}

void FluidSystem::Advect(int b, list d, list d0, list u, list v, real dt)
{
	// TODO: make compatible with width/height

	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;
	int N = gNi;
	dt0 = dt*N;
	for (i = 1; i <= N; i++) {
		for (j = 1; j <= N; j++) {
			x = i - dt0*u[IX(i, j)]; y = j - dt0*v[IX(i, j)];
			if (x < 0.5f) x = 0.5f; if (x > N + 0.5f) x = N + 0.5f; i0 = (int) x; i1 = i0 + 1;
			if (y < 0.5f) y = 0.5f; if (y > N + 0.5f) y = N + 0.5f; j0 = (int) y; j1 = j0 + 1;
			s1 = x - i0; s0 = 1 - s1; t1 = y - j0; t0 = 1 - t1;
			d[IX(i, j)] = s0*(t0*d0[IX(i0, j0)] + t1*d0[IX(i0, j1)]) +
				s1*(t0*d0[IX(i1, j0)] + t1*d0[IX(i1, j1)]);
		}
	}
	SetBorder(b, d);
}

void FluidSystem::LinearSolve(int b, list x, list x0, real a, real c) {
	int i, j, k;
	int N = gNi;
	for (k = 0; k < 20; k++) {
		for (i = 1; i <= N; i++) {
			for (j = 1; j <= N; j++) {
				x[IX(i, j)] = (x0[IX(i, j)] + a*(x[IX(i - 1, j)] + x[IX(i + 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)])) / c;
			}
		}
		SetBorder(b, x);
	}
}

void FluidSystem::SetBorder(int b, list x)
{
	int i;
	int N = gNi;

	// sides
	for (i = 1; i <= N; i++) {
		x[ID(0, i)] = b == 1 ? -x[ID(1, i)] : x[ID(1, i)];
		x[ID(N + 1, i)] = b == 1 ? -x[ID(N, i)] : x[ID(N, i)];
		x[ID(i, 0)] = b == 2 ? -x[ID(i, 1)] : x[ID(i, 1)];
		x[ID(i, N + 1)] = b == 2 ? -x[ID(i, N)] : x[ID(i, N)];
	}

	// corners
	x[ID(0, 0)] = 0.5f*(x[ID(1, 0)] + x[ID(0, 1)]);
	x[ID(0, N + 1)] = 0.5f*(x[ID(1, N + 1)] + x[ID(0, N)]);
	x[ID(N + 1, 0)] = 0.5f*(x[ID(N, 0)] + x[ID(N + 1, 1)]);
	x[ID(N + 1, N + 1)] = 0.5f*(x[ID(N, N + 1)] + x[ID(N + 1, N)]);
}

void FluidSystem::Project(list u, list v, list p, list div)
{
	int N = gNi;
	int i, j;

	for (i = 1; i <= N; i++) {
		for (j = 1; j <= N; j++) {
			div[IX(i, j)] = -0.5f*(u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]) / N;
			p[IX(i, j)] = 0;
		}
	}

	SetBorder(0, div); 
	SetBorder(0, p);

	LinearSolve(0, p, div, 1, 4);

	for (i = 1; i <= N; i++) {
		for (j = 1; j <= N; j++) {
			u[IX(i, j)] -= 0.5f*N*(p[IX(i + 1, j)] - p[IX(i - 1, j)]);
			v[IX(i, j)] -= 0.5f*N*(p[IX(i, j + 1)] - p[IX(i, j - 1)]);
		}
	}

	SetBorder(1, u); 
	SetBorder(2, v);
}

void FluidSystem::AddArrayDt(list x, list s, real dt)
{
	for (int i = 0; i < gTrueCellCount; i++)
		x[i] += dt*s[i];
}