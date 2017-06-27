#include "FluidSystem.h"

#define SWAP(x0,x) {auto& tmp=x0;x0=x;x=tmp;}

FluidSystem::FluidSystem()
{
}

FluidSystem::~FluidSystem()
{
}

void FluidSystem::Setup(int N, float diff, float visc)
{
	this->N = N;
	this->diff = diff;
	this->visc = visc;

	int mem_size = (N + 2) * (N + 2);

	vX.Resize(mem_size);
	vY.Resize(mem_size);
	dens.Resize(mem_size);
}

void FluidSystem::Update(real dt)
{
	// zero out
	Tools::Fill<real>(vX.Prev(), 0);
	Tools::Fill<real>(vY.Prev(), 0);
	Tools::Fill<real>(dens.Prev(), 0);

	CallUpdates(dt, FluidUpdate::VEL);
	VelStep(dt);

	/*for (int i = 0; i < N + 2; i++) {
		auto& vTop = vY.Curr()[ID(i, N + 1)];
		auto& vBot = vY.Curr()[ID(i, 0)];
		vTop = 0; // vTop > 0 ? 0 : vTop;
		vBot = 0; // vBot < 0 ? 0 : vBot;
		//std::cout.precision(3);
		//std::cout << std::setw(4) << v << " ";
	}
	//std::cout << "\n";*/


	CallUpdates(dt, FluidUpdate::DENS);
	DensStep(dt);
}

void FluidSystem::SetDensity(int x, int y, real val)
{
	dens.Curr()[ID(x, y)] = val;
}

void FluidSystem::SetVelocity(int x, int y, real velx, real vely)
{
	auto id = ID(x, y);
	vX.Curr()[id] = velx;
	vY.Curr()[id] = vely;
}

void FluidSystem::AddUpdater(Tools::UpdatableR<bool, FluidUpdate>* pU)
{
	gUpdaters.push_back(pU);
}

void FluidSystem::RemoveUpdater(Tools::UpdatableR<bool, FluidUpdate>* ptr, bool to_delete)
{
	Tools::RemoveOneReverse(gUpdaters, ptr);
	if (to_delete) delete ptr;
}

void FluidSystem::Clear()
{
	vX.SetZero();
	vY.SetZero();
	dens.SetZero();
}

void FluidSystem::ClearUpdaters()
{
	for (auto pU : gUpdaters) {
		delete pU;
	}

	gUpdaters.clear();
}

Tools::Surface2D<int> FluidSystem::GetSize()
{
	return { N + 2, N + 2 };
}

int FluidSystem::GetN_inner()
{
	return N;
}

FluidSystem::list FluidSystem::GetDensities()
{
	return dens.Curr();
}

FluidSystem::list FluidSystem::GetVelX()
{
	return vX.Curr();
}

FluidSystem::list FluidSystem::GetVelY()
{
	return vY.Curr();
}

real& FluidSystem::Density(int x, int y)
{
	return dens.Curr()[ID(x, y)];
}

real& FluidSystem::VelX(int x, int y)
{
	return vX.Curr()[ID(x, y)];
}

real& FluidSystem::VelY(int x, int y)
{
	return vY.Curr()[ID(x, y)];
}

void FluidSystem::AddFluid(int x, int y, float d, float velx, float vely, bool speedModify)
{
	int id = ID(x, y);
	// update density, and find out what part the new fluid is
	// update velocity with this part
	if (speedModify) {
		real d_part = d / (dens.Curr()[id] += d);
		real d_inv_part = 1 - d_part;
		vX.Curr()[id] = d_inv_part * vX.Curr()[id] + d_part * velx;
		vY.Curr()[id] = d_inv_part * vY.Curr()[id] + d_part * vely;
	}
	else {
		dens.Curr()[id] += d;
		SetVelocity(x, y, velx, vely);
	}
}

void FluidSystem::CallUpdates(real dt, FluidUpdate::Type t)
{
	FluidUpdate fu{ this, dt, t };
	std::vector<Tools::UpdatableR<bool, FluidUpdate>*>::iterator i = gUpdaters.begin();
	while (i != gUpdaters.end())
	{
		if (!(*i)->Update(fu)) i = gUpdaters.erase(i);
		else i++;
	}
}

void FluidSystem::DensStep(float dt)
{
	// x = density
	AddArrDt(N, dens.Curr(), dens.Prev(), dt);
	dens.Swap();
	Diffuse(N, 0, dens.Curr(), dens.Prev(), diff, dt);
	dens.Swap();
	Advect(N, 0, dens.Curr(), dens.Prev(), vX.Curr(), vY.Curr(), dt);
}

void FluidSystem::VelStep(float dt)
{
	AddArrDt(N, vX.Curr(), vY.Prev(), dt); 
	AddArrDt(N, vY.Curr(), vY.Prev(), dt);

	vX.Swap();
	
	Diffuse(N, 1, vX.Curr(), vX.Prev(), visc, dt);

	vY.Swap();

	Diffuse(N, 2, vY.Curr(), vY.Prev(), visc, dt);
	Project(N, vX.Curr(), vY.Curr(), vX.Prev(), vY.Prev());

	vX.Swap();
	vY.Swap();

	Advect(N, 1, vX.Curr(), vX.Prev(), vX.Prev(), vY.Prev(), dt);
	Advect(N, 2, vY.Curr(), vY.Prev(), vX.Prev(), vY.Prev(), dt);
	Project(N, vX.Curr(), vY.Curr(), vX.Prev(), vY.Prev());
}

void FluidSystem::AddArrDt(int N, list x, list s, float dt)
{
	int i, size = (N + 2)*(N + 2);
	for (i = 0; i < size; i++) x[i] += dt*s[i];
}

void FluidSystem::SetBorder(int N, int b, list x)
{
	int i;

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

void FluidSystem::LinearSolve(int N, int b, list x, list x0, float a, float c)
{
	int i, j, k;

	int limit = a > 0 ? 20 : 1;

	for (k = 0; k < limit; k++) {
		for (i = 1; i <= N; i++) {
			for (j = 1; j <= N; j++) {
				x[ID(i, j)] = (x0[ID(i, j)] + a*(x[ID(i - 1, j)] + x[ID(i + 1, j)] + x[ID(i, j - 1)] + x[ID(i, j + 1)])) / c;
			}
		}
		SetBorder(N, b, x);
	}
}

void FluidSystem::Diffuse(int N, int b, list x, list x0, float diff, float dt)
{
	float a = dt*diff*N*N;
	LinearSolve(N, b, x, x0, a, 1 + 4 * a);
}

void FluidSystem::Advect(int N, int b, list d, list d0, list u, list v, float dt)
{
	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;

	dt0 = dt*N;
	for (i = 1; i <= N; i++) {
		for (j = 1; j <= N; j++) {
			x = i - dt0*u[ID(i, j)];
			y = j - dt0*v[ID(i, j)];

			if (x < 0.5f) x = 0.5f;
			if (x > N + 0.5f) x = N + 0.5f;

			i0 = (int) x;
			i1 = i0 + 1;

			if (y < 0.5f) y = 0.5f; 
			if (y > N + 0.5f) y = N + 0.5f; 
			j0 = (int) y; 
			j1 = j0 + 1;

			s1 = x - i0; 
			s0 = 1 - s1; 
			t1 = y - j0;
			t0 = 1 - t1;
			d[ID(i, j)] = s0*(t0*d0[ID(i0, j0)] + t1*d0[ID(i0, j1)]) + s1*(t0*d0[ID(i1, j0)] + t1*d0[ID(i1, j1)]);
		}
	}
	SetBorder(N, b, d);
}

void FluidSystem::Project(int N, list u, list v, list p, list div)
{
	int i, j;

	for (i = 1; i <= N; i++) {
		for (j = 1; j <= N; j++) {
			div[ID(i, j)] = -0.5f * (u[ID(i + 1, j)] - u[ID(i - 1, j)] + v[ID(i, j + 1)] - v[ID(i, j - 1)]) / N;
			p[ID(i, j)] = 0;
		}
	}
	SetBorder(N, 0, div); 
	SetBorder(N, 0, p);

	LinearSolve(N, 0, p, div, 1, 4);

	for (i = 1; i <= N; i++) {
		for (j = 1; j <= N; j++) {
			u[ID(i, j)] -= 0.5f * N * (p[ID(i + 1, j)] - p[ID(i - 1, j)]);
			v[ID(i, j)] -= 0.5f * N * (p[ID(i, j + 1)] - p[ID(i, j - 1)]);
		}
	}

	SetBorder(N, 1, u); 
	SetBorder(N, 2, v);
}
