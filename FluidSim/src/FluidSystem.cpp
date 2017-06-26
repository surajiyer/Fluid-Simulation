#include "FluidSystem.h"

#define ID(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

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

	velX.Resize(mem_size);
	velY.Resize(mem_size);
	dens.Resize(mem_size);

	/*u = new float[mem];
	v = new float[mem];
	u_prev = new float[mem];
	v_prev = new float[mem];
	dens = new float[mem];
	dens_prev = new float[mem];*/
}

void FluidSystem::Update(real dt)
{
	FluidUpdate fu { this, dt };

	std::vector<Tools::UpdatableR<bool, FluidUpdate>*>::iterator i = gUpdaters.begin();
	while (i != gUpdaters.end())
	{
		if (!(*i)->Update(fu)) i = gUpdaters.erase(i);
		else i++;
	}

	// zero out
	Tools::Fill<real>(velX.Prev(), 0);
	Tools::Fill<real>(velY.Prev(), 0);
	Tools::Fill<real>(dens.Prev(), 0);

	VelStep(N, velX.Curr().data(), velY.Curr().data(), velX.Prev().data(), velY.Prev().data(), visc, dt);
	DensStep(N, dens.Curr().data(), dens.Prev().data(), velX.Curr().data(), velY.Curr().data(), diff, dt);
}

void FluidSystem::SetDensity(int x, int y, real val)
{
	dens.Curr()[ID(x, y)] = val;
	//dens.Prev()[IX(x, y)] = val;
}

void FluidSystem::SetVelocity(int x, int y, real vx, real vy)
{
	auto id = ID(x, y);
	velX.Curr()[id] = vx;
	//velX.Prev()[id] = vx;
	velY.Curr()[id] = vy;
	//velY.Prev()[id] = vy;
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
	velX.SetZero();
	velY.SetZero();
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
	return { N+2, N+2 };
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
	return velX.Curr();
}

FluidSystem::list FluidSystem::GetVelY()
{
	return velY.Curr();
}

real& FluidSystem::Density(int x, int y)
{
	return dens.Curr()[ID(x,y)];
}

real& FluidSystem::VelX(int x, int y)
{
	return velX.Curr()[ID(x, y)];
}

real& FluidSystem::VelY(int x, int y)
{
	return velY.Curr()[ID(x, y)];
}

void FluidSystem::AddFluid(int x, int y, float d, float vx, float vy, bool speedModify)
{
	int id = ID(x, y);
	// update density, and find out what part the new fluid is
	// update velocity with this part
	if (speedModify) {
		real d_part = d / (dens.Curr()[id] += d);
		real d_inv_part = 1 - d_part;
		velX.Curr()[id] = d_inv_part * velX.Curr()[id] + d_part * vx;
		velY.Curr()[id] = d_inv_part * velY.Curr()[id] + d_part * vy;
	}
	else {
		dens.Curr()[id] += d;
		SetVelocity(x, y, vx, vy);
	}
}

void FluidSystem::DensStep(int N, float * x, float * x0, float * u, float * v, float diff, float dt)
{
	// x = density
	AddArrDt(N, x, x0, dt);
	SWAP(x0, x);
	Diffuse(N, 0, x, x0, diff, dt);
	SWAP(x0, x);
	Advect(N, 0, x, x0, u, v, dt);
}

void FluidSystem::VelStep(int N, float * u, float * v, float * u0, float * v0, float visc, float dt)
{
	AddArrDt(N, u, u0, dt); AddArrDt(N, v, v0, dt);
	SWAP(u0, u); Diffuse(N, 1, u, u0, visc, dt);
	SWAP(v0, v); Diffuse(N, 2, v, v0, visc, dt);
	Project(N, u, v, u0, v0);
	SWAP(u0, u); SWAP(v0, v);
	Advect(N, 1, u, u0, u0, v0, dt); Advect(N, 2, v, v0, u0, v0, dt);
	Project(N, u, v, u0, v0);
}



void FluidSystem::AddArrDt(int N, float * x, float * s, float dt)
{
	int i, size = (N + 2)*(N + 2);
	for (i = 0; i < size; i++) x[i] += dt*s[i];
}

void FluidSystem::SetBorder(int N, int b, float * x)
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

void FluidSystem::LinearSolve(int N, int b, float * x, float * x0, float a, float c)
{
	int i, j, k;

	if (a > 0) {
		for (k = 0; k < 20; k++) {
			FOR_EACH_CELL
				x[ID(i, j)] = (x0[ID(i, j)] + a*(x[ID(i - 1, j)] + x[ID(i + 1, j)] + x[ID(i, j - 1)] + x[ID(i, j + 1)])) / c;
			END_FOR
				SetBorder(N, b, x);
		}
	}
	else {
		FOR_EACH_CELL
			x[ID(i, j)] = x0[ID(i, j)];
		END_FOR
		SetBorder(N, b, x);
	}
	
}

void FluidSystem::Diffuse(int N, int b, float * x, float * x0, float diff, float dt)
{
	float a = dt*diff*N*N;
	LinearSolve(N, b, x, x0, a, 1 + 4 * a);
}

void FluidSystem::Advect(int N, int b, float * d, float * d0, float * u, float * v, float dt)
{
	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;

	dt0 = dt*N;
	FOR_EACH_CELL
		x = i - dt0*u[ID(i, j)]; y = j - dt0*v[ID(i, j)];
	if (x < 0.5f) x = 0.5f; if (x > N + 0.5f) x = N + 0.5f; i0 = (int) x; i1 = i0 + 1;
	if (y < 0.5f) y = 0.5f; if (y > N + 0.5f) y = N + 0.5f; j0 = (int) y; j1 = j0 + 1;
	s1 = x - i0; s0 = 1 - s1; t1 = y - j0; t0 = 1 - t1;
	d[ID(i, j)] = s0*(t0*d0[ID(i0, j0)] + t1*d0[ID(i0, j1)]) +
		s1*(t0*d0[ID(i1, j0)] + t1*d0[ID(i1, j1)]);
	END_FOR
		SetBorder(N, b, d);
}

void FluidSystem::Project(int N, float * u, float * v, float * p, float * div)
{
	int i, j;

	FOR_EACH_CELL
		div[ID(i, j)] = -0.5f*(u[ID(i + 1, j)] - u[ID(i - 1, j)] + v[ID(i, j + 1)] - v[ID(i, j - 1)]) / N;
	p[ID(i, j)] = 0;
	END_FOR
		SetBorder(N, 0, div); SetBorder(N, 0, p);

	LinearSolve(N, 0, p, div, 1, 4);

	FOR_EACH_CELL
		u[ID(i, j)] -= 0.5f*N*(p[ID(i + 1, j)] - p[ID(i - 1, j)]);
	v[ID(i, j)] -= 0.5f*N*(p[ID(i, j + 1)] - p[ID(i, j - 1)]);
	END_FOR
		SetBorder(N, 1, u); SetBorder(N, 2, v);
}
