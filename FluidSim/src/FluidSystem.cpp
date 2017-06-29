#include "FluidSystem.h"
#include "FluidCollider.h"

#include <Tools/Math.h>
#include <limits> // for infinity

FluidSystem::FluidSystem()
{
}

FluidSystem::~FluidSystem()
{
}

void FluidSystem::Setup(int N, real diff, real visc, real vorticity, FSType fst)
{
	this->N = N;
	this->diff = diff;
	this->visc = visc;
	this->vorticity = vorticity;
	this->steps = 20;

	int mem_size = (N + 2) * (N + 2);

	vX.Resize(mem_size);
	vY.Resize(mem_size);
	dens.Resize(mem_size);

	buffer1.resize(mem_size);
	buffer2.resize(mem_size);
	buffer3.resize(mem_size);
	buffer4.resize(mem_size);

	vort.resize(mem_size);
	absVort.resize(mem_size);
	gradVortX.resize(mem_size);
	gradVortY.resize(mem_size);
	lenGrad.resize(mem_size);
	vcfx.resize(mem_size);
	vcfy.resize(mem_size);

	cellInfo.resize(mem_size);

	ResetCellInfo();

	// TODO; check if border is closed

	// TODO : temp code
	// make a square

	CalcBorderFromContent();

	//for (int i = 0; i < N + 2; i++) {
	//	cellInfo[ID(i, N / 2)] &= ~CellInfo::RIGHT;
	//	cellInfo[ID(i, 1 + N / 2)] &= CellInfo::LEFT;
	//}

	SetType(fst);
}

void FluidSystem::SetType(FSType fst)
{
	/*
		TODO: add pressure force to V2
	*/

	if (fst == FSType::ORIGINAL) {
		pDensStep = &FluidSystem::DensStep1;
		pVelStep = &FluidSystem::VelStep1;
	}
	else if (fst == FSType::ORIGINAL_BORDERED) {
		pDensStep = &FluidSystem::DensStep1B;
		pVelStep = &FluidSystem::VelStep1B;
	}
	else if (fst == FSType::V2) {
		pDensStep = &FluidSystem::DensStep2;
		pVelStep = &FluidSystem::VelStep1;
	}
	else if (fst == FSType::V2_BORDERED) {
		pDensStep = &FluidSystem::DensStep2; // TODO: make b version
		pVelStep = &FluidSystem::VelStep1B;
	}
	else if (fst == FSType::HYBRID) {
		pDensStep = &FluidSystem::DensStep3;
		pVelStep = &FluidSystem::VelStep1;
	}
}

void FluidSystem::Update(real dt)
{
	// zero out
	Tools::Fill<real>(vX.Prev(), 0);
	Tools::Fill<real>(vY.Prev(), 0);
	Tools::Fill<real>(dens.Prev(), 0);

	CallColliders();

	// todo: remove temp square code
	real size = 0.3;
	int low = 1 + N / 2 - size * N / 2;
	int high = 1 + N / 2 + size * N / 2;

	for (int i = low; i <= high; i++) {
		// bottom side
		//DisableEdge(i, low, i, low - 1);
		// top side
		//DisableEdge(i, high, i, high + 1);
		// left side
		//DisableEdge(low, i, low - 1, i);
		// right side
		//DisableEdge(high, i, high + 1, i);
		for (int j = low; j <= high; j++) {
			cellInfo[ID(i, j)] &= ~CellInfo::FLUID;
		}
	}

	CalcBorderFromContent();

	if (useVort) VortConfinement();

	CallUpdates(dt, FluidUpdate::VEL);
	(this->*pVelStep)(dt);
	CallUpdates(dt, FluidUpdate::DENS);
	(this->*pDensStep)(dt);
}

void FluidSystem::ToggleVert()
{
	useVort = !useVort;
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

void FluidSystem::AddCollider(FluidCollider* pfc)
{
	gColliders.push_back(pfc);
}

void FluidSystem::Clear()
{
	vX.SetZero();
	vY.SetZero();
	dens.SetZero();
}

void FluidSystem::ClearUpdaters()
{
	for (auto& pU : gUpdaters) {
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

void FluidSystem::AddFluid(int x, int y, real d, real velx, real vely, bool speedModify)
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

real FluidSystem::TotalInnerMass(bool curr)
{
	int i, j;
	real result = 0;

	list d = dens.Curr();
	if (!curr) d = dens.Prev();

	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			result += d[ID(i, j)];
		}
	}
	return result;
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

void FluidSystem::DensStep1(real dt)
{
	AddArrDt(N, dens.Curr(), dens.Prev(), dt);
	dens.Swap();
	Diffuse1(N, 0, dens.Curr(), dens.Prev(), diff, dt);
	dens.Swap();
	Advect1(N, 0, dens.Curr(), dens.Prev(), vX.Curr(), vY.Curr(), dt);
}

void FluidSystem::DensStep1B(real dt)
{
	AddArrDt(N, dens.Curr(), dens.Prev(), dt);
	dens.Swap();
	Diffuse1B(N, 0, dens.Curr(), dens.Prev(), diff, dt);
	dens.Swap();
	Advect1B(N, 0, dens.Curr(), dens.Prev(), vX.Curr(), vY.Curr(), dt);
}

void FluidSystem::DensStep2(real dt)
{
	AddArrDt(N, dens.Curr(), dens.Prev(), dt);
	dens.Swap();
	Diffuse2(N, 0, dens.Curr(), dens.Prev(), diff, dt);
	dens.Swap();
	//std::cout << "\nA : MASS " << TotalInnerMass(false) << "\n";
	Advect2(N, 0, dens.Curr(), dens.Prev(), vX.Curr(), vY.Curr(), dt);
	//std::cout << "B : MASS " << TotalInnerMass(true) << "\n";
}

void FluidSystem::DensStep2B(real dt)
{
	AddArrDt(N, dens.Curr(), dens.Prev(), dt);
	dens.Swap();
	Diffuse2B(N, 0, dens.Curr(), dens.Prev(), diff, dt);
	dens.Swap();
	Advect2B(N, 0, dens.Curr(), dens.Prev(), vX.Curr(), vY.Curr(), dt);
}

void FluidSystem::DensStep3(real dt)
{
	AddArrDt(N, dens.Curr(), dens.Prev(), dt);
	dens.Swap();
	Diffuse3(N, 0, dens.Curr(), dens.Prev(), diff, dt);
	dens.Swap();
	//std::cout << "\nA : MASS " << TotalInnerMass(false) << "\n";
	Advect3(N, 0, dens.Curr(), dens.Prev(), vX.Curr(), vY.Curr(), dt);
	//std::cout << "B : MASS " << TotalInnerMass(true) << "\n";
}

void FluidSystem::VelStep1(real dt)
{
	AddArrDt(N, vX.Curr(), vY.Prev(), dt);
	AddArrDt(N, vY.Curr(), vY.Prev(), dt);

	vX.Swap();

	Diffuse1(N, 1, vX.Curr(), vX.Prev(), visc, dt);

	vY.Swap();

	Diffuse1(N, 2, vY.Curr(), vY.Prev(), visc, dt);

	Project(N, vX.Curr(), vY.Curr(), vX.Prev(), vY.Prev());

	vX.Swap();
	vY.Swap();

	Advect1(N, 1, vX.Curr(), vX.Prev(), vX.Prev(), vY.Prev(), dt);
	Advect1(N, 2, vY.Curr(), vY.Prev(), vX.Prev(), vY.Prev(), dt);

	Project(N, vX.Curr(), vY.Curr(), vX.Prev(), vY.Prev());
}

void FluidSystem::VelStep1B(real dt)
{
	AddArrDt(N, vX.Curr(), vY.Prev(), dt);
	AddArrDt(N, vY.Curr(), vY.Prev(), dt);

	vX.Swap();

	Diffuse1B(N, 1, vX.Curr(), vX.Prev(), visc, dt);

	vY.Swap();

	Diffuse1B(N, 2, vY.Curr(), vY.Prev(), visc, dt);
	ProjectB(N, vX.Curr(), vY.Curr(), vX.Prev(), vY.Prev());

	vX.Swap();
	vY.Swap();

	Advect1B(N, 1, vX.Curr(), vX.Prev(), vX.Prev(), vY.Prev(), dt);
	Advect1B(N, 2, vY.Curr(), vY.Prev(), vX.Prev(), vY.Prev(), dt);
	ProjectB(N, vX.Curr(), vY.Curr(), vX.Prev(), vY.Prev());
}

void FluidSystem::AddArrDt(int N, list x, list s, real dt)
{
	int i, size = (N + 2)*(N + 2);
	for (i = 0; i < size; i++) x[i] += dt*s[i];
}

void FluidSystem::ZeroBorder(int N, list x)
{
	for (int i = 0; i <= N + 2; i++) {
		x[ID(i, 0)] = 0;
		x[ID(i, N + 1)] = 0;
		x[ID(0, i)] = 0;
		x[ID(N + 1, i)] = 0;
	}
}

void FluidSystem::SetBorder(int N, int b, list x)
{
	int i;

	// sides
	if (b == 0) {
		for (i = 1; i <= N; i++) {
			x[ID(0, i)] = x[ID(1, i)];
			x[ID(N + 1, i)] = x[ID(N, i)];
			x[ID(i, 0)] = x[ID(i, 1)];
			x[ID(i, N + 1)] = x[ID(i, N)];
		}
	}
	else if (b == 1) {
		for (i = 1; i <= N; i++) {
			x[ID(0, i)] = -x[ID(1, i)];
			x[ID(N + 1, i)] = -x[ID(N, i)];
			x[ID(i, 0)] = x[ID(i, 1)];
			x[ID(i, N + 1)] = x[ID(i, N)];
		}
	}
	else if (b == 2) {
		for (i = 1; i <= N; i++) {
			x[ID(0, i)] = x[ID(1, i)];
			x[ID(N + 1, i)] = x[ID(N, i)];
			x[ID(i, 0)] = -x[ID(i, 1)];
			x[ID(i, N + 1)] = -x[ID(i, N)];
		}
	}

	// corners
	x[ID(0, 0)] = 0.5f*(x[ID(1, 0)] + x[ID(0, 1)]);
	x[ID(0, N + 1)] = 0.5f*(x[ID(1, N + 1)] + x[ID(0, N)]);
	x[ID(N + 1, 0)] = 0.5f*(x[ID(N, 0)] + x[ID(N + 1, 1)]);
	x[ID(N + 1, N + 1)] = 0.5f*(x[ID(N, N + 1)] + x[ID(N + 1, N)]);
}

void FluidSystem::PushBorderInward(int N, list x)
{
	int i;

	// sides
	for (i = 1; i <= N; i++) {
		x[ID(1, i)] += x[ID(0, i)];
		x[ID(0, i)] = 0;

		x[ID(N, i)] += x[ID(N + 1, i)];
		x[ID(N + 1, i)] = 0;

		x[ID(i, 1)] += x[ID(i, 0)];
		x[ID(i, 0)] = 0;

		x[ID(i, N)] += x[ID(i, N + 1)];
		x[ID(i, N + 1)] = 0;
	}

	x[ID(1, 1)] += x[ID(0, 0)];
	x[ID(0, 0)] = 0;

	x[ID(N, 1)] += x[ID(N + 1, 0)];
	x[ID(N + 1, 0)] = 0;

	x[ID(1, N)] += x[ID(0, N + 1)];
	x[ID(0, N + 1)] = 0;

	x[ID(N, N)] += x[ID(N + 1, N + 1)];
	x[ID(N + 1, N + 1)] = 0;
}

void FluidSystem::LinearSolve1(int N, int b, list x, list x0, real a, real c)
{
	int i, j, k;

	int limit = a > 0 ? steps : 1;

	for (k = 0; k < limit; k++) {
		for (j = 1; j <= N; j++) {
			for (i = 1; i <= N; i++) {
				x[ID(i, j)] = (x0[ID(i, j)] + a*(x[ID(i - 1, j)] + x[ID(i + 1, j)] + x[ID(i, j - 1)] + x[ID(i, j + 1)])) / c;
			}
		}
		SetBorder(N, b, x);
	}
}

void FluidSystem::LinearSolve1B(int N, int b, list x, list x0, real a, real c)
{
	int i, j, k;

	int limit = a > 0 ? steps : 1;

	// scale my own value based on b if neighbour is unreachable
	real bScX = (b == 1) ? -1 : 1;
	real bScY = (b == 2) ? -1 : 1;

	for (k = 0; k < limit; k++) {
		for (j = 1; j <= N; j++) {
			for (i = 1; i <= N; i++) {
				int id = ID(i, j);
				auto info = cellInfo[id];

				if (info & FLUID) {
					real my_val = x[id];
					real add_value = (info & LEFT) ? x[ID(i - 1, j)] : (bScX * my_val); // (bScX * bx[ID(i, j)]);
					add_value += (info & RIGHT) ? x[ID(i + 1, j)] : (bScX * my_val); //(bScX * bx[ID(i, j)]);
					add_value += (info & BOTTOM) ? x[ID(i, j - 1)] : (bScY * my_val); //(bScY * bx[ID(i, j)]);
					add_value += (info & TOP) ? x[ID(i, j + 1)] : (bScY * my_val); //(bScY * bx[ID(i, j)]);

					x[id] = (x0[id] + a * add_value) / c;
				}
			}
		}
	}
}

void FluidSystem::LinearSolve2(int N, int b, list x, list x0, real a, real c)
{
	int i, j, k;

	int limit = a > 0 ? steps : 1;
	limit += ((limit % 2 == 0) ? 1 : 0);

	for (k = 0; k < limit; k++) {
		if (k > 0) std::swap(x, x0);

		SetBorder(N, b, x0); // copy values to the border to conserve mass in the next loop

		for (j = 1; j <= N; j++) {
			for (i = 1; i <= N; i++) {
				int id = ID(i, j);
				x[id] = (x0[id] + a*(x0[id - 1] + x0[id + 1] + x0[id - N - 2] + x0[id + N + 2])) / c;
			}
		}
	}

	ZeroBorder(N, x);
}

void FluidSystem::LinearSolve3(int N, int b, list x, list x0, real dt)
{
	// scales with density
	int i, j, k;

	int limit = diff > 0 ? steps : 1;
	limit += ((limit % 2 == 0) ? 1 : 0);

	for (k = 0; k < limit; k++) {
		SetBorder(N, b, x0);
		// TODO: account for loss at edges with correct border

		for (j = 1; j <= N; j++) {
			for (i = 1; i <= N; i++) {
				int id = ID(i, j);
				real d = x0[id] + 1;
				d = d*d*d*d;
				d *= d * d;
				real a = d * diff * N * N;
				x[id] = (x0[id] + a*(x0[id - 1] + x0[id + 1] + x0[id - N - 2] + x0[id + N + 2])) / (4 * a + 1);
				//x[ID(i, j)] = (x0[ID(i, j)] + a*(x[ID(i - 1, j)] + x[ID(i + 1, j)] + x[ID(i, j - 1)] + x[ID(i, j + 1)])) / c;
			}
		}

		std::swap(x, x0);
	}

}

void FluidSystem::Diffuse1(int N, int b, list x, list x0, real diff, real dt)
{
	real a = dt*diff*N*N;
	LinearSolve1(N, b, x, x0, a, 1 + 4 * a);
}

void FluidSystem::Diffuse1B(int N, int b, list x, list x0, real diff, real dt)
{
	real a = dt*diff*N*N;
	LinearSolve1B(N, b, x, x0, a, 1 + 4 * a);
}

void FluidSystem::Diffuse2(int N, int b, list x, list x0, real diff, real dt)
{
	real a = dt*diff*N*N;
	LinearSolve2(N, b, x, x0, a, 1 + 4 * a);
}

void FluidSystem::Diffuse2B(int N, int b, list x, list x0, real diff, real dt)
{
	// TODO
}

void FluidSystem::Diffuse3(int N, int b, list x, list x0, real diff, real dt)
{
	LinearSolve3(N, b, x, x0, dt);
}

void FluidSystem::Advect1(int N, int b, list d, list d0, list u, list v, real dt)
{
	int i, j;

	real dtN = dt*N;
	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			MidpointBackward(N, i, j, dtN, u, v, d, d0);
		}
	}

	SetBorder(N, b, d);
}

void FluidSystem::Advect1B(int N, int b, list d, list d0, list u, list v, real dt)
{
	int i, j;
	real dtN = dt*N;

	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			MidpointBackward(N, i, j, dtN, u, v, d, d0);
		}
	}
}

void FluidSystem::Advect2(int N, int b, list d, list d0, list u, list v, real dt)
{
	// forward pushing version
	int i, j;// , i0, j0, i1, j1;

	Tools::Fill(d, (real) 0);

	real dtN = dt*N;
	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			MidpointForward(N, i, j, dtN, u, v, d, d0);
		}
	}

	PushBorderInward(N, d);
}

void FluidSystem::Advect2B(int N, int b, list d, list d0, list u, list v, real dt)
{
	// TODO
}

void FluidSystem::Advect3(int N, int b, list d, list d0, list u, list v, real dt)
{
	// combine forward and backward methods

	// first do backwards methods, but only count usage per cell
	// do actual backwards method, divide sell usage by total usage during looking back
	// for all the cells with density > 0 and usage == 0, do forward step
	int i, j;// , i0, j0, i1, j1;
	real newX, newY, dt0;// , s0, t0, s1, t1;
	list newRatio = buffer1;
	Tools::Fill<real>(newRatio, 0);

	dt0 = dt*N;
	BilinearCoeffs s;

	// assemble counts
	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			newX = i - dt0 * u[ID(i, j)];
			newY = j - dt0 * v[ID(i, j)];

			s.calc(N, newX, newY);

			// update density usages
			newRatio[ID(s.i0, s.j0)] += s.s0*s.t0;
			newRatio[ID(s.i0, s.j1)] += s.s0*s.t1;
			newRatio[ID(s.i1, s.j0)] += s.s1*s.t0;
			newRatio[ID(s.i1, s.j1)] += s.s1*s.t1;
		}
	}


	PushBorderInward(N, d);

	// do original / count
	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			newX = i - dt0 * u[ID(i, j)];
			newY = j - dt0 * v[ID(i, j)];

			s.calc(N, newX, newY);

			real myVal = d0[ID(i, j)];
			int id00 = ID(s.i0, s.j0);
			int id01 = ID(s.i0, s.j1);
			int id10 = ID(s.i1, s.j0);
			int id11 = ID(s.i1, s.j1);

			d[ID(i, j)] =
				s.s0 * (
				((newRatio[id00] > 1) ? (s.t0 * d0[id00] / newRatio[id00]) : (s.t0 * d0[id00])) +
					((newRatio[id01] > 1) ? (s.t1 * d0[id01] / newRatio[id01]) : (s.t1 * d0[id01]))
					) + s.s1 * (
					((newRatio[id10] > 1) ? (s.t0 * d0[id10] / newRatio[id10]) : (s.t0 * d0[id10])) +
						((newRatio[id11] > 1) ? (s.t1 * d0[id11] / newRatio[id11]) : (s.t1 * d0[id11]))
						);
		}
	}

	PushBorderInward(N, d);

	// update count = 0
	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			int id = ID(i, j);
			if (newRatio[id] < 1) {
				real r = 1 - newRatio[id];
				newX = i + dt0 * Tools::Math::ScaleSqrt(u[ID(i, j)]);
				newY = j + dt0 * Tools::Math::ScaleSqrt(v[ID(i, j)]);

				s.calc(N, newX, newY);

				real myVal_sc = d0[ID(i, j)] * r;
				d[ID(s.i0, s.j0)] += s.s0 * s.t0 * myVal_sc;
				d[ID(s.i0, s.j1)] += s.s0 * s.t1 * myVal_sc;
				d[ID(s.i1, s.j0)] += s.s1 * s.t0 * myVal_sc;
				d[ID(s.i1, s.j1)] += s.s1 * s.t1 * myVal_sc;
			}
		}
	}

	PushBorderInward(N, d);
}

void FluidSystem::Project(int N, list u, list v, list p, list div)
{
	int i, j;

	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			div[ID(i, j)] = -0.5f * (u[ID(i + 1, j)] - u[ID(i - 1, j)] + v[ID(i, j + 1)] - v[ID(i, j - 1)]) / N;;
		}
	}

	// The code doenst need a set border here...
	// SetBorder(N, 0, div);
	Tools::Fill<real>(p, 0);

	LinearSolve1(N, 0, p, div, 1, 4);

	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			u[ID(i, j)] -= 0.5f * N * (p[ID(i + 1, j)] - p[ID(i - 1, j)]);
			v[ID(i, j)] -= 0.5f * N * (p[ID(i, j + 1)] - p[ID(i, j - 1)]);
		}
	}

	SetBorder(N, 1, u);
	SetBorder(N, 2, v);
}

void FluidSystem::ProjectB(int N, list u, list v, list p, list div)
{
	int i, j;

	// gradient function:
	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			int id = ID(i, j);

			real val = 0;
			byte info = cellInfo[id];

			val += (info & RIGHT) ? u[ID(i + 1, j)] : -u[id];
			val -= (info & LEFT) ? u[ID(i - 1, j)] : -u[id];
			val += (info & TOP) ? v[ID(i, j + 1)] : -v[id];
			val -= (info & BOTTOM) ? v[ID(i, j - 1)] : -v[id];

			div[id] = -0.5f * val / N;
			//div[ID(i, j)] = -0.5f * (u[ID(i + 1, j)] - u[ID(i - 1, j)] + v[ID(i, j + 1)] - v[ID(i, j - 1)]) / N;;
		}
	}

	Tools::Fill<real>(p, 0);

	LinearSolve1B(N, 0, p, div, 1, 4);

	for (j = 1; j <= N; j++) {
		for (i = 1; i <= N; i++) {
			byte info = cellInfo[ID(i, j)];
			real m = p[ID(i, j)];

			real l = (info & LEFT) ? p[ID(i - 1, j)] : m;
			real r = (info & RIGHT) ? p[ID(i + 1, j)] : m;
			real t = (info & TOP) ? p[ID(i, j + 1)] : m;
			real b = (info & BOTTOM) ? p[ID(i, j - 1)] : m;
			
			u[ID(i, j)] -= 0.5f * N * (r - l);
			v[ID(i, j)] -= 0.5f * N * (t - b);
		}
	}
}

void FluidSystem::VortConfinement()
{
	std::cout << "\n USE VORT \n";

	for (int i = 1; i <= N; i++)
	{
		for (int j = 1; j <= N; j++)
		{
			vort[ID(i, j)] = 0.5f * (vY.Curr()[ID(i + 1, j)] - vY.Curr()[ID(i - 1, j)] - vX.Curr()[ID(i, j + 1)] + vX.Curr()[ID(i, j - 1)]);
			if (vort[ID(i, j)] >= 0.0f) absVort[ID(i, j)] = vort[ID(i, j)];
			else absVort[ID(i, j)] = -vort[ID(i, j)];
		}
	}

	SetBorder(N, 0, vort);
	SetBorder(N, 0, absVort);

	for (int i = 1; i <= N; i++)
	{
		for (int j = 1; j <= N; j++)
		{
			gradVortX[ID(i, j)] = 0.5f*(absVort[ID(i + 1, j)] - absVort[ID(i - 1, j)]);
			gradVortY[ID(i, j)] = 0.5f*(absVort[ID(i, j + 1)] - absVort[ID(i, j - 1)]);

			lenGrad[ID(i, j)] = sqrt(gradVortX[ID(i, j)] * gradVortX[ID(i, j)] + gradVortY[ID(i, j)] * gradVortY[ID(i, j)]);

			if (lenGrad[ID(i, j)] < 0.01f)
			{
				vcfx[ID(i, j)] = 0.0f;
				vcfy[ID(i, j)] = 0.0f;
			}
			else
			{
				vcfx[ID(i, j)] = gradVortX[ID(i, j)] / lenGrad[ID(i, j)];
				vcfy[ID(i, j)] = gradVortY[ID(i, j)] / lenGrad[ID(i, j)];
			}
		}
	}

	SetBorder(N, 0, vcfx);
	SetBorder(N, 0, vcfy);

	for (int i = 1; i <= N; i++)
	{
		for (int j = 1; j <= N; j++)
		{
			vX.Curr()[ID(i, j)] += vorticity * (vcfy[ID(i, j)] * vort[ID(i, j)]);
			vY.Curr()[ID(i, j)] += vorticity * (-vcfx[ID(i, j)] * vort[ID(i, j)]);
		}
	}

	SetBorder(N, 0, vX.Curr());
	SetBorder(N, 0, vY.Curr());
}

void FluidSystem::CalcBorderFromContent()
{
	// for all cells, the 4 borders must be set
	// TODO: border needed?
	for (int i = 1; i <= N; i++)
	{
		for (int j = 1; j <= N; j++)
		{
			// sides:
			byte l = cellInfo[ID(i - 1, j)];
			byte r = cellInfo[ID(i + 1, j)];
			byte t = cellInfo[ID(i, j + 1)];
			byte b = cellInfo[ID(i, j - 1)];

			// myself
			byte m = cellInfo[ID(i, j)];

			bool mIsFluid = (m & FLUID);
			// write cell info - equal to neighbour type?
			m = mIsFluid * FLUID;
			m |= ((!!(l & FLUID)) == mIsFluid) * LEFT;
			m |= ((!!(r & FLUID)) == mIsFluid) * RIGHT;
			m |= ((!!(t & FLUID)) == mIsFluid) * TOP;
			m |= ((!!(b & FLUID)) == mIsFluid) * BOTTOM;
			cellInfo[ID(i, j)] = m;
		}
	}
}

void FluidSystem::ResetCellInfo()
{
	Tools::Fill<byte>(cellInfo, CellInfo::NORMAL_CELL);

	// set edges as non fluid
	for (int i = 1; i <= N; i++) {
		cellInfo[ID(i, 0)] &= ~CellInfo::FLUID;
		cellInfo[ID(i, N + 1)] &= ~CellInfo::FLUID;
		cellInfo[ID(0, i)] &= ~CellInfo::FLUID;
		cellInfo[ID(N + 1, i)] &= ~CellInfo::FLUID;
	}

	// set corners as non fluid
	cellInfo[ID(0, 0)] &= ~CellInfo::FLUID;
	cellInfo[ID(0, N + 1)] &= ~CellInfo::FLUID;
	cellInfo[ID(N + 1, 0)] &= ~CellInfo::FLUID;
	cellInfo[ID(N + 1, N + 1)] &= ~CellInfo::FLUID;
}

void FluidSystem::CallColliders()
{
	for (auto pfC : gColliders) {
		pfC->FillGrid(N, cellInfo);
	}
}

BilinearCoeffs FluidSystem::RayTrace(int N, int x0, int y0, real x1, real y1, list grid)
{
	if (x1 < 0.5f) x1 = 0.5f;
	if (x1 > N + 0.5f) x1 = N + 0.5f;
	if (y1 < 0.5f) y1 = 0.5f;
	if (y1 > N + 0.5f) y1 = N + 0.5f;

	real dx = fabs(x1 - x0);
	real dy = fabs(y1 - y0);

	int x = int(floor(x0));
	int y = int(floor(y0));

	int n = 1;
	int x_inc, y_inc;
	real error;

	if (dx == 0)
	{
		x_inc = 0;
		error = std::numeric_limits<real>::infinity();
	}
	else if (x1 > x0)
	{
		x_inc = 1;
		n += int(floorf(x1)) - x;
		error = (floorf(x0) + 1 - x0) * dy;
	}
	else
	{
		x_inc = -1;
		n += x - int(floorf(x1));
		error = (x0 - floorf(x0)) * dy;
	}

	if (dy == 0)
	{
		y_inc = 0;
		error -= std::numeric_limits<real>::infinity();
	}
	else if (y1 > y0)
	{
		y_inc = 1;
		n += int(floor(y1)) - y;
		error -= (floor(y0) + 1 - y0) * dx;
	}
	else
	{
		y_inc = -1;
		n += y - int(floor(y1));
		error -= (y0 - floor(y0)) * dx;
	}

	BilinearCoeffs result;
	int pX = x0, pY = y0;
	for (; n > 0; --n)
	{
		// if can go from (pX, pY) to (x,y)
		// continue
		// if not
		// return px, py

		if (!CanMoveNeighbour(pX, pY, x, y)) {
			x = pX; y = pY;
			break;
		}

		if (error > 0)
		{
			pY = y;
			y += y_inc;
			error -= dx;
		}
		else
		{
			pX = x;
			x += x_inc;
			error += dy;
		}
	}

	// x and y are the last valid coordinate

	// project x, y onto x0 x1 y0 y1
	Vec2 s = Vec2(x1 - x0, y1 - y0);
	Vec2 v = Vec2(x - x0, y - y0);
	Vec2 newpoint = s * s.dot(v) / s.dot(s);
	newpoint += Vec2(x0, y0);
	BilinearCoeffs bilin;
	bilin.calc(N, newpoint(0), newpoint(1));


	return bilin;
}

bool FluidSystem::CanMoveNeighbour(int x0, int y0, int x1, int y1)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	auto info = cellInfo[ID(x0, y0)];
	if (dx > 0) {
		return info & RIGHT;
	}
	else if (dx < 0) {
		return info & LEFT;
	}
	else if (dy > 0) {
		return info & TOP;
	}
	else if (dy < 0) {
		return info & BOTTOM;
	}
	return true;
}

void FluidSystem::DisableEdge(int x0, int y0, int x1, int y1)
{
	int id0 = ID(x0, y0);
	int id1 = ID(x1, y1);
	if (x1 > x0) {
		cellInfo[id0] &= ~RIGHT;
		cellInfo[id1] &= ~LEFT;
	}
	else if (x0 > x1) {
		cellInfo[id0] &= ~LEFT;
		cellInfo[id1] &= ~RIGHT;
	}
	else if (y1 > y0) {
		cellInfo[id0] &= ~TOP;
		cellInfo[id1] &= ~BOTTOM;
	}
	else if (y0 > y1) {
		cellInfo[id0] &= ~BOTTOM;
		cellInfo[id1] &= ~TOP;
	}
}
