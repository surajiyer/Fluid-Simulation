#pragma once
#include "Inc.h"
#include "FluidCode.h"

class FluidSystem
{
	typedef void (FluidSystem::*FSFuncDT)(real);

public:

	enum class FSType {
		ORIGINAL, // complete
		ORIGINAL_BORDERED,
		ORIGINAL_BORDERED_MF, // multi fluid
		V2, // works -> could use density pressue forces
		V2_BORDERED,  // forward step disrespects borders
		HYBRID
	};

	FluidSystem();
	~FluidSystem();

	void Setup(int N, real vorticity, std::vector<FluidProps>&, FSType);
	void SetType(FSType);
	void Update(real dt);

	void ToggleVort();

	inline void SetDensity(int x, int y, real v, int dn);
	inline void SetDensity(int x, int y, real v);
	void SetVelocity(int x, int y, real vx, real vy);
	void SetObjectVelocity(int x, int y, real vx, real vy);

	void AddUpdater(Tools::UpdatableR<bool, FluidUpdate>*);
	void RemoveUpdater(Tools::UpdatableR<bool, FluidUpdate>*, bool to_delete);

	void AddCollider(FluidCollider*);
	void AddBorder(FluidBorder*);

	void Clear();
	void ClearUpdaters();

	Tools::Surface2D<int> GetSize();
	int GetN_inner();
	std::vector<FlipFlopArr<real>>& GetDensities();
	std::vector<FluidProps>& GetProperties();
	std::vector<byte>& GetInfo();
	list GetVelX();
	list GetVelY();
	byte CellInfo(int i, int j);
	std::vector<FluidCollider*>& GetObjects();

	real& Density(int x, int y, int dn);
	real CombinedDensity(int x, int y);
	real& VelX(int x, int y);
	real& VelY(int x, int y);

	void DisableEdge(int x0, int y0, int x1, int y1);

	void AddFluid(int x, int y, real d, real vx, real vy, int dn);
	real TotalInnerMass(bool curr = true);

 	__forceinline int ID(int i, int j) {
		return i + (N + 2) * j;
	}

	__forceinline Tools::Point2D<int> rID(int i) {
		return {i % (N + 2), (int)(i / (N + 2)) };
	}

private:

	void CallUpdates(real dt, FluidUpdate::Type);

	void DensStep1(real dt);	// basic density step, but diffusing is replace by blurring/spreading density
	void DensStep1BS(real dt);	// + respects border + respects multi fluid diffusion

	void DensStep2(real dt);	// just test versions
	void DensStep2B(real dt);	//
	void DensStep3(real dt);	//

	void VelStep1(real dt);		// basic velocity field
	void VelStep1B(real dt);	// + respects border
	void VelStep1BS(real dt);	// + respects multi fluid viscosity + air has 0 viscosity
	void VelStep1BSO(real dt);	// + respect object movement

	void AddArrDt(int N, list x, list s, real dt);

	void ZeroBorder(int N, list x);
	void SetBorder(int N, int b, list x);
	void PushBorderInward(int N, list x);

	void LinearSolve1(int N, int b, list x, list x0, real a, real c);					// basic solve
	void LinearSolve1B(int N, int b, list x, list x0, real a, real c);					// + supports border
	void LinearSolve1BO(int N, int b, list x, list x0, real a, real c);					// + supports object movement
	void LinearSolve1BS(int N, int b, list x, list x0, real aSc, real a_fallback, FluidProps::FP type);  // + supports viscosity mixing (for velocity field)
	void LinearSolve1BSO(int N, int b, list x, list x0, real aSc, real a_fallback, FluidProps::FP type);
	void Blur(int N, int b, list x, list x0, real a, real c);							// alternate diffusion
	void BlurB(int N, int b, list x, list x0, real a, real c);							// alternate diffusion + border
	void LinearSolve3_not_updated(int N, int b, list x, list x0, real dt);

	void Diffuse1(int N, int b, list x, list x0, real coef, real dt);
	void Diffuse1B(int N, int b, list x, list x0, real coef, real dt);
	void Diffuse1BS(int N, int b, list x, list x0, FluidProps::FP fpid, real dt);
	void Diffuse1BSO(int N, int b, list x, list x0, FluidProps::FP fpid, real dt);
	void Diffuse_blur(int N, int b, list x, list x0, real coef, real dt);
	void Diffuse_blurB(int N, int b, list x, list x0, real coef, real dt);
	void Diffuse3(int N, int b, list x, list x0, real coef, real dt);

	void Advect1(int N, int b, list d, list d0, list u, list v, real dt);
	void Advect1B(int N, int b, list d, list d0, list u, list v, real dt);
	void Advect2(int N, int b, list d, list d0, list u, list v, real dt);
	void Advect2B(int N, int b, list d, list d0, list u, list v, real dt);
	void Advect3(int N, int b, list d, list d0, list u, list v, real dt);

	void Project(int N, list u, list v, list p, list div);		// basic project
	void ProjectB(int N, list u, list v, list p, list div);		// + supports border
	void ProjectBO(int N, list u, list v, list p, list div);	// + supports moving objects

	void VortConfinement();
	void Damp(list vel, float amount, float dt);
	
	BilinearCoeffs RayTrace(int N, int sx, int sy, real tx, real ty, list grid);

	// moves with velocity
	BilinearCoeffs BackTrace(real sx, real sy, real dt, list vX, list vY);

	inline void MidpointBackward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0);
	inline void EulerBack(int N, int x, int y, real dtN, list vX, list vY, list d, list d0);

	inline void EulerForward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0);
	inline void MidpointForward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0);

	// neighbour stuffs
	bool CanMoveNeighbour(int x0, int y0, int x1, int y1);
	void CalcBorderFromContent();
	void ResetCellInfo();
	void UpdateColliders_1(real dt);
	void UpdateColliders_2(real dt);
	void CallBorders();

public:
	int N;
	int steps;
	int fluid_count;

	std::vector<FluidProps> fprops;

	real visc_air = 0.001;
	real damp = 0.1;
	real visc_avg;
	real vorticity;

	FSFuncDT pDensStep;
	FSFuncDT pVelStep;

	FlipFlopArr<real> vX;
	FlipFlopArr<real> vY;
	std::vector<real> p;
	std::vector<FlipFlopArr<real>> densList;

	std::vector<real> buffer1;
	std::vector<real> buffer2;
	std::vector<real> buffer3;
	std::vector<real> buffer4;

	std::vector<real> vort;
	std::vector<real> absVort;
	std::vector<real> gradVortX;
	std::vector<real> gradVortY;
	std::vector<real> lenGrad;
	std::vector<real> vcfx;
	std::vector<real> vcfy;

	std::vector<byte> cellInfo;

	std::vector<Tools::UpdatableR<bool, FluidUpdate>*> gUpdaters;
	std::vector<FluidCollider*> gColliders;
	std::vector<FluidBorder*> gBorders;

	bool useVort = true;
};


void FluidSystem::MidpointBackward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0)
{
	int id = ID(x, y);
	BilinearCoeffs s;

	// backtrace
	real dirX = -dtN * vX[id];
	real dirY = -dtN * vY[id];

	// this gives a backwards direction, evaluate velocity halfway there
	real midX = x + 0.5f * dirX;
	real midY = y + 0.5f * dirY;
	s.calc(N, midX, midY);
	real vx = s.Eval(N + 2, vX);
	real vy = s.Eval(N + 2, vY);

	// update density sampling the midpoint-backwards location
	real newX = x - dtN * vx;
	real newY = y - dtN * vy;
	s.calc(N, newX, newY);
	d[id] = s.Eval(N + 2, d0);
}

void FluidSystem::EulerBack(int N, int x, int y, real dtN, list vX, list vY, list d, list d0)
{
	int id = ID(x, y);
	real newX = x - dtN * vX[id];
	real newY = y - dtN * vY[id];

	BilinearCoeffs s;
	s.calc(N, newX, newY);

	// update density with bilinear interpolation
	d[id] = s.Eval(N + 2, d0);
}

void FluidSystem::EulerForward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0)
{
	int id = ID(x, y);
	real newX = x + dtN * vX[id];
	real newY = y + dtN * vY[id];

	BilinearCoeffs s;
	s.calc(N, newX, newY);

	real myVal = d0[id];
	d[ID(s.i0, s.j0)] += s.s0*s.t0*myVal;
	d[ID(s.i0, s.j1)] += s.s0*s.t1*myVal;
	d[ID(s.i1, s.j0)] += s.s1*s.t0*myVal;
	d[ID(s.i1, s.j1)] += s.s1*s.t1*myVal;
}


void FluidSystem::MidpointForward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0)
{
	int id = ID(x, y);
	BilinearCoeffs s;

	// go half way
	real newX = x + 0.5f * dtN * vX[id];
	real newY = y + 0.5f * dtN * vY[id];

	// evaluage velocity half way
	s.calc(N, newX, newY);
	real vx = s.Eval(N + 2, vX);
	real vy = s.Eval(N + 2, vY);

	// evaluate new poisition
	newX = x + dtN * vx;
	newY = y + dtN * vy;
	s.calc(N, newX, newY);

	real myVal = d0[id];
	d[ID(s.i0, s.j0)] += s.s0*s.t0*myVal;
	d[ID(s.i0, s.j1)] += s.s0*s.t1*myVal;
	d[ID(s.i1, s.j0)] += s.s1*s.t0*myVal;
	d[ID(s.i1, s.j1)] += s.s1*s.t1*myVal;
}

void FluidSystem::SetDensity(int x, int y, real v)
{
	SetDensity(x, y, v, 0);
}

void FluidSystem::SetDensity(int x, int y, real v, int dn)
{
	densList[dn].Curr()[ID(x, y)] = v;
}
