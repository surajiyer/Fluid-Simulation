#pragma once
#include "Inc.h"
#include "FluidCode.h"

class FluidSystem
{
	using list = std::vector<real>&;
	typedef void (FluidSystem::*FSFuncDT)(real);

public:

	enum class FSType {
		ORIGINAL, // complete
		ORIGINAL_BORDERED, // doesnt respect borders properly
		V2, // works -> could use density pressue forces
		V2_BORDERED,  // forward step disrespects borders
		HYBRID
	};

	enum CellInfo : byte {
		// for each direction if there is fluid there:
		LEFT = 1,
		RIGHT = 2,
		TOP = 4,
		BOTTOM = 8,
		// if it is a fluid cell : otherwise its inside an object or the boundary
		FLUID = 16,
		// Combined value for optimalization of standard cell, all others are true
		NORMAL_CELL = 1 + 2 + 4 + 8 + 16
	};

	FluidSystem();
	~FluidSystem();

	void Setup(int N, real diff, real visc, real vort, FSType);
	void SetType(FSType);
	void Update(real dt);

	void ToggleVert();

	void SetDensity(int x, int y, real v);
	void SetVelocity(int x, int y, real vx, real vy);

	void AddUpdater(Tools::UpdatableR<bool, FluidUpdate>*);
	void RemoveUpdater(Tools::UpdatableR<bool, FluidUpdate>*, bool to_delete);

	void Clear();
	void ClearUpdaters();

	Tools::Surface2D<int> GetSize();
	int GetN_inner();
	list GetDensities();
	list GetVelX();
	list GetVelY();

	real& Density(int x, int y);
	real& VelX(int x, int y);
	real& VelY(int x, int y);

	void AddFluid(int x, int y, real d, real vx, real vy, bool speedModify = true);
	real TotalInnerMass(bool curr = true);

 	__forceinline int ID(int i, int j) {
		return i + (N + 2) * j;
	}

	__forceinline Tools::Point2D<int> rID(int i) {
		return {i % (N + 2), (int)(i / (N + 2)) };
	}

private:
	//typedef real S;
	template<class S>
	class FlipFlopArr {
	public:
		std::vector<S>				gStates[2];
		uint8_t						gPrevID = 0;
		uint8_t						gCurrID = 1;

		inline list Curr() {
			return gStates[gCurrID];
		}

		inline list Prev() {
			return gStates[gPrevID];
		}

		inline void Swap() {
			gPrevID = gCurrID;
			gCurrID ^= 1; // xor
		}

		inline void Resize(int n) {
			gStates[0].resize(n);
			gStates[1].resize(n);
		}

		inline void SetZero() {
			Tools::Fill<S>(gStates[0], 0);
			Tools::Fill<S>(gStates[1], 0);
		}
	};

	void CallUpdates(real dt, FluidUpdate::Type);

	void DensStep1(real dt);
	void DensStep1B(real dt);
	void DensStep2(real dt);
	void DensStep2B(real dt);
	void DensStep3(real dt);

	void VelStep1(real dt);
	void VelStep1B(real dt);

	void AddArrDt(int N, list x, list s, real dt);

	void ZeroBorder(int N, list x);
	void SetBorder(int N, int b, list x);
	void PushBorderInward(int N, list x);

	void LinearSolve1(int N, int b, list x, list x0, real a, real c);
	void LinearSolve1B(int N, int b, list x, list x0, real a, real c);
	void LinearSolve2(int N, int b, list x, list x0, real a, real c);
	void LinearSolve3(int N, int b, list x, list x0, real dt);

	void Diffuse1(int N, int b, list x, list x0, real diff, real dt);
	void Diffuse1B(int N, int b, list x, list x0, real diff, real dt);
	void Diffuse2(int N, int b, list x, list x0, real diff, real dt);
	void Diffuse2B(int N, int b, list x, list x0, real diff, real dt);
	void Diffuse3(int N, int b, list x, list x0, real diff, real dt);

	void Advect1(int N, int b, list d, list d0, list u, list v, real dt);
	void Advect1B(int N, int b, list d, list d0, list u, list v, real dt);
	void Advect2(int N, int b, list d, list d0, list u, list v, real dt);
	void Advect2B(int N, int b, list d, list d0, list u, list v, real dt);
	void Advect3(int N, int b, list d, list d0, list u, list v, real dt);

	void Project(int N, list u, list v, list p, list div);
	void ProjectB(int N, list u, list v, list p, list div);

	void VortConfinement();

	void CalcBorderFromContent();
	
	BilinearCoeffs RayTrace(int N, int sx, int sy, real tx, real ty, list grid);

	// moves with velocity
	BilinearCoeffs BackTrace(real sx, real sy, real dt, list vX, list vY);

	inline void MidpointBackward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0);
	inline void EulerBack(int N, int x, int y, real dtN, list vX, list vY, list d, list d0);

	inline void EulerForward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0);
	inline void MidpointForward(int N, int x, int y, real dtN, list vX, list vY, list d, list d0);

	bool CanMoveNeighbour(int x0, int y0, int x1, int y1);

	void DisableEdge(int x0, int y0, int x1, int y1);

public:
	int N;
	real diff, visc, vorticity;

	FSFuncDT pDensStep;
	FSFuncDT pVelStep;

	FlipFlopArr<real> vX;
	FlipFlopArr<real> vY;
	FlipFlopArr<real> dens;

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
