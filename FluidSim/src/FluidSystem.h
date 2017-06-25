#pragma once
#include "Inc.h"

class FluidSystem
{
public:
	FluidSystem();
	~FluidSystem();

	void Setup(Tools::Surface2D<uint32_t>);
	void SetDensity(int x, int y, real v);
	void Update(real dt);

	Tools::Surface2D<uint32_t> GetSize();
	std::vector<real>& GetDensities();

	std::vector<real>& GetVelX();
	std::vector<real>& GetVelY();

	void SetZeroVelocity();

private:

	using list = std::vector<real>&;

	inline int ID(int ix, int iy) {
		return ix + iy * gTrueSurface.width;
	}

	// translations
	inline int IX(int a, int b) {
		return ID(a, b);
	}
	inline void set_bnd(int N, int b, list x) {
		SetBorder(b, x);
	}
	inline void add_source(int N, list x, list s, float dt) {
		AddArrayDt(x, s, dt);
	}
	inline void lin_solve(int N, int b, list x, list x0, float a, float c) {
		LinearSolve(b, x, x0, a, c);
	}
	inline void diffuse(int N, int b, list x, list x0, float diff, float dt)
	{
		Diffuse(b, x, x0, diff, dt);
	}
	inline void advect(int N, int b, list d, list d0, list u, list v, float dt) {
		Advect(b, d, d0, u, v, dt);
	}
	inline void project(int N, list u, list v, list p, list div)
	{
		Project(u, v, p, div);
	}
	inline list u() {
		return gVelY.Curr();
	}
	inline list v() {
		return gVelX.Curr();
	}
	inline list u0() {
		return gVelY.Prev();
	}
	inline list v0() {
		return gVelX.Prev();
	}
	inline list x() {
		return gDens.Curr();
	}
	inline list x0() {
		return gDens.Prev();
	}
	inline real visc() {
		return gViscosity;
	}

	void DensityStep(real dt);
	void VelocityStep(real dt);
	void ForcesStep(real dt);
	void DampStep(real dt);

	void Diffuse(int b, list x, list x0, real strength, real dt);
	void Advect(int b, list d, list d0, list u, list v, real dt);

	void LinearSolve(int b, list x, list x0, real a, real c);
	void SetBorder(int b, list x);
	void Project(list u, list v, list p, list div);
	void AddArrayDt(list x, list s, real dt);

	enum NeighbourFlags : byte {
		TL = 1, T = 2, TR = 4,
		L = 8,		/* cell */	R = 16,
		BL = 32, B = 64, BR = 128
	};

	template<class S>
	struct FlipFlop {
		S							gStates[2];
		uint8_t						gPrevID = 0;
		uint8_t						gCurrID = 1;

		inline list Curr() {
			return gStates[gCurrID];
		}

		inline list Prev() {
			return gStates[gPrevID];
		}

		inline void Swap() {
			auto temp = gPrevID;
			gPrevID = gCurrID;
			gCurrID = temp; // gPrevID ^ 1; // xor
		}
	};

	// metadata
	Tools::Surface2D<uint32_t>		gTrueSurface;
	int								gTrueCellCount;
	int								gNi; // inner cell count - without border
	int								gInnerCellCount;

	// parameters
	real							gDiffuse;
	real							gViscosity;

	// data
	FlipFlop<std::vector<real>>		gDens;
	FlipFlop<std::vector<real>>		gVelY; // u
	FlipFlop<std::vector<real>>		gVelX; // v
	std::vector<NeighbourFlags>		gConnections;
};
