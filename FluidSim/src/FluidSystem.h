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

	void SetZeroVelocity();

private:

	using list = std::vector<real>&;

	inline int ID(int ix, int iy) {
		return ix + iy * gTrueSurface.width;
	}

	void DensityStep(real dt);
	void VelocityStep(real dt);
	void ForcesStep(real dt);

	void Diffuse(int b, list x, list x0, real strength, real dt);
	void Advect(int b, list d, list d0, list u, list v, real dt);

	void LinearSolve(int b, list x, list x0, real a, real c);
	void SetBand(int b, list x);
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

		inline list Current() {
			return gStates[gCurrID];
		}

		inline list Previous() {
			return gStates[gPrevID];
		}

		inline void Swap() {
			gPrevID = gCurrID;
			gCurrID = gPrevID ^ 1; // xor
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
	FlipFlop<std::vector<real>>		gDensities;
	FlipFlop<std::vector<real>>		gVelU;
	FlipFlop<std::vector<real>>		gVelV;
	std::vector<NeighbourFlags>		gConnections;
};
