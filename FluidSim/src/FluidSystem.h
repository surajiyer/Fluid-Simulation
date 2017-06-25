#pragma once
#include "Inc.h"

class FluidSystem
{
	using list = std::vector<real>&;

public:
	FluidSystem();
	~FluidSystem();

	void Setup(int N, float diff, float visc);
	void Update(real dt);

	void SetDensity(int x, int y, real v);
	void SetVelocity(int x, int y, real vx, real vy);

	void AddUpdater(Tools::UpdatableR<bool, FluidSystem*>*);
	void RemoveUpdater(Tools::UpdatableR<bool, FluidSystem*>*, bool to_delete);

	void Clear();
	void ClearUpdaters();

	Tools::Surface2D<int> GetSize();
	list GetDensities();
	list GetVelX();
	list GetVelY();

	real& Density(int x, int y);
	real& VelX(int x, int y);
	real& VelY(int x, int y);

	void AddFluid(int x, int y, float d, float vx, float vy, bool speedModify = true);

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
			gCurrID = gCurrID ^ 1; // xor
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

	void DensStep(int N, float * x, float * x0, float * u, float * v, float diff, float dt);
	void VelStep(int N, float * u, float * v, float * u0, float * v0, float visc, float dt);
	void AddArrDt(int N, float * x, float * s, float dt);
	void SetBorder(int N, int b, float * x);
	void LinearSolve(int N, int b, float * x, float * x0, float a, float c);
	void Diffuse(int N, int b, float * x, float * x0, float diff, float dt);
	void Advect(int N, int b, float * d, float * d0, float * u, float * v, float dt);
	void Project(int N, float * u, float * v, float * p, float * div);

public:
	int N;
	real diff, visc;

	FlipFlopArr<real> velX;
	FlipFlopArr<real> velY;
	FlipFlopArr<real> dens;

	std::vector<Tools::UpdatableR<bool, FluidSystem*>*> gUpdaters;
};
