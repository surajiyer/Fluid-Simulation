#pragma once
#include "Inc.h"

struct FluidUpdate {
	enum Type {
		VEL, DENS
	};
	FluidSystem* fs;
	real dt;
	Type type;
};

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

	void AddFluid(int x, int y, float d, float vx, float vy, bool speedModify = true);

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

	void DensStep(float dt);
	void VelStep(float dt);
	void AddArrDt(int N, list x, list s, float dt);
	void SetBorder(int N, int b, list x);
	void LinearSolve(int N, int b, list x, list x0, float a, float c);
	void Diffuse(int N, int b, list x, list x0, float diff, float dt);
	void Advect(int N, int b, list d, list d0, list u, list v, float dt);
	void Project(int N, list u, list v, list p, list div);

public:
	int N;
	real diff, visc;

	FlipFlopArr<real> vX;
	FlipFlopArr<real> vY;
	FlipFlopArr<real> dens;

	std::vector<Tools::UpdatableR<bool, FluidUpdate>*> gUpdaters;
};
