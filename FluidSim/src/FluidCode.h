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

struct BilinearCoeffs {
	real s0, s1, t0, t1;
	int i0, j0, i1, j1;

	inline real A(int size, std::vector<real>& d) {
		return s0*t0*d[i0 + size * j0];
	}
	inline real B(int size, std::vector<real>& d) {
		return s0*t1*d[i0 + size * j1];
	}
	inline real C(int size, std::vector<real>& d) {
		return s1*t0*d[i1 + size * j0];
	}
	inline real D(int size, std::vector<real>& d) {
		return s1*t1*d[i1 + size * j1];
	}
	inline real Eval(int size, std::vector<real>& d) {
		return s0*(t0*d[i0 + size * j0] + t1*d[i0 + size * j1]) + s1*(t0*d[i1 + size * j0] + t1*d[i1 + size * j1]);
	}

	inline void calc(int N, real i, real j) {
		if (i < 0.5f) i = 0.5f;
		if (i > N + 0.5f) i = N + 0.5f;

		i0 = (int) i;
		i1 = i0 + 1;

		if (j < 0.5f) j = 0.5f;
		if (j > N + 0.5f) j = N + 0.5f;

		j0 = (int) j;
		j1 = j0 + 1;

		s1 = i - i0;
		s0 = 1 - s1;
		t1 = j - j0;
		t0 = 1 - t1;
	}
};