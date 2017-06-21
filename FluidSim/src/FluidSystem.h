#pragma once
class FluidSystem
{
public:
	FluidSystem();
	~FluidSystem();
private:
	MatX densities;
	MatX velocities;

	enum NeighbourFlags : byte {
		TL = 1, T = 2, TR = 4, L = 8, R = 16, BL = 32, B = 64, BR = 128
	};
	byte[] flags;


};
