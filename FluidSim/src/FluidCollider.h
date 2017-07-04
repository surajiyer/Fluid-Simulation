#pragma once
#include "Inc.h"

class FluidCollider
{
public:
	FluidCollider();
	~FluidCollider();

	
	void Update(int N, FluidSystem*, std::vector<byte>& cellInfo, real dt);
	virtual void UpdateChild(int N, FluidSystem*, std::vector<byte>& cellInfo, real dt) = 0;

protected:
	Vec2 vel;

	Vec2 loc;
	Vec2 scale;
	real rot;
};

class RectCollider
	: public FluidCollider
{
public:
	RectCollider(int ox, int oy, real w, real h, real r);

	virtual void UpdateChild(int N, FluidSystem*, std::vector<byte>& cellInfo, real dt) override;
};