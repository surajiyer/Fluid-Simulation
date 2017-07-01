#pragma once
#include "Inc.h"

class FluidCollider
{
public:
	FluidCollider();
	~FluidCollider();

	virtual void FillGrid(int N, std::vector<byte>& cellInfo) = 0;
};

class RectCollider
	: public FluidCollider
{
public:
	RectCollider(int ox, int oy, real w, real h, real r);

	virtual void FillGrid(int N, std::vector<byte>& cellInfo) override;

	Vec2 loc;
	Vec2 scale;
	real rot;
};