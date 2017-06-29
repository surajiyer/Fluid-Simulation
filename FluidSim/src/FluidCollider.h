#pragma once
#include "Inc.h"

class FluidCollider
{
public:
	FluidCollider();
	~FluidCollider();

	virtual void FillGrid(int N, std::vector<byte> cellInfo) = 0;
};

class SquareCollider 
	: public FluidCollider 
{
public:
	SquareCollider(int ox, int oy, float w, float h, float r);

	virtual void FillGrid(int N, std::vector<byte> cellInfo) override;

	int ox, oy;
	float w, h;
	float rotation;
};