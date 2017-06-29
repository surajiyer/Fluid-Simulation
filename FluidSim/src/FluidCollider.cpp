#include "FluidCollider.h"



FluidCollider::FluidCollider()
{
}


FluidCollider::~FluidCollider()
{
}

SquareCollider::SquareCollider(int ox, int oy, float w, float h, float r)
{
	this->ox = ox;
	this->oy = oy;
	this->w = w;
	this->h = h;
}

void SquareCollider::FillGrid(int N, std::vector<byte> cellInfo)
{
	
}
